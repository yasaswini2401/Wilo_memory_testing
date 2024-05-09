#include "cppspi/cppspi.h"
#include <stdio.h>
#include <iostream>
#include "scanchain_spi/scanchain_spi.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "driver/periph_ctrl.h"
#include "soc/timer_group_struct.h"
#include "driver/gptimer.h"

#define UART_NUM UART_NUM_0
#define UART_RX_PIN 44
#define TIMER_DIVIDER 40 // 80MHz / 80 = 1MHz
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // Timer scale to microseconds
#define OUTPUT_DEMOD_IN_DIAG GPIO_NUM_1  // GPIO pin for DEMOD_IN_DIAG
#define OUTPUT_SIG_IN_DIAG GPIO_NUM_2  // GPIO pin for Preamble_indicator
#define OUTPUT_Preamble_indicator GPIO_NUM_3  // GPIO pin for Preamble_indicator
#define SEQUENCE_LENGTH   41        // Length of bit sequence

constexpr static int SPI_3_MISO = 45; // Changing to scanout
constexpr static int SPI_3_MOSI = 38;
constexpr static int SPI_3_SCLK = 43;
constexpr static int SS_PIN = 47;
constexpr static int GND = 44;
constexpr static int SC_RESET = 36;
constexpr static int SC_FEED = 0;
constexpr static int CLK_IN_DIAG = 39;
constexpr static int LIN_RX_ON = 4;
// constexpr static int PREAMBLE_IND = 3;

// Data to send (example)
// static const uint8_t sequence0[SEQUENCE_LENGTH] = {1,0,1,0,1,1,1,1,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,0}; //{0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0};
// static const uint8_t sequence1[SEQUENCE_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static const uint8_t sequence0[SEQUENCE_LENGTH] = {1,0,1,0,1,1,1,1,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,1,0,1,1,0,1,1,0,1,1,0,1,0,1,0,1,1,0};//case 2
static const uint8_t sequence1[SEQUENCE_LENGTH] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1};//case 2
static const uint8_t sequence2[SEQUENCE_LENGTH] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// Board configuration
// J19: RF 1uA current reference enable
// J18: Disable

// J33: Ring Oscillator current source enable
// J32: Ring Oscillator current source disable

// Make sure to connect the 1.2V LDO output to the daughter card for level shifters

int MEM_DATA_DIAG_Size = 16;
int MEM_ADDR_DIAG_Size = 6;
int MEM_SEL_DIAG_Size = 3;
int MC_Debug_addr_Size = 4;
int SENS_CODE_DIAG_Size = 3;
int DIG_MUX_SEL_Size = 4;
int PLL_OUT_EN_Size = 2;
int CRC_PRECODE_Size = 7;
int CDR_TRIM_Size = 2;
int UNCONNECTED_Size = 6;
int RF_TRIM_Size = 10;
int sequence_end = 0;
gptimer_handle_t gptimer = NULL;

bool SCAN_OUT[] = {0};
bool MEM_DATA_DIAG[] = {1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool MEM_ADDR_DIAG[] = {0, 0, 0, 0, 0, 0};
bool MEM_SEL_DIAG[] = {0, 0, 0};
bool MEM_DIAG_MODE[] = {1};
bool UNCONNECTED_1[] = {0};
bool EXT_2M_CLK_EN[] = {0};
bool MC_Debug_addr[] = {0, 0, 0, 0};
bool BYPASS_RF[] = {1};
bool SENS_CODE_DIAG[] = {0, 0, 0};
bool ADC_DIAG_MODE[] = {0};
bool DIG_MUX_SEL[] = {1, 1, 0, 0};
bool PLL_OUT_EN[] = {1, 0};
bool OSC_CONTROL_diag[] = {1};
bool calibration_control_diag[] = {0};
bool PRECODE_CTRL[] = {1};
bool CRC_PRECODE[] = {0, 0, 0, 0, 0, 0, 0};
bool CRC_DIAG_MODE[] = {1};
bool CDR_TRIM[] = {0, 1};
bool IEB1[] = {0};
bool IEB2[] = {0};
bool UNCONNECTED[] = {0, 0, 0, 0, 0, 0};
bool Analog_Diag_Ctrl[] = {0};
bool RF_TRIM[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
bool RF_DIAG_MODE[] = {1};
bool TX_EN_diag[] = {0};
bool morb_trans_diag[] = {1};
bool rforbase_diag[] = {1};
bool pll_enable_diag[] = {1};

// Define the number of arrays and their sizes
#define NUM_ARRAYS 31 // Update this if you add or remove arrays

// Define sizes of each array
const int arraySizes[NUM_ARRAYS] = {16, 6, 3, 1, 1, 1, 4, 1, 3, 1, 4, 2, 1, 1, 1, 7, 1, 2, 1, 1, 6, 1, 10, 1, 1, 1, 1, 1, 1, 1};

// Define the integer arrays
bool* arrays[NUM_ARRAYS] = {MEM_DATA_DIAG, MEM_ADDR_DIAG, MEM_SEL_DIAG, MEM_DIAG_MODE, UNCONNECTED_1, EXT_2M_CLK_EN, MC_Debug_addr, BYPASS_RF, SENS_CODE_DIAG, ADC_DIAG_MODE, DIG_MUX_SEL, PLL_OUT_EN, OSC_CONTROL_diag, calibration_control_diag, PRECODE_CTRL, CRC_PRECODE, CRC_DIAG_MODE, CDR_TRIM, IEB1, IEB2, UNCONNECTED, Analog_Diag_Ctrl, RF_TRIM, RF_DIAG_MODE, TX_EN_diag, morb_trans_diag, rforbase_diag, pll_enable_diag};

//<<<<<Aligning>>>>>><MEM_DATA_DIAG>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  <MEM_ADDR_DIAG>> <MEM_SEL> <M_EN><N><EXT> <MC_DEG_A><RFB><SEN_CO><ADC><DIG_MUX_S><PLL_O><OSC><CAL><P><<<CRC_PRECODE>>>>>><CD><CDR_T><IE12><<<NC_NC_NC_6>>>><AD><RF_TRIM>>>>>>>>>>>>>>>>>>>>><RD><TX><MOB><ROB><PL_EN>
bool send_data[80] = {0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0,    1,  0,  0, 0, 0, 0,  1, 0, 0, 0, 1,  0, 0, 1, 1, 0, 0, 1,   1,   0, 1, 1, 1, 1, 1, 1, 1, 0, 0,  1, 0, 1, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1,  1,    1,   1};

//

// 4nA current reference
// IEB2 = 0 -> Make a connection to external pin for use with current mirror
// IEB2 = 1 -> Disconnects external pin from internal current mirror

// IEB1 = 0 -> Make a connection with IPTAT to internal current mirror
// IEB1 = 1 -> Disconnects IPTAT to internal current mirror

// RF Trim Bits
// RF Trim <9:0>:  <HF_AMP_TRIM><LC_OSC_Trim><NL_AMP_TRIM><4nA_REF_TRIM<1:0>><LC_OSC_1bit_trim_cap_EN><BB_Comparator Offset<3:0>>

const int unit_delay = 10;

#ifdef CONFIG_IDF_TARGET_ESP32
#define SENDER_HOST HSPI_HOST
#else
#define SENDER_HOST SPI2_HOST
#endif
CPPSPI::Spi spi3;

void sendArray(bool* array, int size) {
     // Send all bits of the array serially
     for (int i = 0; i < size; ++i) {
          bool value = array[i];

          gpio_set_level(gpio_num_t(SPI_3_SCLK), 0);
//          vTaskDelay(pdMS_TO_TICKS(unit_delay));
          gpio_set_level(gpio_num_t(SPI_3_SCLK), 1);
//          vTaskDelay(pdMS_TO_TICKS(unit_delay));
          gpio_set_level(gpio_num_t(SPI_3_MOSI), value);
//          vTaskDelay(pdMS_TO_TICKS(unit_delay));                           
          //    for (int j = 0; j < sizeof(int) * 8; ++j) {
          //        sendBit((value >> j) & 1);
          //    }
     }
}

void write_scan_chain() {
    // Sweep through all arrays and send one bit at a time
//     for (int i = 0; i < NUM_ARRAYS; ++i) {
//         sendArray(arrays[i], arraySizes[i]);
//     }
     sendArray(send_data, 80);

//     vTaskDelay(pdMS_TO_TICKS(unit_delay));  // Adjust delay between transmissions as needed
     // Write to scan feed
//     gpio_set_level(gpio_num_t(SC_FEED), 1);
//     vTaskDelay(pdMS_TO_TICKS(unit_delay));
     gpio_set_level(gpio_num_t(SC_FEED), 0);
     vTaskDelay(pdMS_TO_TICKS(5));
     gpio_set_level(gpio_num_t(SC_FEED), 1); 
     vTaskDelay(pdMS_TO_TICKS(2));
     gpio_set_level(gpio_num_t(SC_FEED), 0);
     vTaskDelay(pdMS_TO_TICKS(5));
     gpio_set_level(gpio_num_t(SC_FEED), 1); 

}

//////////////////////////////////////////////////////////


// Function prototype for the timer ISR
static bool IRAM_ATTR onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data);
// Configure GPTIMER
void configureGPTimer() {
     gptimer_config_t timer_config = {
     .clk_src = GPTIMER_CLK_SRC_DEFAULT,
     .direction = GPTIMER_COUNT_UP,
     .resolution_hz = 2000000, // This entry corresponds to the timer frequency.
     };
     gptimer_event_callbacks_t cbs = {
     .on_alarm = onTimer, // register user callback
     };
     ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
     ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, nullptr ));
     ESP_ERROR_CHECK(gptimer_enable(gptimer));
     gptimer_alarm_config_t alarm_config = {
          .alarm_count = 25, // period = 12.5us @resolution 2MHz
          .reload_count = 0, // counter will reload with 0 on alarm event
     };
     alarm_config.flags.auto_reload_on_alarm = true;
     ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
     ESP_ERROR_CHECK(gptimer_start(gptimer));
}
// Timer ISR
static bool IRAM_ATTR onTimer(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    // Send the next bit
    static uint8_t index = 0;
    gpio_set_level(OUTPUT_DEMOD_IN_DIAG, sequence0[index]);
	gpio_set_level(OUTPUT_SIG_IN_DIAG, sequence1[index]);
	gpio_set_level(OUTPUT_Preamble_indicator, sequence2[index]);
    // Increment index and wrap around if necessary
    index = (index + 1) % SEQUENCE_LENGTH;
    if(index == 0 ){
        sequence_end = 1;
        gpio_set_level(OUTPUT_DEMOD_IN_DIAG, 0);

    }
    else{
        //sequence_end = 0;
    }
    return 0;
}

extern "C" void app_main(void)
{  

     // Disable UART before configuring the pin
    uart_driver_delete(UART_NUM);

       // Configure GPIO pin as output
    gpio_config_t io_conf;

//     esp_err_t ret;

// //   Can Removing the scan chain serial interface, instead sending bits using GPIO functions
//     SCANCHAINSPI::scanchainDevice scanchainSpi;

// Setting up the pins for Scan chain as GPIO:
     io_conf.pin_bit_mask = (1ULL << SPI_3_MOSI);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
     gpio_config(&io_conf);
     gpio_set_level(gpio_num_t(SPI_3_MOSI), 1);

     io_conf.pin_bit_mask = (1ULL << SPI_3_SCLK);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(SPI_3_SCLK), 1);

     // initialise the spi bus
     // ret = spi3.Init(SENDER_HOST, SPI_3_MISO, SPI_3_MOSI, SPI_3_SCLK);
     // assert(ret == ESP_OK);

     // // registered the device .
     // ret = scanchainSpi.InitSpi(&spi3 ,SS_PIN,1000000);
     // assert(ret == ESP_OK);



     io_conf.pin_bit_mask = (1ULL << GND);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     gpio_config(&io_conf);

     // Set GPIO level to 0 (ground)
     gpio_set_level(gpio_num_t(GND), 0);

    io_conf.pin_bit_mask = (1ULL << SC_RESET);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

     gpio_set_level(gpio_num_t(SC_RESET), 0);

     //gpio_config_t io_conf;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.pin_bit_mask = (1ULL << OUTPUT_DEMOD_IN_DIAG);
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     gpio_config(&io_conf);

     //gpio_config_t io_conf;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.pin_bit_mask = (1ULL << OUTPUT_SIG_IN_DIAG);
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     gpio_config(&io_conf);

     //std::string send_data = "0b01010101"; // 12-bit data to send

     // // Two consecutive bits had to be 1. This means there are some issues with the setup / hold time? Maybe change the edge?
     // int send_data[] = {0, 0, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1};
     // uint8_t nbits = 80; // Number of bits to send

     // Make sure to feed scan chain through the program button
     io_conf.pin_bit_mask = (1ULL << SC_FEED);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     gpio_config(&io_conf);
     gpio_set_level(gpio_num_t(SC_FEED), 1);

// CDR settings

     io_conf.pin_bit_mask = (1ULL << OUTPUT_Preamble_indicator );
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
     gpio_config(&io_conf);
     gpio_set_level(gpio_num_t(OUTPUT_Preamble_indicator), 1);


//      gpio_pad_select_gpio(SPI_3_MOSI);
//     gpio_set_direction(gpio_num_t(SPI_3_MOSI), GPIO_MODE_OUTPUT);
    write_scan_chain();
    vTaskDelay(pdMS_TO_TICKS(100));  // Adjust delay between transmissions as needed

    configureGPTimer();

    while(1){

     // ret = spi3.Init(SENDER_HOST, SPI_3_MISO, SPI_3_MOSI, SPI_3_SCLK);
     // assert(ret == ESP_OK);

     // // registered the device .
     // ret = scanchainSpi.InitSpi(&spi3 ,SS_PIN,1000000);
     // assert(ret == ESP_OK);

//     spi3.sendNBitData(send_data, nbits);

//     gpio_set_level(gpio_num_t(SPI_3_MOSI), 1);

//     vTaskDelay(pdMS_TO_TICKS(5));  // Adjust delay between transmissions as needed
     
//     vTaskDelay(pdMS_TO_TICKS(10));

//     gpio_set_level(gpio_num_t(SPI_3_MOSI), 1);

//     vTaskDelay(pdMS_TO_TICKS(5));
     // gpio_set_level(gpio_num_t(SC_FEED), 0);
     // vTaskDelay(pdMS_TO_TICKS(10));
     // gpio_set_level(gpio_num_t(SC_FEED), 1); 
     // vTaskDelay(pdMS_TO_TICKS(10));
     // gpio_set_level(gpio_num_t(SC_FEED), 0);
     // vTaskDelay(pdMS_TO_TICKS(10));
     // gpio_set_level(gpio_num_t(SC_FEED), 1); 


      if (sequence_end == 1){
               ESP_ERROR_CHECK(gptimer_stop(gptimer));
               ESP_ERROR_CHECK(gptimer_disable(gptimer));
               ESP_ERROR_CHECK(gptimer_del_timer(gptimer));
               // break;
               while(1){

               }
          }

     }

     // Sending external clock signal to test various blocks
     io_conf.pin_bit_mask = (1ULL << CLK_IN_DIAG);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     gpio_config(&io_conf);

     gpio_set_level(gpio_num_t(CLK_IN_DIAG), 1);

     // while(1){

     //           gpio_set_level(gpio_num_t(CLK_IN_DIAG), 1);
     //           vTaskDelay(pdMS_TO_TICKS(10));  // Adjust delay between transmissions as needed
     //           gpio_set_level(gpio_num_t(CLK_IN_DIAG), 0);
     //           vTaskDelay(pdMS_TO_TICKS(10));  // Adjust delay between transmissions as needed

     // }

     // Setting up linear receiver to turn on

     /*io_conf.pin_bit_mask = (1ULL << LIN_RX_ON);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     gpio_config(&io_conf);

     gpio_set_level(gpio_num_t(LIN_RX_ON), 1);*/

}
