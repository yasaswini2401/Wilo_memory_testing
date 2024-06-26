#include "cppspi/cppspi.h"
#include <iostream>
#include "scanchain_spi/scanchain_spi.h"
#include "string.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#define UART_NUM UART_NUM_0
#define UART_RX_PIN 44

constexpr static int SPI_3_MISO = 45; // Changing to scanout
constexpr static int SPI_3_MOSI = 38;
constexpr static int SPI_3_SCLK = 43;
constexpr static int SS_PIN = 47;
constexpr static int SC_RESET = 36;
constexpr static int SC_FEED = 0;
constexpr static int CLK_IN_DIAG = 39;
constexpr static int LIN_RX_ON = 4;

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

bool SCAN_OUT[] = {0};
bool MEM_DATA_DIAG[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
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
bool CDR_TRIM[] = {0, 0};
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

void write_scan_chain(bool send_data[80]) {
    // Sweep through all arrays and send one bit at a time
//     for (int i = 0; i < NUM_ARRAYS; ++i) {
//         sendArray(arrays[i], arraySizes[i]);
//     }

     sendArray(send_data, 80);

//     vTaskDelay(pdMS_TO_TICKS(unit_delay));  // Adjust delay between transmissions as needed
     // Write to scan feed
     // gpio_set_level(gpio_num_t(SC_FEED), 1);
     // vTaskDelay(pdMS_TO_TICKS(unit_delay));
     gpio_set_level(gpio_num_t(SC_FEED), 0);
     vTaskDelay(pdMS_TO_TICKS(5));
     gpio_set_level(gpio_num_t(SC_FEED), 1); 
     vTaskDelay(pdMS_TO_TICKS(2));
     gpio_set_level(gpio_num_t(SC_FEED), 0);
     vTaskDelay(pdMS_TO_TICKS(10));
     //gpio_set_level(gpio_num_t(SC_FEED), 1); 

}




//Other code
constexpr static int ME_PC_B = 33; //preamb ind at 3
constexpr static int ME_SE = 35;
constexpr static int ME_WE = 34;
constexpr static int ME_OUT_SEL_0 = 47;
constexpr static int ME_OUT_SEL_1 = 48;
constexpr static int ME_OUT_SEL_2 = 26;
constexpr static int ME_OUT_SEL_3 = 21;

void write_bits(){
     
     gpio_set_level(gpio_num_t(ME_PC_B), 1);
     gpio_set_level(gpio_num_t(ME_SE), 0);
     gpio_set_level(gpio_num_t(ME_WE), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_0), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_1), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_2), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_3), 0);

     vTaskDelay(pdMS_TO_TICKS(10));

     gpio_set_level(gpio_num_t(ME_PC_B), 0);

     vTaskDelay(pdMS_TO_TICKS(10));

     gpio_set_level(gpio_num_t(ME_PC_B), 1);
     gpio_set_level(gpio_num_t(ME_WE), 1);
     
     vTaskDelay(pdMS_TO_TICKS(10));

     gpio_set_level(gpio_num_t(ME_WE), 0);

     vTaskDelay(pdMS_TO_TICKS(10));

}

void read_bits(){
     
     gpio_set_level(gpio_num_t(ME_PC_B), 1);
     gpio_set_level(gpio_num_t(ME_SE), 0);
     gpio_set_level(gpio_num_t(ME_WE), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_0), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_1), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_2), 0);
     gpio_set_level(gpio_num_t(ME_OUT_SEL_3), 0);

     vTaskDelay(pdMS_TO_TICKS(10));

     gpio_set_level(gpio_num_t(ME_PC_B), 0);
     gpio_set_level(gpio_num_t(ME_SE), 0);
     gpio_set_level(gpio_num_t(ME_WE), 0);

     vTaskDelay(pdMS_TO_TICKS(10));

     gpio_set_level(gpio_num_t(ME_PC_B), 1);
     gpio_set_level(gpio_num_t(ME_SE), 1);

     vTaskDelay(pdMS_TO_TICKS(10));

     gpio_set_level(gpio_num_t(ME_SE), 0);

     // vTaskDelay(pdMS_TO_TICKS(20));

     for (int sel = 0; sel < 16; ++sel) {
        gpio_set_level(gpio_num_t(ME_OUT_SEL_0), (sel & 0b0001) ? 1 : 0);
        gpio_set_level(gpio_num_t(ME_OUT_SEL_1), (sel & 0b0010) ? 1 : 0);
        gpio_set_level(gpio_num_t(ME_OUT_SEL_2), (sel & 0b0100) ? 1 : 0);
        gpio_set_level(gpio_num_t(ME_OUT_SEL_3), (sel & 0b1000) ? 1 : 0);

        vTaskDelay(pdMS_TO_TICKS(10));
    }

}



extern "C" void app_main(void)
{  

     // Disable UART before configuring the pin
    uart_driver_delete(UART_NUM); 

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

    io_conf.pin_bit_mask = (1ULL << SC_RESET);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);

     gpio_set_level(gpio_num_t(SC_RESET), 0);


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

     // constexpr static int ME_PC_B = 33; //preamb ind at 3
     // constexpr static int ME_SE = 35;
     // constexpr static int ME_WE = 34;
     // constexpr static int ME_OUT_SEL_0 = 47;
     // constexpr static int ME_OUT_SEL_1 = 48;
     // constexpr static int ME_OUT_SEL_2 = 26;
     // constexpr static int ME_OUT_SEL_3 = 21;

     io_conf.pin_bit_mask = (1ULL << ME_PC_B);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
     gpio_config(&io_conf);
     gpio_set_level(gpio_num_t(ME_PC_B), 1);

     io_conf.pin_bit_mask = (1ULL << ME_SE);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     // io_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
     io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // Ensure no pull-up
     io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; // Ensure no pull-down    
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(ME_SE), 0);

     io_conf.pin_bit_mask = (1ULL << ME_WE);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(ME_WE), 0);

     io_conf.pin_bit_mask = (1ULL << ME_OUT_SEL_0);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(ME_OUT_SEL_0), 0);

     io_conf.pin_bit_mask = (1ULL << ME_OUT_SEL_1);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(ME_OUT_SEL_1), 0);

     io_conf.pin_bit_mask = (1ULL << ME_OUT_SEL_2);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(ME_OUT_SEL_2), 0);

     io_conf.pin_bit_mask = (1ULL << ME_OUT_SEL_3);
     io_conf.mode = GPIO_MODE_OUTPUT;
     io_conf.intr_type = GPIO_INTR_DISABLE;
     io_conf.pull_up_en = GPIO_PULLUP_ENABLE;     
     gpio_config(&io_conf);  
     gpio_set_level(gpio_num_t(ME_OUT_SEL_3), 0);



     bool** send_data = (bool**)malloc(192 * sizeof(bool*));
    for(int i=0; i < 192; i++){
        send_data[i] = (bool*)malloc(80*sizeof(bool));
    }

    // bool zero_to_four[5] = {1, 0, 1, 0, 1};
    // bool eleven_to_fifteen[5] = {0, 1, 0, 1, 0};
    bool last[55] = {1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    for(int i=0; i<192; i++){

        int a = i % 64;

        bool binary[6] = {0, 0, 0, 0, 0, 0};
        int place = 5;
        while(a > 0){
            binary[place] = a % 2;
            a = a / 2;
            place = place - 1;
        }

        int b = i;
        bool binary_2[10] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1};

        place = 9;
        while(b > 0){
            binary_2[place] = (b % 2);
            b = b / 2;
            place = place - 1;
        }

        int tt_tf;
        if (i <= 63) tt_tf = 1;
        else if (i > 63 && i <= 127) tt_tf = 2;
        else tt_tf = 4;

        bool tt_tf_array[3] = {0, 0, 0};
        place = 2;
        while(tt_tf > 0){
            tt_tf_array[place] = tt_tf % 2;
            tt_tf = tt_tf / 2;
            place = place - 1;
        }

        for(int j=0; j<80; j++){

            if (j<=5)
            {
                send_data[i][j] = binary[j];
            }
            else if (j>5 && j<=15)
            {
                send_data[i][j] = binary_2[j-6];
            }
            else if (j>15 and j<=21)
            {
                send_data[i][j] = binary[j-16];
            }
            else if (j>21 and j<=24){
                send_data[i][j] = tt_tf_array[j-22];
            }
            else if (j>24 and j<=79){
                send_data[i][j] = last[j-25];
            }
        }
    }

     int i = 0;

     while(i < 192){
          write_scan_chain(send_data[i]);
          vTaskDelay(pdMS_TO_TICKS(10)); 
          write_bits();
          vTaskDelay(pdMS_TO_TICKS(10)); 
          read_bits();
          vTaskDelay(pdMS_TO_TICKS(100));

          i = i + 1; // Increment the index and wrap around to 0 when it reaches 192
     }
}
