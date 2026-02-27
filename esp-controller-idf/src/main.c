#include "c_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void app_main() {

    bc_init_peripherals(1000000, 115200); // Initialize peripherals with a 1 MHz clock speed for the ADC
    xTaskCreatePinnedToCore(bc_bench_test_0, "bench_test_1", 32768, NULL, 5, NULL, 1);
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop can perform other tasks or sleep
    }

}
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/i2c_master.h"
// #include "esp_log.h"

// static const char *TAG = "PCA9685_MODERN";

// // Configuration Constants
// #define I2C_SCL_IO           22                
// #define I2C_SDA_IO           21                
// #define PCA9685_ADDR         0x41              

// // PCA9685 Register Map
// #define PCA9685_MODE1        0x00
// #define PCA9685_LED0_ON_L    0x06
// #define MODE1_RESTART        0x80
// #define MODE1_AI             0x20 

// static i2c_master_dev_handle_t g_dev_handle;

// /**
//  * @brief Set PWM duty cycle for a specific LED channel
//  * 
//  * @param channel LED channel number (0-15)
//  * @param duty_cycle Duty cycle as a percentage (0.0 to 100.0)
//  * @return esp_err_t ESP_OK on success
//  */
// esp_err_t pca9685_set_duty_cycle(uint8_t channel, float duty_cycle) {
//     if (channel > 15) {
//         ESP_LOGE(TAG, "Invalid channel: %d (must be 0-15)", channel);
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     if (duty_cycle < 0.0f || duty_cycle > 100.0f) {
//         ESP_LOGE(TAG, "Invalid duty cycle: %.2f (must be 0.0-100.0)", duty_cycle);
//         return ESP_ERR_INVALID_ARG;
//     }
    
//     // Convert duty cycle percentage to 12-bit value (0-4095)
//     uint16_t off_value = (uint16_t)((duty_cycle / 100.0f) * 4095.0f);

//     ESP_LOGI(TAG, "Setting channel %d to duty cycle %.2f%% (OFF value: %d)", channel, duty_cycle, off_value);
    
//     // Calculate register address for this channel
//     uint8_t base_reg = PCA9685_LED0_ON_L + (4 * channel);
    
//     // Build payload: [Base Register, ON_L, ON_H, OFF_L, OFF_H]
//     uint8_t payload[] = {
//         base_reg,
//         0x00,                    // ON Low byte (start at 0)
//         0x00,                    // ON High byte
//         (uint8_t)(off_value & 0xFF),      // OFF Low byte
//         (uint8_t)((off_value >> 8) & 0x0F) // OFF High byte (only lower 4 bits used)
//     };
    
//     return i2c_master_transmit(g_dev_handle, payload, sizeof(payload), -1);
// }

// void app_main(void) {
//     // 1. Initialize the I2C Bus Configuration
//     i2c_master_bus_config_t bus_config = {
//         .clk_source = I2C_CLK_SRC_DEFAULT,
//         .i2c_port = I2C_NUM_0,
//         .scl_io_num = I2C_SCL_IO,
//         .sda_io_num = I2C_SDA_IO,
//         .glitch_ignore_cnt = 7,
//         .flags.enable_internal_pullup = true,
//     };
//     i2c_master_bus_handle_t bus_handle;
//     ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &bus_handle));

//     // 2. Add the PCA9685 Device to the Bus
//     i2c_device_config_t dev_config = {
//         .dev_addr_length = I2C_ADDR_BIT_LEN_7,
//         .device_address = PCA9685_ADDR,
//         .scl_speed_hz = 100000,
//     };
//     i2c_master_dev_handle_t dev_handle;
//     ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_config, &dev_handle));
    
//     // Store handle globally for the abstraction function
//     g_dev_handle = dev_handle;

//     ESP_LOGI(TAG, "Modern I2C Bus and Device initialized");

//     // 3. Wake up/Reset the PCA9685
//     // We send: [Register Address, Data]
//     uint8_t wakeup_data[] = {PCA9685_MODE1, MODE1_RESTART | MODE1_AI};
//     ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, wakeup_data, sizeof(wakeup_data), -1));
//     vTaskDelay(pdMS_TO_TICKS(10)); 
//     while (true) {
        
//     // 4. Use the abstraction function to set duty cycles
//     ESP_LOGI(TAG, "Setting LED 0 to 25%% duty cycle");
//     ESP_ERROR_CHECK(pca9685_set_duty_cycle(0, 25.0f));

//     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit to see the effect
    
//     ESP_LOGI(TAG, "Setting LED 1 to 50%% duty cycle");
//     ESP_ERROR_CHECK(pca9685_set_duty_cycle(0, 50.0f));

//     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit to see the effect
    
//     ESP_LOGI(TAG, "Setting LED 2 to 75%% duty cycle");
//     ESP_ERROR_CHECK(pca9685_set_duty_cycle(0, 75.0f));

//     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit to see the effect
    
//     ESP_LOGI(TAG, "Setting LED 3 to 100%% duty cycle");
//     ESP_ERROR_CHECK(pca9685_set_duty_cycle(0, 100.0f));
//     vTaskDelay(pdMS_TO_TICKS(1000)); // Wait a bit to see the effect
//     }



//     ESP_LOGI(TAG, "All duty cycles set successfully!");

//     // Note: In a real app, you wouldn't usually delete these in main,
//     // but here is how you clean up properly:
//     // i2c_master_bus_rm_device(dev_handle);
//     // i2c_del_master_bus(bus_handle);
// }