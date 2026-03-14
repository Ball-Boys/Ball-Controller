#include "c_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include <comms/wifi_client.h>
#include "nvs_flash.h"
#include <driver/gpio.h>



void app_main() {
    // gpio_reset_pin(GPIO_NUM_13);
    // gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    
    //     gpio_set_level(GPIO_NUM_13, 1);
        // vTaskDelay(pdMS_TO_TICKS(500));
        // gpio_set_level(GPIO_NUM_34, 0);
        // vTaskDelay(pdMS_TO_TICKS(500));
    bc_init_peripherals(100000, 115200); // Initialize peripherals with a 5 MHz clock speed for the ADC
    // // xTaskCreate(udp_sender_task, "udp_sender", 4096, NULL, 5, NULL);

    // xTaskCreatePinnedToCore(bc_bench_test_0, "bench_test_stress_20ms", 32768, NULL, 5, NULL, 1);
    // // printf("Started magnet step benchmark test on core 1\n");
    
    // while (true) {
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop can perform other tasks or sleep
    // }


    // wifi testing 
    
    

    // 3. Start the Data Task
    // xTaskCreatePinnedToCore(bc_bench_test_imu, "udp_sender", 32768, NULL, 5, NULL, 1);

    // while (true) {
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop sleeps, task runs independently
    // }
    bc_run_state_machine_testing();


}
