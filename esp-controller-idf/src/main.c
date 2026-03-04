#include "c_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>



void app_main() {

    bc_init_peripherals(1000000, 115200); // Initialize peripherals with a 5 MHz clock speed for the ADC
    xTaskCreatePinnedToCore(bc_bench_test_1, "bench_test_stress_20ms", 32768, NULL, 5, NULL, 1);
    // printf("Started magnet step benchmark test on core 1\n");
    
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop can perform other tasks or sleep
    }


}
