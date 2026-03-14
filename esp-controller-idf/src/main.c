#include "c_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include <comms/wifi_client.h>
#include "nvs_flash.h"



void app_main() {

    
    

    // 3. Start the Data Task
    // xTaskCreatePinnedToCore(bc_bench_test_imu, "udp_sender", 32768, NULL, 5, NULL, 1);

    // while (true) {
    //     vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop sleeps, task runs independently
    // }
    bc_run_state_machine_testing();


}
