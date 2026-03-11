#include "c_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include <comms/wifi_client.h>
#include "nvs_flash.h"



void app_main() {

    bc_run_state_machine_testing();


}
