#include "c_bridge.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/spi_master.h>
#include <comms/wifi_client.h>
#include "nvs_flash.h"
#include <driver/gpio.h>



void app_main() {

    // bc_init_peripherals(100000, 115200); // Initialize peripherals with a 5 MHz clock speed for the ADC
    // // xTaskCreate(udp_sender_task, "udp_sender", 4096, NULL, 5, NULL);


    bc_run_state_machine_testing();




}
