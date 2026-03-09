#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "core/peripherals.h"
#include "comms/data_conversion_layer.h"




void udp_sender_task() {
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(RECV_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    
    static ball_data_packet out_data;

    while (1) {
        extract_data_from_globals(&out_data);
        sendto(sock, &out_data, sizeof(out_data), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        vTaskDelay(pdMS_TO_TICKS(100));  // Send every 100ms (10Hz)
    }
}

