#include <string.h>
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "core/peripherals.h"
#include "core/global_state.h"
#include "comms/data_conversion_layer.h"
#include "utils/utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define COMMAND_PORT 5006

// Command packet structure from dashboard to ESP32
typedef struct __attribute__((packed))
{
    uint8_t command_type;     // 0=set_direction, 1=calibrate, 2=emergency_stop, 3=start_running
    float ideal_direction_x;  // X component of desired direction
    float ideal_direction_y;  // Y component of desired direction
    float ideal_direction_z;  // Z component of desired direction
    uint32_t sequence_number; // For acknowledgment tracking
} DashboardCommand;

void udp_sender_task(void *pvParameters)
{
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(RECV_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    static ball_data_packet out_data;

    while (1)
    {
        extract_data_from_globals(&out_data);
        sendto(sock, &out_data, sizeof(out_data), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send at 10Hz
    }
}

// ============================================================
// COMMAND RECEIVER (Dashboard -> ESP32)
// ============================================================

void process_dashboard_command(const DashboardCommand *cmd)
{
    if (cmd == nullptr)
        return;

    GlobalState &state = GlobalState::instance();

    switch (cmd->command_type)
    {
    case 0: // Set ideal direction for steering (or calibration input during calibration)
    {
        Vector3 direction(cmd->ideal_direction_x,
                          cmd->ideal_direction_y,
                          cmd->ideal_direction_z);

        // Route to calibration or normal steering based on system state
        if (state.getSystemState() == GlobalState::SystemState::CALIBRATION)
        {
            state.setCalibrationInput(direction);
            serial_printf("RX: Calibration input (%.2f, %.2f, %.2f)\n",
                          cmd->ideal_direction_x, cmd->ideal_direction_y, cmd->ideal_direction_z);
        }
        else
        {
            state.setIdealDirection(direction);
            serial_printf("RX: Set direction (%.2f, %.2f, %.2f)\n",
                          cmd->ideal_direction_x, cmd->ideal_direction_y, cmd->ideal_direction_z);
        }
    }
    break;

    case 1: // Trigger calibration
    {
        serial_print("RX: Calibrate command\n");
        state.requestCalibration();
    }
    break;

    case 2: // Emergency stop
    {
        state.set_kill(true);
        serial_print("RX: EMERGENCY STOP\n");
    }
    break;

    case 3: // Start running (clear kill flag and request start)
    {
        state.set_kill(false);
        state.requestStart();
        serial_print("RX: Start running\n");
    }
    break;

    default:
        serial_printf("RX: Unknown command type %d\n", cmd->command_type);
        break;
    }
}

void udp_receiver_task(void *pvParameters)
{
    // Wait for WiFi to stabilize
    vTaskDelay(pdMS_TO_TICKS(10));

    // Create socket for receiving commands
    struct sockaddr_in listen_addr;
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    listen_addr.sin_port = htons(COMMAND_PORT);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0)
    {
        serial_print("ERROR: Failed to create command receiver socket\n");
        vTaskDelete(NULL);
        return;
    }

    int err = bind(sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr));
    if (err < 0)
    {
        serial_print("ERROR: Failed to bind command receiver socket\n");
        closesocket(sock);
        vTaskDelete(NULL);
        return;
    }

    // Set socket timeout so we don't block forever
    struct timeval tv;
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    serial_print("Command receiver started on port 5006\n");

    static DashboardCommand cmd;
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);

    while (true)
    {
        int len = recvfrom(sock, (void *)&cmd, sizeof(DashboardCommand), 0,
                           (struct sockaddr *)&source_addr, &socklen);

        if (len > 0)
        {
            // Process the command
            process_dashboard_command(&cmd);
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    closesocket(sock);
    vTaskDelete(NULL);
}