#include "ota_update.h"
#include "esp_ota_ops.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "utils/utils.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define OTA_PORT 8080
#define OTA_BUFFSIZE 1024

static const char *OTA_TAG = "OTA";

static esp_err_t ota_post_handler(httpd_req_t *req)
{
    const esp_partition_t *update_partition = esp_ota_get_next_update_partition(NULL);
    if (update_partition == NULL)
    {
        ESP_LOGE(OTA_TAG, "No OTA partition found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "No OTA partition");
        return ESP_FAIL;
    }

    ESP_LOGI(OTA_TAG, "Writing to partition '%s' at offset 0x%lx",
             update_partition->label, (unsigned long)update_partition->address);

    esp_ota_handle_t ota_handle;
    esp_err_t err = esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(OTA_TAG, "esp_ota_begin failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA begin failed");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    int received = 0;
    char buf[OTA_BUFFSIZE];

    serial_printf("OTA: Receiving %d bytes...\n", remaining);

    while (remaining > 0)
    {
        int to_read = remaining < OTA_BUFFSIZE ? remaining : OTA_BUFFSIZE;
        int recv_len = httpd_req_recv(req, buf, to_read);
        if (recv_len <= 0)
        {
            if (recv_len == HTTPD_SOCK_ERR_TIMEOUT)
            {
                continue;
            }
            ESP_LOGE(OTA_TAG, "Receive error");
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        err = esp_ota_write(ota_handle, buf, recv_len);
        if (err != ESP_OK)
        {
            ESP_LOGE(OTA_TAG, "esp_ota_write failed: %s", esp_err_to_name(err));
            esp_ota_abort(ota_handle);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA write failed");
            return ESP_FAIL;
        }

        remaining -= recv_len;
        received += recv_len;

        // Progress every ~10%
        int total = req->content_len;
        if (total > 0)
        {
            int pct = received * 100 / total;
            int prev_pct = (received - recv_len) * 100 / total;
            if (pct / 10 != prev_pct / 10)
            {
                serial_printf("OTA: %d/%d bytes (%d%%)\n", received, total, pct);
            }
        }
    }

    err = esp_ota_end(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(OTA_TAG, "esp_ota_end failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA validation failed");
        return ESP_FAIL;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK)
    {
        ESP_LOGE(OTA_TAG, "esp_ota_set_boot_partition failed: %s", esp_err_to_name(err));
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Set boot partition failed");
        return ESP_FAIL;
    }

    serial_printf("OTA: Success! Received %d bytes. Rebooting...\n", received);
    httpd_resp_sendstr(req, "OTA update successful. Rebooting...");

    // Delay to let the HTTP response be sent before rebooting
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    return ESP_OK; // unreachable
}

void startOtaUpdateTask()
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = OTA_PORT;
    config.stack_size = 8192;
    config.recv_wait_timeout = 30; // 30s timeout for slow WiFi transfers

    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t ota_uri = {
            .uri = "/update",
            .method = HTTP_POST,
            .handler = ota_post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &ota_uri);
        serial_printf("OTA server started on port %d\n", OTA_PORT);
    }
    else
    {
        serial_print("ERROR: Failed to start OTA server\n");
    }
}
