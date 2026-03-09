#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <unordered_map>
#include <driver/gpio.h>

#include "peripherals.h"
#include <utils/utils.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include <cstring>




namespace {
static bool s_adc_bus_initialized = false;
static int s_adc_clock_hz = 10000000; // Default to 10 MHz
static std::unordered_map<int, spi_device_handle_t> s_adc_devices;

static bool s_i2c_initialized = false;
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static std::unordered_map<int, i2c_master_dev_handle_t> s_pwm_devices;

// BNO08x IMU static variables
static bool s_imu_initialized = false;
static i2c_master_dev_handle_t s_imu_device = nullptr;
static constexpr uint8_t BNO08X_I2C_ADDR = 0x4A;  // Default I2C address
static constexpr uint8_t BNO08X_I2C_ADDR_ALT = 0x4B;  // Alternate address

// BNO08x SH-2 Channel and registers
static constexpr uint8_t CHANNEL_COMMAND = 0;
static constexpr uint8_t CHANNEL_EXECUTABLE = 1;
static constexpr uint8_t CHANNEL_CONTROL = 2;
static constexpr uint8_t CHANNEL_REPORTS = 3;
static constexpr uint8_t CHANNEL_WAKE_REPORTS = 4;
static constexpr uint8_t CHANNEL_GYRO = 5;
static uint8_t s_shtp_tx_seq[6] = {0, 0, 0, 0, 0, 0};

// Sensor report IDs
static constexpr uint8_t SENSOR_REPORTID_ROTATION_VECTOR = 0x05;
static constexpr uint8_t SENSOR_REPORTID_GYROSCOPE = 0x02;
static constexpr uint8_t SENSOR_REPORTID_ACCELEROMETER = 0x01;
static constexpr uint8_t SENSOR_REPORTID_MAGNETIC_FIELD = 0x03;

// Cached sensor data
struct IMUSensorData {
    float quat_w = 1.0f, quat_x = 0.0f, quat_y = 0.0f, quat_z = 0.0f;
    float gyro_x = 0.0f, gyro_y = 0.0f, gyro_z = 0.0f;
    float accel_x = 0.0f, accel_y = 0.0f, accel_z = 0.0f;
    float mag_x = 0.0f, mag_y = 0.0f, mag_z = 0.0f;
    bool quat_valid = false;
    bool gyro_valid = false;
    bool accel_valid = false;
    bool mag_valid = false;
    uint32_t last_update_ms = 0;
};

static IMUSensorData s_imu_data;
}

void init_adc(int clock_speed_hz, gpio_num_t chip_select_pin) {

    // Configure CS as GPIO output (manual control)
    gpio_config_t io_cfg = {
        .pin_bit_mask = 1ULL << chip_select_pin,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&io_cfg);

    gpio_set_level(chip_select_pin, 1); // Set CS high (inactive)

    // Initialize SPI bus only once
    if (!s_adc_bus_initialized) {
        // SPI bus config
        spi_bus_config_t buscfg = {
            .mosi_io_num = ADC_MOSI_PIN,
            .miso_io_num = ADC_MISO_PIN,
            .sclk_io_num = ADC_SLK_PIN,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 4,
        };

        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED));
        s_adc_bus_initialized = true;
    }

    // Device config
    spi_device_interface_config_t devcfg = {
        .mode = 0,
        .clock_speed_hz = clock_speed_hz,
        .spics_io_num = -1,  
        .flags = SPI_DEVICE_NO_DUMMY,
        .queue_size = 1,
    };

    spi_device_handle_t adc_handle;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &adc_handle));

    vTaskDelay(pdMS_TO_TICKS(10));

    s_adc_devices.emplace(chip_select_pin, adc_handle);
}

spi_device_handle_t get_adc_device(gpio_num_t adc_gpio_address) {
    auto it = s_adc_devices.find(adc_gpio_address);
    if (it != s_adc_devices.end()) {
        return it->second;
    }

    if (!s_adc_bus_initialized) {
        init_adc(s_adc_clock_hz, adc_gpio_address);
    }

    
    return s_adc_devices.find(adc_gpio_address)->second;
}

static uint8_t spi_xfer(gpio_num_t chip_select, uint8_t data)
{
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &data,
        .rx_buffer = NULL
    };

    uint8_t rx = 0;
    t.rx_buffer = &rx;

    spi_device_handle_t adc_handle = get_adc_device(chip_select);

    ESP_ERROR_CHECK(spi_device_transmit(adc_handle, &t));
    return rx;
}

uint16_t adc1283_read(gpio_num_t chip_select, int channel)
{
    if (channel > 7) channel = 7;

    gpio_set_level(chip_select, 0);
    esp_rom_delay_us(1);

    uint8_t control = (channel << 3);
    spi_xfer(chip_select, control);

    // Discard old result
    spi_xfer(chip_select, 0x00);

    uint8_t msb = spi_xfer(chip_select, 0x00);
    uint8_t lsb = spi_xfer(chip_select, 0x00);

    gpio_set_level(chip_select, 1);

    // uint16_t raw = ((uint16_t)msb << 8) | lsb;
    // return raw & 0xFFFF;
    uint16_t raw = ((uint16_t)msb << 8) | lsb; // 16-bit combined
    // uint8_t top8 = (raw >> 4) & 0xFF;          // shift right 4 bits to keep bits 11..4
    // return top8;
    return raw;
}

void serial_init(int baud_rate) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    uart_param_config(UART_NUM_0, &uart_config);
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_0, 1024, 0, 0, nullptr, 0);
}

void init_pwm_driver() {
    if (s_i2c_initialized) {
        return;
    }

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = I2C_SDA_PIN;
    bus_cfg.scl_io_num = I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 0;
    bus_cfg.trans_queue_depth = 22;
    bus_cfg.flags.enable_internal_pullup = true;

    i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    s_i2c_initialized = true;

    // Set GPIO 13 to LOW
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 0);

    // Initialize both PCA9685 devices (0x40 and 0x41)
    const uint8_t addresses[] = {0x40, 0x41};
    for (uint8_t addr : addresses) {
        i2c_device_config_t dev_cfg = {};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address = addr;
        dev_cfg.scl_speed_hz = I2C_CLOCK_HZ;

        i2c_master_dev_handle_t handle = nullptr;
        esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &handle);
        
        if (err == ESP_OK && handle != nullptr) {

            // Calculation: (25MHz / (4096 * 50Hz)) - 1 = 121 (0x79)
            uint8_t prescale_val = 0x02;
            uint8_t sleep_mode   = 0x31; // bit5=1 (AI), bit4=1 (Sleep), bit0=1 (ALLCALL)
            uint8_t wake_mode    = 0x21; // bit5=1 (AI), bit4=0 (Wake),  bit0=1 (ALLCALL)
            uint8_t restart_mode = 0xA1; // bit7=1 (Restart), bit5=1 (AI), bit0=1 (ALLCALL)
            uint8_t PCA9685_MODE1 = 0x00;
            uint8_t PCA9685_PRESCALE = 0xFE;

            // 1. Put to Sleep (Required to change frequency)
            uint8_t cmd_sleep[] = {PCA9685_MODE1, sleep_mode};
            i2c_master_transmit(handle, cmd_sleep, 2, 10);

            // 2. Set the Frequency
            uint8_t cmd_freq[] = {PCA9685_PRESCALE, prescale_val};
            i2c_master_transmit(handle, cmd_freq, 2, 10);

            // 3. Wake up
            uint8_t cmd_wake[] = {PCA9685_MODE1, wake_mode};
            i2c_master_transmit(handle, cmd_wake, 2, 10);

            // 4. Critical: Wait for the internal oscillator to stabilize
            // Using a 1ms delay to be safe (datasheet asks for 500us)
            vTaskDelay(pdMS_TO_TICKS(5)); 

            // 5. Finalize Restart
            uint8_t cmd_restart[] = {PCA9685_MODE1, restart_mode};
            i2c_master_transmit(handle, cmd_restart, 2, 10);

            
            
            // Store the device handle using the I2C address as the key
            s_pwm_devices.emplace(addr, handle);
            
            printf("PCA9685 Initialized at I2C address 0x%02X", addr);
        } else {
            printf("PCA9685 Failed to initialize at 0x%02X", addr);
            
        }
    }
}

static i2c_master_dev_handle_t get_pwm_device(int driver_i2c_address) {
    if (!s_i2c_initialized) {
        init_pwm_driver();
    }

    auto it = s_pwm_devices.find(driver_i2c_address);
    if (it != s_pwm_devices.end()) {
        return it->second;
    }

    // Device not found - this shouldn't happen if init_pwm_driver() worked correctly
    serial_print("ERROR: PCA9685 device at 0x");
    serial_printf("%02X", driver_i2c_address);
    serial_print(" not found. Was it initialized?\n");
    return nullptr;
}

void pca9685_set_pwm(int driver_i2c_address, int channel, int value_0_4095) {
    if (!s_i2c_initialized) {
        init_pwm_driver();
    }

    if (channel < 0 || channel > 15) {
        return;
    }

    if (value_0_4095 < 0) {
        value_0_4095 = 0;
    } else if (value_0_4095 > 4095) {
        value_0_4095 = 4095;
    }

    const i2c_master_dev_handle_t dev = get_pwm_device(driver_i2c_address);
    if (dev == nullptr) {
        serial_print("ERROR: Cannot set PWM - device handle is null\n");
        return;
    }
    
    const int on_count = 0;
    const int off_count = value_0_4095;

    const uint8_t reg = static_cast<uint8_t>(0x06 + 4 * channel); // LED0_ON_L

    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on_count & 0xFF),
        static_cast<uint8_t>((on_count >> 8) & 0x0F),
        static_cast<uint8_t>(off_count & 0xFF),
        static_cast<uint8_t>((off_count >> 8) & 0x0F)
    };

    i2c_master_transmit(dev, data, sizeof(data), 10);
}


// Helper function to send SHTP packets to BNO08x
static esp_err_t bno08x_send_packet(uint8_t channel, const uint8_t* data, size_t len) {
    if (s_imu_device == nullptr) return ESP_FAIL;
    if (channel > CHANNEL_GYRO) return ESP_ERR_INVALID_ARG;
    if (len + 4 > 256) return ESP_ERR_INVALID_SIZE;

    // SHTP header: 4 bytes length + channel number
    uint16_t packet_len = len + 4;
    uint8_t buffer[256];
    
    buffer[0] = packet_len & 0xFF;
    buffer[1] = (packet_len >> 8) & 0xFF;
    buffer[2] = channel;
    buffer[3] = s_shtp_tx_seq[channel]++;
    
    if (len > 0 && data != nullptr) {
        memcpy(&buffer[4], data, len);
    }
    
    printf("[SEND] ch=%u seq=%u len=%u\n", channel, buffer[3], (unsigned)packet_len);
    esp_err_t err = i2c_master_transmit(s_imu_device, buffer, packet_len, 100);
    printf("[SEND] result=%d\n", err);
    return err;
}

// Helper function to read SHTP packets from BNO08x
static esp_err_t bno08x_receive_packet(uint8_t* buffer, size_t max_len, size_t* received_len) {
    if (s_imu_device == nullptr || buffer == nullptr || received_len == nullptr || max_len < 4) {
        return ESP_ERR_INVALID_ARG;
    }

    *received_len = 0;

    // Read 4-byte SHTP header
    uint8_t header[4] = {0};
    esp_err_t err = i2c_master_receive(s_imu_device, header, sizeof(header), 20);
    if (err != ESP_OK) {
        return err;
    }

    // SHTP length includes header; bit15 is continuation flag
    uint16_t packet_len = static_cast<uint16_t>(header[0] | (header[1] << 8)) & 0x7FFF;
    uint8_t channel = header[2];

    if (packet_len == 0) {
        return ESP_ERR_NOT_FOUND;
    }

    if (packet_len < 4 || packet_len > max_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (channel > CHANNEL_GYRO) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memcpy(buffer, header, sizeof(header));

    const size_t payload_len = packet_len - 4;
    if (payload_len > 0) {
        err = i2c_master_receive(s_imu_device, &buffer[4], payload_len, 20);
        if (err != ESP_OK) {
            return err;
        }
    }

    *received_len = packet_len;
    return ESP_OK;
}

// Enable a specific sensor report
static esp_err_t bno08x_enable_report(uint8_t report_id, uint32_t interval_us) {
    uint8_t cmd[17] = {0};
    
    cmd[0] = 0xFD;  // Set feature command
    cmd[1] = report_id;
    cmd[2] = 0;  // Feature flags
    cmd[3] = 0;
    cmd[4] = 0;
    
    // Report interval in microseconds (little endian)
    cmd[5] = interval_us & 0xFF;
    cmd[6] = (interval_us >> 8) & 0xFF;
    cmd[7] = (interval_us >> 16) & 0xFF;
    cmd[8] = (interval_us >> 24) & 0xFF;
    
    // Batch interval (same as report interval)
    cmd[9] = cmd[5];
    cmd[10] = cmd[6];
    cmd[11] = cmd[7];
    cmd[12] = cmd[8];
    
    esp_err_t err = bno08x_send_packet(CHANNEL_CONTROL, cmd, 17);
    if (err != ESP_OK) {
        ESP_LOGE("IMU", "Failed enabling report 0x%02X, err=%s", report_id, esp_err_to_name(err));
    } else {
        ESP_LOGI("IMU", "Enabled report 0x%02X @ %u us", report_id, static_cast<unsigned>(interval_us));
    }
    return err;
}

// Parse sensor reports from BNO08x
static void bno08x_parse_sensor_report(const uint8_t* data, size_t len) {
    
    if (data == nullptr || len < 4) return;

    const uint8_t rid0 = data[0];
    const uint8_t rid5 = (len > 5) ? data[5] : 0xFF;
    uint8_t report_id = rid0;

    const bool rid0_known =
        rid0 == SENSOR_REPORTID_ROTATION_VECTOR ||
        rid0 == SENSOR_REPORTID_GYROSCOPE ||
        rid0 == SENSOR_REPORTID_ACCELEROMETER ||
        rid0 == SENSOR_REPORTID_MAGNETIC_FIELD;

    if (!rid0_known && rid5 != 0xFF) {
        report_id = rid5;
    }
    
    // Helper to extract int16_t from little-endian bytes
    auto get_int16 = [](const uint8_t* buf, size_t offset) -> int16_t {
        return (int16_t)(buf[offset] | (buf[offset + 1] << 8));
    };
    
    // Helper to convert Q-point fixed to float
    auto quat_to_float = [](int16_t val) -> float {
        return val / 16384.0f;  // Q14 fixed point
    };
    
    auto gyro_to_float = [](int16_t val) -> float {
        return val / 32768.0f * 2000.0f * (3.14159265f / 180.0f);  // Convert to rad/s
    };
    
    auto accel_to_float = [](int16_t val) -> float {
        return val / 32768.0f * 16.0f * 9.81f;  // Convert to m/s^2
    };
    
    auto mag_to_float = [](int16_t val) -> float {
        return val / 32768.0f * 1000.0f;  // Convert to µT
    };

    
    
    switch (report_id) {
        case SENSOR_REPORTID_ROTATION_VECTOR:
            if (len >= 14) {
                s_imu_data.quat_x = quat_to_float(get_int16(data, 4));
                s_imu_data.quat_y = quat_to_float(get_int16(data, 6));
                s_imu_data.quat_z = quat_to_float(get_int16(data, 8));
                s_imu_data.quat_w = quat_to_float(get_int16(data, 10));
                s_imu_data.quat_valid = true;
                s_imu_data.last_update_ms = esp_timer_get_time() / 1000;
            }
            break;
            
        case SENSOR_REPORTID_GYROSCOPE:
            if (len >= 10) {
                s_imu_data.gyro_x = gyro_to_float(get_int16(data, 4));
                s_imu_data.gyro_y = gyro_to_float(get_int16(data, 6));
                s_imu_data.gyro_z = gyro_to_float(get_int16(data, 8));
                s_imu_data.gyro_valid = true;
            }
            break;
            
        case SENSOR_REPORTID_ACCELEROMETER:
            if (len >= 10) {
                s_imu_data.accel_x = accel_to_float(get_int16(data, 4));
                s_imu_data.accel_y = accel_to_float(get_int16(data, 6));
                s_imu_data.accel_z = accel_to_float(get_int16(data, 8));
                s_imu_data.accel_valid = true;
            }
            break;
            
        case SENSOR_REPORTID_MAGNETIC_FIELD:
            if (len >= 10) {
                s_imu_data.mag_x = mag_to_float(get_int16(data, 4));
                s_imu_data.mag_y = mag_to_float(get_int16(data, 6));
                s_imu_data.mag_z = mag_to_float(get_int16(data, 8));
                s_imu_data.mag_valid = true;
            }
            break;
        default:
            printf("[PARSE] Unknown report id 0x%02X (rid0=0x%02X rid5=0x%02X len=%u)\n",
                   report_id, rid0, rid5, static_cast<unsigned>(len));
            break;
    }
}

// Poll for new data from BNO08x
static void bno08x_poll_data() {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        printf("[Poll] IMU not initialized\n");
        return;
    }

    uint8_t buffer[256];
    size_t len = 0;
    
    esp_err_t err = bno08x_receive_packet(buffer, sizeof(buffer), &len);

    printf("[RECV] err=%d len=%u\n", err, (unsigned)len);

    if (err != ESP_OK || len <= 4) {
        printf("[RECV] Early return (err=%d, len=%u)\n", err, (unsigned)len);
        return;  // No data or error
    }

    const uint8_t channel = buffer[2];
    const uint8_t seq = buffer[3];
    const uint8_t* payload = &buffer[4];
    const size_t payload_len = len - 4;

    printf("[PKT] ch=%u seq=%u len=%u p0=0x%02X p1=0x%02X\n",
             channel,
             seq,
             static_cast<unsigned>(len),
             payload_len > 0 ? payload[0] : 0,
             payload_len > 1 ? payload[1] : 0);

    // Parse only sensor report channels
    if (channel == CHANNEL_REPORTS || channel == CHANNEL_WAKE_REPORTS) {
        printf("  -> Parsing payload...\n");
        bno08x_parse_sensor_report(payload, payload_len);
    } else {
        printf("  -> Ignoring non-report channel\n");
    }
}

void update_imu_data() {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        return;
    }

    printf("[UPDATE_IMU] Polling...\n");
    // Drain a few queued packets each cycle so 100Hz reports don't back up.
    for (int i = 0; i < 4; ++i) {
        bno08x_poll_data();
    }
    printf("[UPDATE_IMU] Done\n");
}

void init_imu() {
    printf("[INIT_IMU] Starting...\n");
    if (s_imu_initialized) {
        printf("[INIT_IMU] Already initialized\n");
        return;
    }

    // Ensure I2C bus is initialized
    if (!s_i2c_initialized) {
        printf("[INIT_IMU] I2C not initialized, calling init_pwm_driver...\n");
        init_pwm_driver();  // This initializes the I2C bus
    }
    printf("[INIT_IMU] I2C bus ready\n");

    // Try primary address first
    i2c_device_config_t imu_cfg = {};
    imu_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    imu_cfg.device_address = BNO08X_I2C_ADDR;
    imu_cfg.scl_speed_hz = I2C_CLOCK_HZ;

    esp_err_t err = i2c_master_bus_add_device(s_i2c_bus, &imu_cfg, &s_imu_device);
    
    if (err != ESP_OK) {
        // Try alternate address
        serial_print("BNO08x not found at 0x4A, trying 0x4B...\n");
        imu_cfg.device_address = BNO08X_I2C_ADDR_ALT;
        err = i2c_master_bus_add_device(s_i2c_bus, &imu_cfg, &s_imu_device);
        
        if (err != ESP_OK) {
            serial_print("ERROR: BNO08x IMU not detected on I2C bus\n");
            s_imu_device = nullptr;
            return;
        }
    }

    ESP_LOGI("IMU", "BNO08x attached at I2C address 0x%02X", imu_cfg.device_address);


    // Wait for IMU to be ready
    vTaskDelay(pdMS_TO_TICKS(100));

    // Soft reset the device
    uint8_t reset_cmd[] = {0x01};
    err = bno08x_send_packet(CHANNEL_EXECUTABLE, reset_cmd, 1);
    if (err != ESP_OK) {
        ESP_LOGE("IMU", "Failed to send soft reset, err=%s", esp_err_to_name(err));
    }
    vTaskDelay(pdMS_TO_TICKS(300));

    // Clear any pending data
    for (int i = 0; i < 5; i++) {
        uint8_t dummy[256];
        size_t len;
        bno08x_receive_packet(dummy, sizeof(dummy), &len);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    printf("BNO08x IMU detected and reset successfully\n");

    // Enable sensor reports at 100Hz (10000 microseconds)
    uint32_t report_interval_us = 10000;
    
    err = bno08x_enable_report(SENSOR_REPORTID_ROTATION_VECTOR, report_interval_us);
    if (err != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(50));
    
    err = bno08x_enable_report(SENSOR_REPORTID_GYROSCOPE, report_interval_us);
    if (err != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(50));

    err = bno08x_enable_report(SENSOR_REPORTID_ACCELEROMETER, report_interval_us);
    if (err != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(50));

    err = bno08x_enable_report(SENSOR_REPORTID_MAGNETIC_FIELD, report_interval_us);
    if (err != ESP_OK) return;
    vTaskDelay(pdMS_TO_TICKS(50));

    s_imu_initialized = true;
    printf("[INIT_IMU] Initialization complete!\n");
}

void init_comms() {

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize the underlying TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());
    
    // Create default event loop (handles wifi events in the background)
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // Create the default Wi-Fi Access Point interface
    esp_netif_create_default_wifi_ap();

    // Initialize Wi-Fi with default configuration
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Configure the Soft-AP settings
    wifi_config_t wifi_config = {
        .ap = {
            
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .max_connection = 4,
          
        },    
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // Start the Wi-Fi hardware
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Soft-AP setup finished. SSID:%s password:%s",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
}




// IMU data reading functions
bool imu_data_available() {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        return false;
    }
    
    update_imu_data();
    
    return s_imu_data.quat_valid || s_imu_data.gyro_valid || 
           s_imu_data.accel_valid || s_imu_data.mag_valid;
}

bool read_imu_quaternion(float& w, float& x, float& y, float& z) {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        return false;
    }
    
    if (s_imu_data.quat_valid) {
        w = s_imu_data.quat_w;
        x = s_imu_data.quat_x;
        y = s_imu_data.quat_y;
        z = s_imu_data.quat_z;
        return true;
    }
    
    return false;
}

bool read_imu_angular_velocity(float& x, float& y, float& z) {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        return false;
    }
    
    if (s_imu_data.gyro_valid) {
        x = s_imu_data.gyro_x;
        y = s_imu_data.gyro_y;
        z = s_imu_data.gyro_z;
        return true;
    }
    
    return false;
}

bool read_imu_accelerometer(float& x, float& y, float& z) {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        return false;
    }
    
    if (s_imu_data.accel_valid) {
        x = s_imu_data.accel_x;
        y = s_imu_data.accel_y;
        z = s_imu_data.accel_z;
        return true;
    }
    
    return false;
}

bool read_imu_magnetometer(float& x, float& y, float& z) {
    if (!s_imu_initialized || s_imu_device == nullptr) {
        return false;
    }
    
    if (s_imu_data.mag_valid) {
        x = s_imu_data.mag_x;
        y = s_imu_data.mag_y;
        z = s_imu_data.mag_z;
        return true;
    }
    
    return false;
}

void init_peripherals(int adc_clock_speed_hz, int uart_baud_rate) {
    for (gpio_num_t pin : ADC_CHANNEL_SELECT) {
        init_adc(adc_clock_speed_hz, pin);
    }

    init_pwm_driver();
    init_imu();
    init_comms();
    serial_init(uart_baud_rate);
    
}