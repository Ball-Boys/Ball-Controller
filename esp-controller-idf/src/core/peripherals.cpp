#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <unordered_map>
#include <driver/gpio.h>
#include <math.h>

#include <string.h>

#include "peripherals.h"
#include <utils/utils.h>
#include <core/global_state.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
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

// SHTP Advertise message (from BNO08x datasheet)
static constexpr uint8_t SHTP_CHANNEL_CTRL_ADDR = 0;
static constexpr uint8_t SHTP_ADVERTISE_LEN = 4;
static const uint8_t s_shtp_advertise[] = {0x14, 0x00, 0x00, 0x00};  // Protocol version

// SHTP service state
enum AdvertPhase {
    ADVERT_NEEDED = 0,
    ADVERT_REQUESTED = 1,
    ADVERT_RECEIVED = 2
};

static AdvertPhase s_advert_phase = ADVERT_NEEDED;
static uint8_t s_shtp_rx_buffer[256];  // Receive buffer for SHTP packets

// Sensor report IDs
static constexpr uint8_t SENSOR_REPORTID_ROTATION_VECTOR = 0x05;
static constexpr uint8_t SENSOR_REPORTID_GYROSCOPE = 0x02;
static constexpr uint8_t SENSOR_REPORTID_ACCELEROMETER = 0x01;
static constexpr uint8_t SENSOR_REPORTID_MAGNETIC_FIELD = 0x03;

static void log_stack_watermark(const char* tag) {
    const UBaseType_t watermark_words = uxTaskGetStackHighWaterMark(nullptr);
    const size_t watermark_bytes = static_cast<size_t>(watermark_words) * sizeof(StackType_t);
    printf("[STACK] %s high_water=%u words (~%u bytes free)\n",
           tag,
           static_cast<unsigned>(watermark_words),
           static_cast<unsigned>(watermark_bytes));
}
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

    // Set GPIO 13 to LOW
    gpio_reset_pin(GPIO_NUM_13);
    gpio_set_direction(GPIO_NUM_13, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_13, 1);

    i2c_master_bus_config_t bus_cfg = {};
    bus_cfg.i2c_port = I2C_NUM_0;
    bus_cfg.sda_io_num = I2C_SDA_PIN;
    bus_cfg.scl_io_num = I2C_SCL_PIN;
    bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_cfg.glitch_ignore_cnt = 7;
    bus_cfg.intr_priority = 0;
    bus_cfg.trans_queue_depth = 0;
    bus_cfg.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_i2c_bus));
    s_i2c_initialized = true;

    gpio_set_level(GPIO_NUM_13, 0); // Set GPIO 13 LOW after initializing I2C bus (if needed by the hardware)

    

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
            uint8_t PCA9685_MODE1 = 0x00;
            uint8_t PCA9685_PRESCALE = 0xFE;
            uint8_t restart_mode = 0b00000000;
 

            // 1. Wake up the device (Set MODE1)
            uint8_t cmd_wake[] = {0x00, 0x00}; 
            i2c_master_transmit(handle, cmd_wake, 2, 10);

            uint8_t cmd_wake2[] = {0x01, 0x14};
            i2c_master_transmit(handle, cmd_wake2, 2, 10);


            // 2. Set LEDOUT registers to enable PWM (0xAA sets all 4 LEDs in a reg to PWM mode)
            // We need to do this for LEDOUT0, 1, 2, and 3 (Registers 0x14 - 0x17)
            for (uint8_t reg = 0x14; reg <= 0x17; reg++) {
                uint8_t cmd_ledout[] = {reg, 0xAA}; // 0xAA = 10101010 in binary
                i2c_master_transmit(handle, cmd_ledout, 2, 10);
            }

            // 5. Finalize Restart
            uint8_t cmd_restart[] = {PCA9685_MODE1, restart_mode};
            i2c_master_transmit(handle, cmd_restart, 2, 10);

            for (int channel = 0; channel < 16; channel++) {
                pca9685_set_pwm(addr, channel, 0); // Start with all channels at 0
                vTaskDelay(pdMS_TO_TICKS(1)); // Short delay between channel setups
            }

            
            
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

void pca9685_set_pwm(int driver_i2c_address, int channel, int value_0_255) {
    printf("Setting PWM on I2C addr 0x%02X, channel %d to value %d\n", driver_i2c_address, channel, value_0_255);
    if (!s_i2c_initialized) {
        init_pwm_driver();
    }

    if (channel < 0 || channel > 15) {
        return;
    }

    if (value_0_255 < 0) {
        value_0_255 = 0;
    } else if (value_0_255 > 255) {
        value_0_255 = 255;
    }

    const i2c_master_dev_handle_t dev = get_pwm_device(driver_i2c_address);
    if (dev == nullptr) {
        serial_print("ERROR: Cannot set PWM - device handle is null\n");
        return;
    }
    


    const uint8_t reg = static_cast<uint8_t>(0x02 + channel); 

    uint8_t data[2] = {
        reg,
        static_cast<uint8_t>(value_0_255) // Scale 0-255 to 0-4095 by multiplying by 16,
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
    ESP_ERROR_CHECK(i2c_master_transmit(s_imu_device, buffer, packet_len, 100));
    printf("[SEND] result=%d\n", ESP_OK);
    return ESP_OK;
}




// Persistant state for assembling fragmented packets
static uint8_t s_assembly_buffer[MAX_ASSEMBLY_LEN];
static uint16_t s_assembly_cursor = 0;

void parse_rotation_vector(const uint8_t* data) {
    // RV Q-point = 14
    auto q_to_f = [](int16_t raw, int q) { return static_cast<float>(raw) * std::pow(2.0f, -q); };

    int16_t i_raw = static_cast<int16_t>(data[4] | (data[5] << 8));
    int16_t j_raw = static_cast<int16_t>(data[6] | (data[7] << 8));
    int16_t k_raw = static_cast<int16_t>(data[8] | (data[9] << 8));
    int16_t r_raw = static_cast<int16_t>(data[10] | (data[11] << 8));

    float i = q_to_f(i_raw, 14);
    float j = q_to_f(j_raw, 14);
    float k = q_to_f(k_raw, 14);
    float r = q_to_f(r_raw, 14);

    GlobalState& instance = GlobalState::instance();

    instance.setOrientation(Orientation(r, i, j, k));

}

void parse_accelerometer(const uint8_t* data) {
    // do nothing
}

void parse_gyroscope(const uint8_t* data) {
    // Gyro Q-point = 8
    auto q_to_f = [](int16_t raw, int q) { return static_cast<float>(raw) * std::pow(2.0f, -q); };

    int16_t x_raw = static_cast<int16_t>(data[4] | (data[5] << 8));
    int16_t y_raw = static_cast<int16_t>(data[6] | (data[7] << 8));
    int16_t z_raw = static_cast<int16_t>(data[8] | (data[9] << 8));

    float x = q_to_f(x_raw, 8);
    float y = q_to_f(y_raw, 8);
    float z = q_to_f(z_raw, 8);

    GlobalState& instance = GlobalState::instance();

    instance.setAngularVelocity(AngularVelocity(x, y, z));

}

static void rxAssemble(const uint8_t* packet, uint16_t len) {
    if (len < SHTP_HEADER_SIZE) return;

    // Byte 0-1: Length (LSB first)
    // IMPORTANT: Just take the 15 bits. Ignore the 16th bit for now.
    uint16_t packet_len = (packet[0] | (packet[1] << 8)) & 0x7FFF;
    uint8_t channel = packet[2];
    uint8_t seq = packet[3];

    

    // DEBUG: If you see len=276, you are reading the Advertisement.
    // In most cases, one I2C read = One complete SHTP packet.
    
    // Skip the 4-byte header to get to the data
    const uint8_t* payload = &packet[SHTP_HEADER_SIZE];
    uint16_t payload_len = packet_len - SHTP_HEADER_SIZE;

    if (channel == 3) {
        uint16_t i = 0;
        process_channel_3(&payload[i], payload_len - i);
        
    } else if (channel == 0) {
        // This is the advertisement (276 bytes). 
        // You can ignore this for now unless you want to parse Q-points dynamically.
        ESP_LOGI(TAG, "Ch 0: Advertisement received (len %d)", packet_len);
    }
}

void imu_purge_buffer() {
    // nothing as of right now
}

void process_channel_3(const uint8_t* payload, uint16_t payload_len) {
    uint16_t i = 0;

    while (i < payload_len) {
        uint8_t report_id = payload[i];
        

        switch (report_id) {
            case 0xFB: // Base Timestamp Reference
                // Total length: 5 bytes (ID + 4 bytes of 32-bit timestamp)
                i += 5; 
                break;

            case 0xF2: // Base Timestamp (used in some firmware versions)
                i += 5;
                break;

            case 0x05: // Rotation Vector
                parse_rotation_vector(&payload[i]);
                i += 14; 
                break;

            case 0x01: // Accelerometer
                parse_accelerometer(&payload[i]);
                i += 10;
            break;

            case 0x02: // Gyroscope
                parse_gyroscope(&payload[i]); // Gyro has same data format as Accel, just different scaling
                i += 10;
            break;

            case 0x03: // Magnetometer
                i += 10;
            break;

            default:
                // CRITICAL: If we hit an unknown ID, we don't know the length.
                // We must abort parsing this packet to avoid reading garbage.
                ESP_LOGW("IMU", "Unknown Report ID: 0x%02x at index %d", report_id, i);
                return; 
        }
    }
}

void shtp_service() {
    uint8_t header[SHTP_HEADER_SIZE];
    static uint8_t packet_scratchpad[MAX_PACKET_LEN];

    // 1. Read the 4-byte header to see how much data is waiting
    esp_err_t err = i2c_master_receive(s_imu_device, header, SHTP_HEADER_SIZE, 50);
    if (err != ESP_OK) return;

    // Mask the length (ignore Bit 15 here)
    uint16_t packet_len = (header[0] | (header[1] << 8)) & 0x7FFF;

    if (packet_len < SHTP_HEADER_SIZE || packet_len > MAX_PACKET_LEN) {
        return;
    }

    // 2. Perform a full read of the entire packet (header + payload)
    // The FSM30X requires the full length to be read to clear its internal buffer.
    err = i2c_master_receive(s_imu_device, packet_scratchpad, packet_len, 100);
    if (err == ESP_OK) {
        rxAssemble(packet_scratchpad, packet_len);
    }
}


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


// Sequence number tracking for TX (The sensor expects us to increment this)
static uint8_t tx_seq_counts[6] = {0, 0, 0, 0, 0, 0};

/**
 * Sends a raw SHTP packet. 
 * Correctly includes the 4-byte header in the length.
 */
esp_err_t imu_send_packet(i2c_master_dev_handle_t dev, uint8_t channel, uint8_t* payload, uint16_t len) {
    uint16_t packet_len = len + 4;
    uint8_t tx_buffer[MAX_PACKET_LEN];

    tx_buffer[0] = packet_len & 0xFF;
    tx_buffer[1] = (packet_len >> 8) & 0xFF;
    tx_buffer[2] = channel;
    tx_buffer[3] = tx_seq_counts[channel]++;

    memcpy(&tx_buffer[4], payload, len);

    return i2c_master_transmit(dev, tx_buffer, packet_len, 100);
}

/**
 * The "Right" Setup Flow
 */
void init_imu() {
    ESP_LOGI(TAG, "Starting IMU Hardware Reset...");

    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_25, 1);


    // Ensure I2C bus is initialized f
    if (!s_i2c_initialized) {
        printf("[INIT_IMU] I2C not initialized, calling init_pwm_driver...\n");
        init_pwm_driver(); // This initializes the I2C bus
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
            log_stack_watermark("init_imu:not_detected");
            return;
        }
    }


    // 1. Hardware/Soft Reset
    uint8_t reset_cmd = 0x01; // Reset command for Executable channel
    imu_send_packet(s_imu_device, 1, &reset_cmd, 1);
    
    // 2. Wait for the chip to reboot
    vTaskDelay(pdMS_TO_TICKS(200));

    // 3. DRAIN THE ADVERTISEMENT
    // The sensor pushes a ~276 byte 'CV' on Channel 0 after reset.
    // If you don't read this, you can't talk to it on other channels.
    ESP_LOGI(TAG, "Draining SHTP Advertisement...");
    for(int i = 0; i < 15; i++) {
        shtp_service(); // Using your service to empty the buffer
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // 4. ENABLE ROTATION VECTOR
    // We use Batch Interval = 0 for immediate real-time updates.
    ESP_LOGI(TAG, "Configuring Rotation Vector @ 100Hz...");
    uint32_t interval_us = 10000; // 100Hz
    uint8_t feat_cmd[17] = {0};
    
    feat_cmd[0] = 0xFD; // Set Feature Command
    feat_cmd[1] = 0x05; // Rotation Vector ID
    // 5-8: Report Interval
    feat_cmd[5] = (interval_us & 0xFF);
    feat_cmd[6] = ((interval_us >> 8) & 0xFF);
    feat_cmd[7] = ((interval_us >> 16) & 0xFF);
    feat_cmd[8] = ((interval_us >> 24) & 0xFF);
    // 9-12: Batch Interval (MUST BE 0 for real-time)
    feat_cmd[9] = 0; 
    feat_cmd[10] = 0;
    feat_cmd[11] = 0;
    feat_cmd[12] = 0;

    err = imu_send_packet(s_imu_device, 2, feat_cmd, 17);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Feature Command");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay to ensure the sensor processes the command

    // ENABLE GYRO
    uint8_t feat_cmd_gyro[17] = {0};
    feat_cmd_gyro[0] = 0xFD; // Set Feature Command
    feat_cmd_gyro[1] = 0x02; // Gyroscope ID
    // 5-8: Report Interval
    feat_cmd_gyro[5] = (interval_us & 0xFF);
    feat_cmd_gyro[6] = ((interval_us >> 8) & 0xFF);
    feat_cmd_gyro[7] = ((interval_us >> 16) & 0xFF);
    feat_cmd_gyro[8] = ((interval_us >> 24) & 0xFF);
    // 9-12: Batch Interval (MUST BE 0 for real-time)
    feat_cmd_gyro[9] = 0; 
    feat_cmd_gyro[10] = 0;
    feat_cmd_gyro[11] = 0;
    feat_cmd_gyro[12] = 0;

    err = imu_send_packet(s_imu_device, 2, feat_cmd_gyro, 17);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send Gyro Feature Command");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1)); // Short delay to ensure the sensor processes the command

    // 5. FINAL WAIT
    // Give the fusion engine a moment to stabilize
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI(TAG, "IMU Setup Complete. Fusion Engine Active.");
    return;
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






void init_peripherals(int adc_clock_speed_hz, int uart_baud_rate) {
    init_pwm_driver();
    for (gpio_num_t pin : ADC_CHANNEL_SELECT) {
        init_adc(adc_clock_speed_hz, pin);
    }

    
    
    init_comms();

    serial_init(uart_baud_rate);
    init_imu();
    
}