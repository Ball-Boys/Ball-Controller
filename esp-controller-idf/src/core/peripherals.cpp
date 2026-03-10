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
#include "lwip/sockets.h"




namespace {
static bool s_adc_bus_initialized = false;
static int s_adc_clock_hz = 10000000; // Default to 10 MHz
static std::unordered_map<int, spi_device_handle_t> s_adc_devices;

static bool s_i2c_initialized = false;
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static std::unordered_map<int, i2c_master_dev_handle_t> s_pwm_devices;
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
            uint8_t PCA9685_MODE1 = 0x00;
            uint8_t PCA9685_PRESCALE = 0xFE;
            uint8_t restart_mode = 0b00000000;

            // 5. Finalize Restart
            uint8_t cmd_restart[] = {PCA9685_MODE1, restart_mode};
            i2c_master_transmit(handle, cmd_restart, 2, 10);

            // 1. Wake up the device (Set MODE1)
            uint8_t cmd_wake[] = {0x00, 0x00}; 
            i2c_master_transmit(handle, cmd_wake, 2, 10);

            // 2. Set LEDOUT registers to enable PWM (0xAA sets all 4 LEDs in a reg to PWM mode)
            // We need to do this for LEDOUT0, 1, 2, and 3 (Registers 0x14 - 0x17)
            for (uint8_t reg = 0x14; reg <= 0x17; reg++) {
                uint8_t cmd_ledout[] = {reg, 0xAA}; // 0xAA = 10101010 in binary
                i2c_master_transmit(handle, cmd_ledout, 2, 10);
            }

            
            
            // Store the device handle using the I2C address as the key
            s_pwm_devices.emplace(addr, handle);
            
            serial_print("Initialized PCA9685 at I2C address 0x");
            serial_printf("%02X", addr);
            serial_print("\n");
        } else {
            serial_print("Failed to initialize PCA9685 at 0x");
            serial_printf("%02X", addr);
            serial_print("\n");
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


void init_imu() {

}

void init_comms() {
    // --- REFACTORED WIFI SETUP FUNCTION ---

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
    for (gpio_num_t pin : ADC_CHANNEL_SELECT) {
        init_adc(adc_clock_speed_hz, pin);
    }

    init_pwm_driver();
    init_imu();
    init_comms();
    serial_init(uart_baud_rate);
    
}