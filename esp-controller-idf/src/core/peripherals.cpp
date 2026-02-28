#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <unordered_map>
#include <driver/gpio.h>

#include "peripherals.h"
#include <utils/utils.h>
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
            uint8_t prescale_val = 0x20;
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


void init_imu() {

}

void init_comms() {

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