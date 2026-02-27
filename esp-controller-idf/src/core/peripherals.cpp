#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/uart.h>
#include <unordered_map>
#include <driver/gpio.h>

#include "peripherals.h"
#include <utils/utils.h>

namespace {
static bool s_adc_bus_initialized = false;
static int s_adc_clock_hz = 0;
static std::unordered_map<int, spi_device_handle_t> s_adc_devices;

static bool s_i2c_initialized = false;
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static std::unordered_map<int, i2c_master_dev_handle_t> s_pwm_devices;
}

void init_adc(int clock_speed_hz) {
    if (s_adc_bus_initialized) {
        s_adc_clock_hz = clock_speed_hz;
        return;
    }

    spi_bus_config_t buscfg = {
        .mosi_io_num = ADC_SENSE_PIN,
        .miso_io_num = ADC_CHANNEL_SELECT,
        .sclk_io_num = ADC_SLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    s_adc_clock_hz = clock_speed_hz;
    spi_bus_initialize(HSPI_HOST, &buscfg, SPI_DMA_CH_AUTO);
    s_adc_bus_initialized = true;
}

spi_device_handle_t get_adc_device(int adc_gpio_address) {
    auto it = s_adc_devices.find(adc_gpio_address);
    if (it != s_adc_devices.end()) {
        return it->second;
    }

    if (!s_adc_bus_initialized) {
        init_adc(1000000);
    }

    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .duty_cycle_pos = 0,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz = s_adc_clock_hz > 0 ? s_adc_clock_hz : 1000000,
        .input_delay_ns = 0,
        .spics_io_num = adc_gpio_address,
        .flags = 0,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    spi_device_handle_t handle = nullptr;
    spi_bus_add_device(HSPI_HOST, &devcfg, &handle);
    s_adc_devices.emplace(adc_gpio_address, handle);
    return handle;
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
    bus_cfg.trans_queue_depth = 8;
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
            // Wake up the PCA9685 chip (it starts in sleep mode by default)
            // MODE1 register (0x00): RESTART (0x80) | AUTO_INCREMENT (0x20) = 0xA0
            uint8_t wakeup_data[] = {0x00, 0xA0};
            i2c_master_transmit(handle, wakeup_data, sizeof(wakeup_data), 10);
            
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
    serial_print("Setting PWM: driver_i2c_address=0x");
    serial_printf("%02X", driver_i2c_address);
    serial_print(", channel=");
    serial_printf("%d", channel);
    serial_print(", value_0_4095=");
    serial_printf("%d", value_0_4095);
    serial_print("\n");

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
    init_adc(adc_clock_speed_hz);
    init_pwm_driver();
    init_imu();
    init_comms();
    serial_init(uart_baud_rate);
    
}