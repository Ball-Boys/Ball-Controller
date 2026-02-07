#include <driver/spi_master.h>
#include <driver/i2c.h>
#include <driver/uart.h>
#include <unordered_map>

#include "peripherals.h"

namespace {
static bool s_adc_bus_initialized = false;
static int s_adc_clock_hz = 0;
static std::unordered_map<int, spi_device_handle_t> s_adc_devices;

static bool s_i2c_initialized = false;
static constexpr i2c_port_t kI2cPort = I2C_NUM_0;
static constexpr uint8_t kPca9685BaseAddr = 0x40;
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

    spi_device_interface_config_t devcfg = {};
    devcfg.clock_speed_hz = s_adc_clock_hz > 0 ? s_adc_clock_hz : 1000000;
    devcfg.mode = 0;
    devcfg.spics_io_num = adc_gpio_address;
    devcfg.queue_size = 1;

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

    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA_PIN;
    conf.scl_io_num = I2C_SCL_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_CLOCK_HZ;

    i2c_param_config(kI2cPort, &conf);
    i2c_driver_install(kI2cPort, conf.mode, 0, 0, 0);
    s_i2c_initialized = true;
}

void pca9685_set_pwm(int driver_i2c_address, int channel, int value_0_255) {
    if (!s_i2c_initialized) {
        init_pwm_driver();
    }

    if (channel < 0 || channel > 15) {
        return;
    }

    if (value_0_255 < PWM_OUTPUT_BOUNDS[0]) {
        value_0_255 = PWM_OUTPUT_BOUNDS[0];
    }
    if (value_0_255 > PWM_OUTPUT_BOUNDS[1]) {
        value_0_255 = PWM_OUTPUT_BOUNDS[1];
    }

    const uint8_t i2c_addr = static_cast<uint8_t>(kPca9685BaseAddr + driver_i2c_address);
    const int on_count = 0;
    const int off_count = (value_0_255 * 4095) / 255;

    const uint8_t reg = static_cast<uint8_t>(0x06 + 4 * channel); // LED0_ON_L

    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on_count & 0xFF),
        static_cast<uint8_t>((on_count >> 8) & 0x0F),
        static_cast<uint8_t>(off_count & 0xFF),
        static_cast<uint8_t>((off_count >> 8) & 0x0F)
    };

    i2c_master_write_to_device(kI2cPort, i2c_addr, data, sizeof(data), pdMS_TO_TICKS(10));
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