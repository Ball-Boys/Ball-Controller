#include <driver/spi_master.h>
#include "peripherals.h"

void init_adc(int clock_speed_hz) {
    spi_device_handle_t spi;

    spi_bus_config_t buscfg = {
        .mosi_io_num = ADC_SENSE_PIN,
        .miso_io_num = ADC_CHANNEL_SELECT,
        .sclk_io_num = ADC_SLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = clock_speed_hz,
    };

    
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