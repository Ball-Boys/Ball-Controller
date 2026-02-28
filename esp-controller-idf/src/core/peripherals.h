#pragma once

#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>

const gpio_num_t ADC_CHANNEL_SELECT[] = {GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33}; 
const int ADC_MISO_PIN = GPIO_NUM_19; // MISO

const int ADC_MOSI_PIN = GPIO_NUM_23; // MOSI
const int ADC_SLK_PIN = GPIO_NUM_18; //SCL

static constexpr gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
static constexpr gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;
static constexpr int I2C_CLOCK_HZ = 400000;



static const int PWM_OUTPUT_BOUNDS[2] = {0, 255};



void init_adc(int clock_speed_hz, gpio_num_t chip_select_pin);

void init_pwm_driver();

void init_imu();

void init_comms();

void serial_init(int baud_rate);


void init_peripherals(int adc_clock_speed_hz, int uart_baud_rate);

spi_device_handle_t get_adc_device(gpio_num_t adc_gpio_address);

uint16_t adc1283_read(gpio_num_t chip_select, int channel);

void pca9685_set_pwm(int driver_i2c_address, int channel, int value_0_255);


