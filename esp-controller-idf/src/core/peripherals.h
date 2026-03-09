#pragma once

#include <driver/spi_master.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>

#define EXAMPLE_ESP_WIFI_SSID      "ESP32_Data_Link"
#define EXAMPLE_ESP_WIFI_PASS      "password123"
#define PORT                        5005
#define RECV_IP_ADDR               "192.168.4.2"

static const char *TAG = "WIFI_DATA_LINK";


const gpio_num_t ADC_CHANNEL_SELECT[] = {GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33}; 
const int ADC_MISO_PIN = GPIO_NUM_19; // MISO

const int ADC_MOSI_PIN = GPIO_NUM_23; // MOSI
const int ADC_SLK_PIN = GPIO_NUM_18; //SCL

static gpio_num_t I2C_SDA_PIN = GPIO_NUM_21;
static gpio_num_t I2C_SCL_PIN = GPIO_NUM_22;
static int I2C_CLOCK_HZ = 400000;



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

// IMU functions
void update_imu_data();
bool read_imu_quaternion(float& w, float& x, float& y, float& z);
bool read_imu_angular_velocity(float& x, float& y, float& z);
bool read_imu_accelerometer(float& x, float& y, float& z);
bool read_imu_magnetometer(float& x, float& y, float& z);
bool imu_data_available();


