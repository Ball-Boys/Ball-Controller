
#include <driver/spi_master.h>

const int ADC_CHANNEL_SELECT = 13; // MISO
const int ADC_CHIP_SELECT = 15;
const int ADC_SENSE_PIN = 12; // MOSI
const int ADC_SLK_PIN = 14; //SCL

const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const int I2C_CLOCK_HZ = 400000;

static const int PWM_OUTPUT_BOUNDS[2] = {0, 255};



void init_adc(int clock_speed_hz);

void init_pwm_driver();

void init_imu();

void init_comms();

void serial_init(int baud_rate);


void init_peripherals(int adc_clock_speed_hz, int uart_baud_rate);

spi_device_handle_t get_adc_device(int adc_gpio_address);

void pca9685_set_pwm(int driver_i2c_address, int channel, int value_0_255);


