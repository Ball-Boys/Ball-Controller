



const int ADC_CHANNEL_SELECT = 13; // MISO
const int ADC_CHIP_SELECT = 15;
const int ADC_SENSE_PIN = 12; // MOSI
const int ADC_SLK_PIN = 14; //SCL

static const int PWM_OUTPUT_BOUNDS[2] = {0, 255};



void init_adc(int clock_speed_hz);

void init_pwm_driver();

void init_imu();

void init_comms();

void serial_init(int baud_rate);


void init_peripherals(int adc_clock_speed_hz, int uart_baud_rate);


