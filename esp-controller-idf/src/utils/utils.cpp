#include <driver/spi_master.h>
#include "driver/uart.h"

#include "utils/utils.h"

#include <core/global_state.h>
#include <core/peripherals.h>

#include <vector>
#include <cstdarg>
#include <cstdio>
#include <cstring>


namespace {
int read_adc1283_channel(const ADCAddress& adcAddress) {
    
    int value = adc1283_read(adcAddress.adc_gpio_address, adcAddress.channel);
    
    return value;
}
}

float convert_adc_value_to_current(u_int16_t adc_value) {
    const float max_adc_value = 4095.0f;
    const float max_voltage = 3.3f; // Max voltage

    return (adc_value / max_adc_value) * max_voltage / 50 / 0.005;
}

void readIMU() {
    shtp_service();
}


std::vector<float> retreveCurrentValueFromADC(std::vector<int> mag_ids) {
    GlobalState& state = GlobalState::instance();
    std::vector<ADCAddress> adcAddresses;
    std::vector<float> currentValues;

    

    for (const auto& mag_id : mag_ids) {
        adcAddresses.push_back(state.getADCAddress(mag_id));
    }
    

    for (size_t i = 0; i < mag_ids.size(); ++i) {
        int magnetId = mag_ids[i];
        ADCAddress adcAddress = adcAddresses[i];
        (void)magnetId;
        uint16_t raw_value = read_adc1283_channel(adcAddress);
        float current = convert_adc_value_to_current(raw_value);
        currentValues.push_back(current);
    }



    return currentValues;
}

void setPWMOutputs(std::vector<int> magnetIds, std::vector<int> values) {
    GlobalState& state = GlobalState::instance();
    const size_t count = std::min(magnetIds.size(), values.size());



    // serial_printf("Setting PWM outputs for magnet IDs: ");
    // for (size_t i = 0; i < count; ++i) {
    //     serial_printf("%d ", magnetIds[i]);
    // }
    // serial_print("\n");

    // serial_printf("With values: ");
    // for (size_t i = 0; i < count; ++i) {
    //     serial_printf("%d ", values[i]);
    // }
    // serial_print("\n");


    for (size_t i = 0; i < count; ++i) {
        int magnetId = magnetIds[i];
        int value = values[i];
        PWMAddress pwmAddress = state.getPWMAddress(magnetId);
        pca9685_set_pwm(pwmAddress.driver_i2c_address, pwmAddress.channel, value);
        // printf("Set PWM for magnet %d (I2C addr: 0x%02X, channel: %d) to value %d\n", magnetId, pwmAddress.driver_i2c_address, pwmAddress.channel, value);

    }

}



void serial_print(const char* msg) {
    if (!msg) {
        return;
    }
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
}

void serial_printf(const char* fmt, ...) {
    if (!fmt) {
        return;
    }

    char buffer[256];
    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    if (written <= 0) {
        return;
    }

    size_t len = static_cast<size_t>(written);
    if (len >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }
    uart_write_bytes(UART_NUM_0, buffer, len);
}

