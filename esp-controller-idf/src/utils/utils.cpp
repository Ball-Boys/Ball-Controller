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
    spi_device_handle_t dev = get_adc_device(adcAddress.adc_gpio_address);

    uint8_t tx[2] = {0};
    uint8_t rx[2] = {0};

    // ADC1283: assume MSB-first, 12-bit data. Channel is encoded in high bits.
    tx[0] = static_cast<uint8_t>((adcAddress.channel & 0x0F) << 4);
    tx[1] = 0x00;

    spi_transaction_t t = {};
    t.length = 16;
    t.tx_buffer = tx;
    t.rx_buffer = rx;

    spi_device_transmit(dev, &t);

    int value = ((rx[0] & 0x0F) << 8) | rx[1];
    return value;
}
}

std::vector<int> retreveCurrentValueFromADC(std::vector<int> mag_ids) {
    GlobalState& state = GlobalState::instance();
    std::vector<ADCAddress> adcAddresses;
    std::vector<int> currentValues;

    for (const auto& mag_id : mag_ids) {
        adcAddresses.push_back(state.getADCAddress(mag_id));
    }

    for (size_t i = 0; i < mag_ids.size(); ++i) {
        int magnetId = mag_ids[i];
        ADCAddress adcAddress = adcAddresses[i];
        (void)magnetId;
        currentValues.push_back(read_adc1283_channel(adcAddress));

    }

    return currentValues;
}

void setPWMOutputs(std::vector<int> magnetIds, std::vector<int> values) {
    GlobalState& state = GlobalState::instance();
    const size_t count = std::min(magnetIds.size(), values.size());

    for (size_t i = 0; i < count; ++i) {
        int magnetId = magnetIds[i];
        int value = values[i];
        PWMAddress pwmAddress = state.getPWMAddress(magnetId);
        pca9685_set_pwm(pwmAddress.driver_i2c_address, pwmAddress.channel, value);
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

