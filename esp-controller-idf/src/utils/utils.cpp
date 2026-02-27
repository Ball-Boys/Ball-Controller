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

    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_USE_TXDATA,  // Use internal buffers
        .length = 16,      // Total bits to transmit
        .rxlength = 16,    // Bits to receive
        .tx_buffer = NULL,
        .rx_buffer = NULL,
    };
    
    // Copy to internal buffers
    t.tx_data[0] = tx[0];
    t.tx_data[1] = tx[1];
    t.tx_data[2] = 0;
    t.tx_data[3] = 0;

    esp_err_t ret = spi_device_transmit(dev, &t);
    if (ret != ESP_OK) {
        serial_printf("SPI error: %d\n", ret);
        return 0;
    }

    // Read from internal rx buffer
    int value = ((t.rx_data[0] & 0x0F) << 8) | t.rx_data[1];
    serial_printf("ADC ch%d: rx[0]=0x%02X rx[1]=0x%02X value=%d\n", 
                  adcAddress.channel, t.rx_data[0], t.rx_data[1], value);
    
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

    // printf("Retrieved current values from ADC for magnet IDs: ");
    // for (size_t i = 0; i < mag_ids.size(); ++i)
    // {
    //     printf("%d ", mag_ids[i]);
    // }
    // printf("\nValues: ");
    // for (const auto& value : currentValues) {
    //     printf("%d ", value);
    // }
    // printf("\n");

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

