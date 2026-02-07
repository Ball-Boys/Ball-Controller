#include <driver/spi_master.h>
#include "driver/uart.h"

#include "utils/utils.h"

#include <core/global_state.h>
#include <core/peripherals.h>

#include <vector>


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
        currentValues.push_back(0.0f); // TODO: replace 0.0f with actual value read from ADC using adcAddress

    }

    return currentValues;
}

void setPWMOutputs(std::vector<int> magnetIds, std::vector<int> values) {
    // TODO: implement this for realsies

}



inline void serial_print(const char* msg) {
    uart_write_bytes(UART_NUM_0, msg, strlen(msg));
}

