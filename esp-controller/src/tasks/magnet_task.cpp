#include "magnet_task.h"
#include "../core/global_state.h"

void startMagnetTask() {
    GlobalState& state = GlobalState::instance();



    while (true) {

        // this gets a list of all non-zero control outputs 
        // TODO: consider an optimization around setting a flag when the control outputs have changed instead of polling
        const auto& new_control_outputs = state.getLatestControl();

        // TODO: zero the previous magnets before new values. 
        for (const auto& output : new_control_outputs) {
            int magnetId = output.magnetId;
            float current_value = output.current_value;

            PWMAddress pwmAddr = state.getPWMAddress(magnetId);
            ADCAddress adcAddr = state.getADCAddress(magnetId);

            // Read the current and do whatever you need to to mantain the current at the given magnets at the right value.
        }
    }
    
}


