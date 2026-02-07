#include "control_algorithm.h"

// TODO: make this return a list instead of a single output 
ControlOutputs computeControl(const Orientation& current, const float targetDirection[3]) {
    (void)current;
    (void)targetDirection;

    ControlOutputs outputs = ControlOutputs::zero(0);
    // TODO: implement control logic to map orientation and target vector to magnet commands
    return outputs;
}
