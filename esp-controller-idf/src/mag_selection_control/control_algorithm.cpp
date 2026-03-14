#include "control_algorithm.h"

// TODO: make this return a list instead of a single output 
ControlOutputs computeControl(const std::vector<Orientation> orientation_history, const std::vector<AngularVelocity> angular_velocity_history, const Vector3 targetDirection) {

    ControlOutputs outputs = ControlOutputs::zero(0);
    // TODO: implement control logic to map orientation and target vector to magnet commands
    return outputs;
}
