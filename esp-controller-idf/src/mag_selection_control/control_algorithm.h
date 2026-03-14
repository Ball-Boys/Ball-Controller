#pragma once

#include "../core/global_state.h"

ControlOutputs computeControl(const std::vector<Orientation> orientation_history, const std::vector<AngularVelocity> angular_velocity_history, const Vector3 targetDirection);
