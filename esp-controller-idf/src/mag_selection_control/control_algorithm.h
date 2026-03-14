#pragma once

#include "../core/global_state.h"
#include "../control/BallController.h"

// Compute control outputs for a single magnet
// Returns a ControlOutputs with current command, or zero if no magnet should fire
ControlOutputs computeControl(const std::vector<Orientation> orientation_history, const std::vector<AngularVelocity> angular_velocity_history, const Vector3 targetDirection);

// Internal helper to get singleton BallController instance
BallController &getControllerInstance();
