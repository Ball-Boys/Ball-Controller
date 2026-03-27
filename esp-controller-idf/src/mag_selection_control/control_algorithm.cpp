#include "control_algorithm.h"



// Compute control outputs using BallController solver
// Takes orientation and target direction, returns control for one magnet
// TODO: Update to return vector for dual magnet operation
std::vector<ControlOutputs> computeControl(const std::vector<Orientation> orientation_history, const std::vector<AngularVelocity> angular_velocity_history, const Vector3 targetDirection)
{
    
    // Use the most recent orientation
    if (orientation_history.empty())
    {
        return {};
    }

    const Orientation &latest_orient = orientation_history.back();

    // Get the ball controller instance
    GlobalState &state = GlobalState::instance();
    
    std::vector<ControlOutputs> outputs = state.solve(targetDirection.x, targetDirection.y, latest_orient);

    // TODO; Update this so we actually use the outputs from the solver.
    return outputs;
}
