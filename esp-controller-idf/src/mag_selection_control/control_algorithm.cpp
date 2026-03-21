#include "control_algorithm.h"

// Singleton instance of the BallController
static BallController *g_controller = nullptr;

BallController &getControllerInstance()
{
    if (g_controller == nullptr)
    {
        g_controller = new BallController();
    }
    return *g_controller;
}

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
    Quaternion q(latest_orient.w, latest_orient.x, latest_orient.y, latest_orient.z);

    // Get the ball controller instance
    BallController &controller = getControllerInstance();

    // Prepare output array for up to 2 magnets
    MagnetCommand outputs[2];
    int num_magnets = controller.solve(targetDirection.x, targetDirection.y, q, outputs);

    // Return the first magnet command
    // TODO: Update GlobalState and statemachine to handle multiple magnet outputs
    std::vector<ControlOutputs> controlOutputs;  
    for (auto& output: outputs)
    {
        controlOutputs.push_back(ControlOutputs(outputs[0].id, outputs[0].current));
    }

    return {};
}
