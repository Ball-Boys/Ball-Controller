#include "calibration.h"
#include "../core/global_state.h"
#include "../core/peripherals.h"
#include "../utils/utils.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

CalibrationSequence::CalibrationSequence()
    : controller(), num_calibration_steps(0)
{
}

int CalibrationSequence::startCalibrationStep()
{
    // If we've done all steps, calibration is complete
    if (num_calibration_steps >= MAX_CALIBRATION_STEPS)
    {
        return -1;
    }

    // Get the next magnet to fire for calibration
    // For now, we'll get the orientation from global state
    GlobalState &state = GlobalState::instance();
    Orientation current_q = state.getOrientation();
    Quaternion q(current_q.w, current_q.x, current_q.y, current_q.z);

    current_magnet_id = controller.getCalibrationMagnet(q);

    printf("Calibration step %d/%d: Firing magnet %d\n",
           num_calibration_steps + 1, MAX_CALIBRATION_STEPS, current_magnet_id);

    return current_magnet_id;
}

void CalibrationSequence::completeCalibrationStep(float joy_x, float joy_y, const Quaternion &q)
{
    if (current_magnet_id < 0)
    {
        printf("ERROR: No magnet was fired in this step\n");
        return;
    }

    printf("Calibration step %d: User input (%.2f, %.2f)\n",
           num_calibration_steps + 1, joy_x, joy_y);

    // Finalize this calibration step with the user's joystick input
    controller.finishCalibration(current_magnet_id, q, joy_x, joy_y);

    // Move to next step
    num_calibration_steps++;
    current_magnet_id = -1;

    printf("Calibration step %d complete. Progress: %d/%d\n",
           num_calibration_steps, num_calibration_steps, MAX_CALIBRATION_STEPS);
}

bool CalibrationSequence::isCalibrated() const
{
    return controller.isCalibrated() && num_calibration_steps >= MAX_CALIBRATION_STEPS;
}
