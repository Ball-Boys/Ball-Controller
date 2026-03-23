#include "calibration.h"
#include "../core/global_state.h"
#include "../core/peripherals.h"
#include "../utils/utils.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

CalibrationSequence::CalibrationSequence()
    : num_calibration_steps(0)
{
}

int CalibrationSequence::startCalibration()
{
    // Get the next magnet to fire for calibration
    // For now, we'll get the orientation from global state
    GlobalState &state = GlobalState::instance();
    Orientation current_q = state.getOrientation();

    current_magnet_id = state.getCalibrationMagnet(current_q);

    printf("Calibration: Firing magnet %d\n", current_magnet_id);

    return current_magnet_id;
}

void CalibrationSequence::completeCalibrationStep(float joy_x, float joy_y, const Orientation &q)
{
    printf("Calibration result: User input (%.2f, %.2f)\n", joy_x, joy_y);
    GlobalState &state = GlobalState::instance();

    // Finalize this calibration step with the user's joystick input
    state.finishCalibration(current_magnet_id, q, joy_x, joy_y);

    printf("Calibration complete.\n");
}

bool CalibrationSequence::isCalibrated() const
{
    GlobalState &state = GlobalState::instance();
    return state.isCalibrated() && num_calibration_steps >= MAX_CALIBRATION_STEPS;
}
