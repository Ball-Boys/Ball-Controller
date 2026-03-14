#pragma once

#include "../control/BallController.h"
#include "../core/global_state.h"

class CalibrationSequence
{
private:
    BallController controller;
    int current_magnet_id = -1;
    int num_calibration_steps = 0;
    static const int MAX_CALIBRATION_STEPS = 3; // Number of calibration points

public:
    CalibrationSequence();

    // Start the next calibration step - returns the magnet ID to fire
    // Returns -1 if calibration is complete
    int startCalibrationStep();

    // Call this after dashboard sends joystick input to confirm magnet direction
    void completeCalibrationStep(float joy_x, float joy_y, const Quaternion &q);

    // Check if all calibration steps are done
    bool isCalibrated() const;

    // Get current step number for progress reporting
    int getCurrentStep() const { return num_calibration_steps; }
    int getMaxSteps() const { return MAX_CALIBRATION_STEPS; }
};
