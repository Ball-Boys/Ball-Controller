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
    int i = 0;
    Orientation current_q(1.0f, 0.0f, 0.0f, 0.0f);
    while (true) {
        IMUData imu_data = readIMU();
        if (imu_data.orientation.empty()) {
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait for orientation data
            i++;
            continue;
        }
        Orientation q = imu_data.orientation.back(); // Get latest orientation
        printf("Calibration step %d: Current orientation: w=%.3f x=%.3f y=%.3f z=%.3f\n", num_calibration_steps + 1, q.w, q.x, q.y, q.z);
        current_q = q;
        break;
    }
    

    current_magnet_id = state.getCalibrationMagnet(current_q);



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
