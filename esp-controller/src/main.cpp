#include "core/global_state.h"
#include "control/control_algorithm.h"
#include "tasks/imu_task.h"
#include "tasks/magnet_task.h"
#include "tasks/comms_task.h"
#include "calibration/calibration.h"
#include "ota/ota_update.h"

void setup() {
    // Startup sequence
    runCalibration();

    // Continuous tasks
    startImuTask();
    startMagnetTask();
    startCommsTask();
    startOtaUpdateTask();

    // TODO: replace with FreeRTOS scheduler or framework-specific loop
}

void loop() {
    // TODO: main loop here
}
