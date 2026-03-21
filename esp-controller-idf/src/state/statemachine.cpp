#include <iostream>
#include "core/peripherals.h"
#include "core/global_state.h"
#include "mag_selection_control/control_algorithm.h"
#include "calibration/calibration.h"
#include <esp_timer.h>
#include <comms/wifi_client.h>
#include <scripts/bench_test.h>
#include <ota/ota_update.h>
#include <utils/utils.h>

#define SERIAL_BAUD_RATE 115200
#define I2C_CLOCK_HZ 1000000

// Forward declarations
class State;
class ConnectionState;
class StandbyState;
class CalibrateState;
class RunningState;
class TestingState;

// ---------------------------------------------------------
// 1. Base State Interface
// ---------------------------------------------------------
class State
{
public:
    virtual ~State() = default;
    // Each state runs its own blocking loop and returns a pointer to the NEXT state
    virtual State *execute() = 0;
};

// ---------------------------------------------------------
// 2. Concrete States
// ---------------------------------------------------------

class ConnectionState : public State
{
public:
    static ConnectionState &getInstance()
    {
        static ConnectionState instance;
        return instance;
    }

    State *execute() override;

private:
    ConnectionState() = default;
};

class StandbyState : public State
{
public:
    static StandbyState &getInstance()
    {
        static StandbyState instance;
        return instance;
    }

    State *execute() override;

private:
    StandbyState() = default;
};

class CalibrateState : public State
{
public:
    static CalibrateState &getInstance()
    {
        static CalibrateState instance;
        return instance;
    }

    State *execute() override;

private:
    CalibrateState() = default;
};

class RunningState : public State
{
public:
    static RunningState &getInstance()
    {
        static RunningState instance;
        return instance;
    }

    State *execute() override;

private:
    RunningState() = default;
};

class TestingState : public State
{
public:
    static TestingState &getInstance()
    {
        static TestingState instance;
        return instance;
    }

    State *execute() override;

private:
    TestingState() = default;
};

// ---------------------------------------------------------
// 3. State Implementations (The Meat)
// ---------------------------------------------------------

// Static task handles to avoid spawning duplicate UDP tasks across state transitions
static TaskHandle_t s_udp_sender_handle = NULL;
static TaskHandle_t s_udp_receiver_handle = NULL;
static bool s_ota_server_started = false;

static void ensure_udp_sender()
{
    if (s_udp_sender_handle == NULL)
    {
        xTaskCreate(udp_sender_task, "udp_sender", 8192, NULL, 4, &s_udp_sender_handle);
        printf("Started UDP sender task\n");
    }
}

static void ensure_udp_receiver()
{
    if (s_udp_receiver_handle == NULL)
    {
        xTaskCreate(udp_receiver_task, "udp_receiver", 8192, NULL, 4, &s_udp_receiver_handle);
        printf("Started UDP receiver task\n");
    }
}

State *ConnectionState::execute()
{
    printf("=== ConnectionState: Starting WiFi connection ===\n");
    init_peripherals(I2C_CLOCK_HZ, SERIAL_BAUD_RATE);

    // WiFi is initialized in init_peripherals and started with AP mode
    // Wait for WiFi to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    printf("WiFi connection established, moving to StandbyState\n");
    GlobalState &state = GlobalState::instance();
    state.setSystemState(GlobalState::SystemState::STANDBY);

    // Start comms tasks early so the dashboard can detect the connection
    ensure_udp_sender();
    ensure_udp_receiver();

    // Start OTA HTTP server so firmware can be flashed via WiFi
    if (!s_ota_server_started)
    {
        startOtaUpdateTask();
        s_ota_server_started = true;
    }

    return &StandbyState::getInstance();
}

State *StandbyState::execute()
{
    printf("=== StandbyState: Waiting for calibration command from dashboard ===\n");
    GlobalState &state = GlobalState::instance();
    state.setSystemState(GlobalState::SystemState::STANDBY);

    // Ensure comms tasks are running (no-op if already started)
    ensure_udp_sender();
    ensure_udp_receiver();

    // Wait for calibration request from dashboard
    while (!state.getCalibrationRequested())
    {
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }

    state.clearCalibrationRequest();
    printf("Received calibration command, moving to CalibrateState\n");

    return &CalibrateState::getInstance();
}

State *CalibrateState::execute()
{
    printf("=== CalibrateState: Running calibration sequence ===\n");
    GlobalState &global = GlobalState::instance();
    global.setSystemState(GlobalState::SystemState::CALIBRATION);

    // Create calibration sequence
    CalibrationSequence calibration;

    // Ensure comms tasks are running (no-op if already started)
    ensure_udp_sender();
    ensure_udp_receiver();

    // Get the next magnet to fire
    int magnet_id = calibration.startCalibration();

    // TODO: Apply magnet current to fire the selected magnet
    // For now, just a placeholder
    printf("Firing magnet %d for calibration\n", magnet_id);

    // Wait for user input from dashboard (set_direction command)
    global.clearCalibrationInput();
    while (!global.getCalibrationInputAvailable())
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // Wait for input
    }

    // Get user input and current orientation
    Vector3 user_input = global.getCalibrationInput();
    global.clearCalibrationInput();

    Orientation current_q = global.getOrientation();
    Quaternion q(current_q.w, current_q.x, current_q.y, current_q.z);

    // Complete this calibration step
    calibration.completeCalibrationStep(user_input.x, user_input.y, q);

    printf("Calibration finished, waiting for start command...\n");

    // Wait for start command to move to running state
    while (!global.getStartRequested())
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    global.clearStartRequest();
    printf("Received start command, moving to RunningState\n");

    return &RunningState::getInstance();
}

void core1LoopTaskTest(void *param)
{
    GlobalState& instance = GlobalState::instance();
    float current_value = 3.0f;
    while (true)
    {

        instance.setControl(ControlOutputs(1, current_value));
        printf("Setting Control to %f", current_value);
        if (current_value == 0.0f)
        {
            current_value = 3.0f;
        }
        else
        {
            current_value = 0.0f;
        }


        const int64_t interval_us = static_cast<int64_t>(instance.fastLoopTime * 1000000.0f);

        const int64_t slow_loop_time_us = 3 * 1000000.0f; // instance.slowLoopTime * 1000000.0f;
        const int64_t fast_loop_time_us = instance.fastLoopTime * 1000000.0f;

        const int64_t end_us = esp_timer_get_time() + slow_loop_time_us;
        int64_t fast_loop_end_us = esp_timer_get_time() + fast_loop_time_us;

        while (esp_timer_get_time() < end_us)
        {
            fast_loop_end_us = esp_timer_get_time() + fast_loop_time_us;

            instance.currentControlLoop();

            if (esp_timer_get_time() < fast_loop_end_us)
            {
                vTaskDelay(pdMS_TO_TICKS(0.0001f)); // slight smoothing of operation here
            }
        }
    }
}

void core1LoopTask(void *param)
{
    // This is the task that runs on Core 1 for the 10ms control loop
    GlobalState &instance = GlobalState::instance();

    while (true)
    {
        if (instance.isKilled())
        {
            // Reset the kill flag for the next run
            break; // Exit the loop to end the task
        }
        // check IMU and get value
        IMUData imu_data = readIMU();

        for (const auto& angular_velocity : imu_data.angular_velocity) {
            instance.setAngularVelocity(angular_velocity);
        }

        for (const auto& orientation : imu_data.orientation) {
            instance.setOrientation(orientation);
        }

        // compute control outputs
        ControlOutputs control_outputs = computeControl(instance.getOrientationHistory(10), instance.getAngularVelocityHistory(10), instance.getIdealDirection());
        instance.setControl(control_outputs);

        const int64_t interval_us = static_cast<int64_t>(instance.fastLoopTime * 1000000.0f);

        const int64_t slow_loop_time_us = instance.slowLoopTime * 1000000.0f;
        const int64_t fast_loop_time_us = instance.fastLoopTime * 1000000.0f;

        const int64_t end_us = esp_timer_get_time() + slow_loop_time_us;
        int64_t fast_loop_end_us = esp_timer_get_time() + fast_loop_time_us;

        while (esp_timer_get_time() < end_us)
        {
            fast_loop_end_us = esp_timer_get_time() + fast_loop_time_us;

            instance.currentControlLoop();

            if (esp_timer_get_time() < fast_loop_end_us)
            {
                vTaskDelay(pdMS_TO_TICKS(0.0001f)); // slight smoothing of operation here
            }
        }
    }

    // TODO: implement cleanup logic here (e.g. set all PWM outputs to 0)

    vTaskDelete(NULL); // Clean up the task
}

State *RunningState::execute()
{
    printf("=== RunningState: Starting normal operation ===\n");
    GlobalState &state = GlobalState::instance();
    state.setSystemState(GlobalState::SystemState::RUNNING);

    // 1. Spawn Core 1 Task for the 10ms control loop
    xTaskCreatePinnedToCore(core1LoopTaskTest, "Core1", 4096, NULL, 1, NULL, 1);
    printf("Started Core 1 control loop task\n");

    // 2. Ensure comms tasks are running (no-op if already started)
    ensure_udp_sender();
    ensure_udp_receiver();

    // Main loop - check for calibration requests or stop
    while (true)
    {
        // Check if stop requested (kill flag)
        if (state.isKilled())
        {
            printf("Stop requested, stopping control loop\n");
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for control task to exit
            state.set_kill(false);
            state.zeroControl();
            printf("Returning to StandbyState\n");
            return &StandbyState::getInstance();
        }

        // Check if recalibration is requested
        if (state.getCalibrationRequested())
        {
            state.clearCalibrationRequest();
            printf("Recalibration requested, stopping control loop\n");

            // Kill the control task to pause operation
            state.set_kill(true);
            vTaskDelay(pdMS_TO_TICKS(500)); // Wait for tasks to clean up
            state.set_kill(false);

            printf("Returning to CalibrateState for recalibration\n");
            return &CalibrateState::getInstance();
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }

    return &StandbyState::getInstance(); // Placeholder - should only return if emergency stop occurs
}

State *TestingState::execute()
{
    // Run scripts through it. No state switching ever.

    // test_imu();
    test_0();
    // test_stress_20ms();
    // test_4();
    // test_5();

    return nullptr; // Should never reach here
}

class StateMachine
{
private:
    State *currentState;

public:
    // Initialize with the starting state (Testing or Connection based on flash)
    StateMachine(State *initialState) : currentState(initialState) {}

    void run()
    {
        // The main task loop. It simply executes the current state,
        // which blocks until a transition is needed, then updates the pointer.
        while (currentState != nullptr)
        {
            currentState = currentState->execute();
        }
    }
};

void run_state_machine_connection()
{
    StateMachine machine(&ConnectionState::getInstance());
    machine.run();
}

void run_state_machine_testing()
{
    StateMachine machine(&TestingState::getInstance());
    machine.run();
}
