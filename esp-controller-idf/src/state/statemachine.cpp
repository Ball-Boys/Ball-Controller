#include <iostream>
#include "core/peripherals.h"
#include "core/global_state.h"
#include "mag_selection_control/control_algorithm.h"
#include <esp_timer.h>
#include <comms/wifi_client.h>
#include <scripts/bench_test.h>
#include "utils/utils.h"

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
class State {
public:
    virtual ~State() = default;
    // Each state runs its own blocking loop and returns a pointer to the NEXT state
    virtual State* execute() = 0; 
};

// ---------------------------------------------------------
// 2. Concrete States
// ---------------------------------------------------------

class ConnectionState : public State {
public:
    static ConnectionState& getInstance() {
        static ConnectionState instance;
        return instance;
    }

    State* execute() override;
private:
    ConnectionState() = default;
};

class StandbyState : public State {
public:
    static StandbyState& getInstance() {
        static StandbyState instance;
        return instance;
    }

    State* execute() override;
private:
    StandbyState() = default;
};

class CalibrateState : public State {
public:
    static CalibrateState& getInstance() {
        static CalibrateState instance;
        return instance;
    }

    State* execute() override;
private:
    CalibrateState() = default;
};

class RunningState : public State {
public:
    static RunningState& getInstance() {
        static RunningState instance;
        return instance;
    }

    State* execute() override;
private:
    RunningState() = default;
};

class TestingState : public State {
public:
    static TestingState& getInstance() {
        static TestingState instance;
        return instance;
    }

    State* execute() override;
private:
    TestingState() = default;
};

// ---------------------------------------------------------
// 3. State Implementations (The Meat)
// ---------------------------------------------------------

State* ConnectionState::execute() {
    init_peripherals(I2C_CLOCK_HZ, SERIAL_BAUD_RATE); // Initialize peripherals (Wi-Fi, IMU, etc.)

    
    // TODO: Implement WiFi connection handshake here. 
        
    return &StandbyState::getInstance();
        
    
}

State* StandbyState::execute() {
    // TODO: Implement logic to wait for a command from the server to start calibration or running mode.
    return &CalibrateState::getInstance(); // Placeholder to transition to calibration immediately for now
}

State* CalibrateState::execute() {
    // TODO: implement actual calibration logic here
    GlobalState& global = GlobalState::instance();
    global.setOffset({1.0f, 0.0f, 0.0f, 0.0f}); // Placeholder to set zero offset immediately for now
    return &RunningState::getInstance();
}

void core1LoopTask(void* param) {
    // This is the task that runs on Core 1 for the 10ms control loop
    GlobalState& instance = GlobalState::instance();
    
    while (true) {
        if (instance.isKilled()) {
            // Reset the kill flag for the next run
            break; // Exit the loop to end the task
        }
        // check IMU and get value
        readIMU();

        // compute control outputs
        ControlOutputs control_outputs = computeControl(instance.getOrientationHistory(10), instance.getAngularVelocityHistory(10), instance.getIdealDirection());
        instance.setControl(control_outputs);

        const int64_t interval_us = static_cast<int64_t>(instance.fastLoopTime * 1000000.0f);
    

        const int64_t slow_loop_time_us = instance.slowLoopTime * 1000000.0f;
        const int64_t fast_loop_time_us = instance.fastLoopTime * 1000000.0f;

        const int64_t end_us = esp_timer_get_time() + slow_loop_time_us;
        int64_t fast_loop_end_us = esp_timer_get_time() + fast_loop_time_us;
        
        while (esp_timer_get_time() < end_us) {
            fast_loop_end_us = esp_timer_get_time() + fast_loop_time_us;
            
            instance.currentControlLoop();
            
            if (esp_timer_get_time() < fast_loop_end_us) {
                vTaskDelay(pdMS_TO_TICKS(0.0001f)); // slight smoothing of operation here
            }
        }
    }

    // TODO: implement cleanup logic here (e.g. set all PWM outputs to 0)

    vTaskDelete(NULL); // Clean up the task
}

State* RunningState::execute() {
    // 1. Spawn Core 1 Task here for the 10ms loop
    xTaskCreatePinnedToCore(core1LoopTask, "Core1", 4096, NULL, 1, NULL, 1);

    // 2. Core 0 blocks in this loop, sending telemetry
    while (true) {
        // TODO: implement 2-way telemetry logic here.
        // xTaskCreate(udp_sender_task, "udp_sender", 32768, NULL, 5, NULL);


        vTaskDelay(pdMS_TO_TICKS(1000)); // Main loop sleeps, task runs independently
    }
        

    
    

    return &StandbyState::getInstance(); // Placeholder to transition back to standby after running for now
}



State* TestingState::execute() {
    // Run scripts through it. No state switching ever.
    
            // test_imu();
            //test_1();
            //test_stress_20ms();
            //test_4();
            //test_5();
            test_that_wiggle();
    
    return nullptr; // Should never reach here
}


class StateMachine {
private:
    State* currentState;

public:
    // Initialize with the starting state (Testing or Connection based on flash)
    StateMachine(State* initialState) : currentState(initialState) {}

    void run() {
        // The main task loop. It simply executes the current state, 
        // which blocks until a transition is needed, then updates the pointer.
        while (currentState != nullptr) {
            currentState = currentState->execute();
        }
    }
};

void run_state_machine_connection() {
    StateMachine machine(&ConnectionState::getInstance());
    machine.run();
}

void run_state_machine_testing() {
    StateMachine machine(&TestingState::getInstance());
    machine.run();
}

