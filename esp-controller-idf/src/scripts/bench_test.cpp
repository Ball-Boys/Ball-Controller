// in test 1 we will sweep through turning each magnet on one by one for 1 second each.
#include "core/global_state.h"
#include "utils/utils.h"

#include "esp_timer.h"

#include <cstdint>
#include <string>

namespace {
void run_control_loop_for_seconds(GlobalState& instance, float duration_s) {
    if (duration_s <= 0.0f) {
        return;
    }

    const int64_t interval_us = static_cast<int64_t>(instance.fastLoopTime * 1000000.0f);
    if (interval_us <= 0) {
        return;
    }

    const int64_t end_us = esp_timer_get_time() + static_cast<int64_t>(duration_s * 1000000.0f);
    int64_t next_us = esp_timer_get_time();

    while (esp_timer_get_time() < end_us) {
        int64_t now_us = esp_timer_get_time();
        if (now_us >= next_us) {
            instance.currentControlLoop();
            next_us += interval_us;
        }
    }
}
} // namespace

void test_1() {

    serial_print("Starting test 1: Magnet sweep\n");


    GlobalState& instance = GlobalState::instance();

    float loop_iterations = 1.0f / instance.fastLoopTime; // Set loop iterations based on fast loop time

    int loops = 0;
    // loop though index 0 through 19 magents
    for (int mag_id = 1; mag_id <= 20; ++mag_id) {
        serial_printf("Activating magnet %d\n", mag_id);
        instance.setControl(ControlOutputs(mag_id, 255)); // Set magnet to max power

        run_control_loop_for_seconds(instance, 1.0f);

    }
}


// in test 2 we will do a very similar process to the one above but we will turn on 2 magnets at a time.

void test_2() {
    serial_print("Starting test 2: Two magnet sweep\n");

    GlobalState& instance = GlobalState::instance();

    float loop_iterations = 1.0f / instance.fastLoopTime; // Set loop iterations based on fast loop time

    int loops = 0;
    // loop though index 0 through 19 magents
    for (int mag_id = 1; mag_id <= 20; mag_id += 2) {
        serial_printf("Activating magnets %d and %d\n", mag_id, mag_id + 1);
        instance.setControl(ControlOutputs(mag_id, 255)); // Set magnet to max power
        instance.setControl(ControlOutputs(mag_id + 1, 255)); // Set next magnet to max power

        run_control_loop_for_seconds(instance, 1.0f);

    }
}


// In test 3 we will turn on magnet 0 at 10 ms intervals to random values between 0 and 255.
void test_3() {
    serial_print("Starting test 3: Random magnet activation\n");

    GlobalState& instance = GlobalState::instance();

    float loop_iterations = 0.010f / instance.fastLoopTime; // Set loop iterations based on fast loop time

    int loops = 0;
    // loop though index 0 through 19 magents
    for (int mag_id = 1; mag_id <= 20; ++mag_id) {
        int random_value = rand() % 256; // Generate random value between 0 and 255
        serial_printf("Activating magnet %d with value %d\n", mag_id, random_value);
        instance.setControl(ControlOutputs(mag_id, random_value)); // Set magnet to random power

        run_control_loop_for_seconds(instance, 0.01f);

    }
}

// in Test 4 we will turn on 2 magnets at a time at random values between 0 and 255 at 10 ms intervals.
void test_4() {
    serial_print("Starting test 4: Random two magnet activation\n");

    GlobalState& instance = GlobalState::instance();
    float loop_iterations = 0.01f / instance.fastLoopTime; // Set loop iterations based on fast loop time
    int loops = 0;
    // loop though index 0 through 19 magents
    for (int mag_id = 1; mag_id <= 20; mag_id += 2) {
        int random_value_1 = rand() % 256; // Generate random
        int random_value_2 = rand() % 256; // Generate random value between 0 and 255
        serial_printf("Activating magnets %d and %d with values %d and %d\n", mag_id, mag_id + 1, random_value_1, random_value_2);
        instance.setControl(ControlOutputs(mag_id, random_value_1)); // Set magnet to random power
        instance.setControl(ControlOutputs(mag_id + 1, random_value_2)); // Set next magnet to random power 

        run_control_loop_for_seconds(instance, 0.01f);
    }
}


// In test 5 we will test the loop timing by turning on 1, 2, 3, 4, and 5 magnets at a time at random values between 0 and 255 at 10 ms intervals and measuring the loop time for each case.

void test_5() {
    serial_print("Starting test 5: Loop timing for 1-5 magnets\n");

    GlobalState& instance = GlobalState::instance();

    const int magnet_ids[5] = {0, 1, 2, 3, 4};
    const float interval_s = 0.01f; // 10 ms
    int iterations = static_cast<int>(interval_s / instance.fastLoopTime);
    if (iterations <= 0) {
        iterations = 1;
    }

    for (int count = 1; count <= 5; ++count) {
        for (int i = 0; i < count; ++i) {
            int random_value = rand() % 256;
            instance.setControl(ControlOutputs(magnet_ids[i], random_value));
        }

        int64_t start_us = esp_timer_get_time();
        run_control_loop_for_seconds(instance, interval_s);
        int64_t end_us = esp_timer_get_time();

        float avg_us = static_cast<float>(end_us - start_us) / static_cast<float>(iterations);

        std::string msg = "Magnets: " + std::to_string(count) +
                          " | avg loop us: " + std::to_string(avg_us) + "\n";
        serial_print(msg.c_str());

        for (int i = 0; i < count; ++i) {
            instance.setControl(ControlOutputs::zero(magnet_ids[i]));
        }
    }
}


