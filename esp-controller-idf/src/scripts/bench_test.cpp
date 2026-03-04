#include "bench_test.h"

#include <core/peripherals.h>

// in test 1 we will sweep through turning each magnet on one by one for 1 second each.
#include "core/global_state.h"
#include "utils/utils.h"

#include "esp_timer.h"

#include <cstdint>
#include <string>
#include <esp_rom_sys.h>
#include <cmath>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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
    return;
}
} // namespace

void test_adc_isolated() {
    printf("Starting isolated ADC test\n");
    while (true) {
        std::vector<uint16_t> values;
        for (gpio_num_t num : {GPIO_NUM_27, GPIO_NUM_32, GPIO_NUM_33}) {
            for (int channel = 0; channel < 8; ++channel) {
                uint16_t value = adc1283_read(num, channel);

                values.push_back(value);
            }
        }

        printf("Completed ADC read cycle. Values: ");
        for (const auto& value : values) {
            printf("%d ", value);
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay 1 second
        
    }
}


void test_adc() {
    serial_print("Starting ADC test\n");

    while (true) {
        
        std::vector<float> value = retreveCurrentValueFromADC(
            {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20}
            
        );
        printf("Retrieved current values from ADC: ");
        int i = 1;
        for (const float val : value) {
            printf("%d: %.3f A ", i, val);
            i++;
        }
        printf("\n");
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay 1 second
    }

}



void test_0() {
    serial_print("Starting test 0: Basic control loop timing\n");

    GlobalState& instance = GlobalState::instance();

    int num_deltas = 0;
    float delta = 0.000003f;  // Time step in seconds
    float w = 2.0f * 3.14159f * 1000;  // 2*pi for sine wave, freq controlled by delay
    while (true) {
        // set pwm to current value of sin wave (0-255 range)
        int pwm_value = static_cast<int>((std::sin(num_deltas * delta * w) + 1.0f) * 2048.5f);
        // int pwm_value = 2555;
        setPWMOutputs({1}, {pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value});
        printf("Setting value: %d (sin arg: %f)\n", pwm_value, num_deltas * delta * w);
        vTaskDelay(pdMS_TO_TICKS(4));  // Delay ~0.3ms via FreeRTOS
        ++num_deltas; 
    }

}

void test_1() {

    serial_print("Starting test 1: Magnet sweep\n");


    GlobalState& instance = GlobalState::instance();

    float loop_iterations = 1.0f / instance.fastLoopTime; // Set loop iterations based on fast loop time

    int loops = 0;
    // loop though index 0 through 19 magents
    while (true) {
        for (int mag_id = 1; mag_id <= 1; ++mag_id) {
            
            instance.setControl(ControlOutputs(mag_id, 5)); // Set magnet to mid power
            run_control_loop_for_seconds(instance, 2.0f);
            instance.setControl(ControlOutputs(mag_id, 0)); // Set magnet to 0 power
            run_control_loop_for_seconds(instance, 0.5f);

        }
    }
    serial_print("Test 1 complete: Magnet sweep\n");
}

void test_stress_20ms() {
    serial_print("STARTING 20ms STRESS TEST (50Hz Update Rate)\n");
    GlobalState& instance = GlobalState::instance();

    instance.setControl(ControlOutputs(1, 5)); // Set magnet to mid power
    run_control_loop_for_seconds(instance, 3.0f); // Run for 20 seconds

    instance.setControl(ControlOutputs(1, 4)); // Set magnet to 0 power
    instance.setControl(ControlOutputs(2, 5)); // Set magnet to mid power
    run_control_loop_for_seconds(instance, 1.0f); // Run for 20

    instance.setControl(ControlOutputs(2, 4)); // Set magnet to 0 power
    instance.setControl(ControlOutputs(3, 5)); // Set magnet to mid power
    instance.setControl(ControlOutputs(1, 5)); // Set magnet to mid power
    instance.setControl(ControlOutputs(4, 0)); // Set magnet to mid power
    run_control_loop_for_seconds(instance, 1.0f); // Run for 20

    instance.setControl(ControlOutputs(3, 4)); // Set magnet to 0 power
    instance.setControl(ControlOutputs(4, 5)); // Set magnet to mid power
    instance.setControl(ControlOutputs(1, 4)); // Set magnet to mid power
    instance.setControl(ControlOutputs(2, 5)); // Set magnet to mid power
    run_control_loop_for_seconds(instance, 1.0f); // Run for 20

    instance.setControl(ControlOutputs(4, 4)); // Set magnet to 0 power
    instance.setControl(ControlOutputs(1, 0)); // Set magnet to mid power
    instance.setControl(ControlOutputs(2, 5)); // Set magnet to mid power
    instance.setControl(ControlOutputs(3, 5)); // Set magnet to mid power

    run_control_loop_for_seconds(instance, 1.0f); // Run for 20

    instance.setControl(ControlOutputs(1, 0)); // Set magnet to mid power
    instance.setControl(ControlOutputs(3, 1)); // Set magnet to mid power
    instance.setControl(ControlOutputs(2, 0)); // Set magnet to mid power
    instance.setControl(ControlOutputs(4, 0)); // Set magnet to mid power
    run_control_loop_for_seconds(instance, 1.0f); // Run for 20
}

void test_quad_magnet_stress() {
    serial_print("INITIATING 4-MAGNET CROSSFIRE (Indices 1-4)\n");
    GlobalState& instance = GlobalState::instance();

    const float STEP_TIME = 0.020f; // 20ms heartbeat
    const int TOTAL_STEPS = 1000;   // 20 seconds of high-speed switching
    
    for (int step = 0; step < TOTAL_STEPS; ++step) {
        
        /* LOGIC: Binary Ripple (4-bit counter)
           Step % 2  -> Magnet 1 (Toggle every 20ms)
           Step % 4  -> Magnet 2 (Toggle every 40ms)
           Step % 8  -> Magnet 3 (Toggle every 80ms)
           Step % 16 -> Magnet 4 (Toggle every 160ms)
        */
        
        int m1 = (step % 2 < 1) ? 10 : 0;
        int m2 = (step % 4 < 2) ? 10 : 0;
        int m3 = (step % 8 < 4) ? 10 : 0;
        int m4 = (step % 16 < 8) ? 10 : 0;

        // Apply all controls in a single block
        instance.setControl(ControlOutputs(1, m1));
        instance.setControl(ControlOutputs(2, m2));
        instance.setControl(ControlOutputs(3, m3));
        instance.setControl(ControlOutputs(4, m4));

        /* PEAK LOAD STRESS: 
           Every 3 seconds (150 steps), force ALL magnets to 100% 
           power for 100ms (5 cycles) to test for thermal shutdown.
        */
        if (step % 150 >= 0 && step % 150 < 5) {
            for (int i = 1; i <= 4; ++i) {
                instance.setControl(ControlOutputs(i, 10));
            }
            
        }

        run_control_loop_for_seconds(instance, STEP_TIME);
    }

    // Safety: Emergency Clear
    for (int i = 1; i <= 4; ++i) {
        instance.setControl(ControlOutputs(i, 0));
    }
    serial_print("4-Magnet Stress Test Complete.\n");
}

void test_magnet_step() {
    GlobalState& instance = GlobalState::instance();

    float loop_iterations = 1.0f / instance.fastLoopTime; // Set loop iterations based on fast loop time

    int loops = 0;
    // loop though index 0 through 19 magents
    while (true) {
        for (int mag_id = 1; mag_id <= 1; ++mag_id) {
            for (int i = 0; i < 10; ++i) {
                instance.setControl(ControlOutputs(mag_id, i)); // Set magnet to random power
                run_control_loop_for_seconds(instance, 0.1f);
            }
            for (int i = 10; i >= 0; --i) {
                instance.setControl(ControlOutputs(mag_id, i)); // Set magnet to random power
                run_control_loop_for_seconds(instance, 0.1f);
            }
        }
    }
}



// in test 2 we will do a very similar process to the one above but we will turn on 2 magnets at a time.

void test_2() {
    

    GlobalState& instance = GlobalState::instance();

    float loop_iterations = 1.0f / instance.fastLoopTime; // Set loop iterations based on fast loop time

    int loops = 0;
    // loop though index 0 through 19 magents
    while (true) {
    for (int mag_id = 1; mag_id <= 4; mag_id += 2) {
        printf("Activating magnets %d and %d\n", mag_id, mag_id + 1);
        
        
        instance.setControl(ControlOutputs(mag_id, 5)); // Set magnet to max power
        instance.setControl(ControlOutputs(mag_id + 1, 5)); // Set next magnet to max power

        run_control_loop_for_seconds(instance, 2.0f);
        instance.setControl(ControlOutputs(mag_id, 0)); // Set magnet to 0 power
        instance.setControl(ControlOutputs(mag_id + 1, 0)); // Set next magnet to 0 power
        run_control_loop_for_seconds(instance, 0.5f);
    }
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


