#include "c_bridge.h"

#include "core/global_state.h"
#include "core/peripherals.h"
#include "scripts/bench_test.h"
#include "utils/utils.h"

void bc_init_peripherals(int adc_clock_speed_hz, int uart_baud_rate) {
    init_peripherals(adc_clock_speed_hz, uart_baud_rate);
}

void bc_core_1_loop() {
    while (true) {
        // slow loop on top
    }
}

void bc_bench_test_adc() {
    test_adc();
    vTaskDelete(NULL);
}


void bc_bench_test_adc_isolated() {
    test_adc_isolated();
    vTaskDelete(NULL);
}

void bc_bench_test_0() {
    test_0();
    vTaskDelete(NULL);
}

void bc_bench_test_1() {
    test_1();
    vTaskDelete(NULL);
}

void bc_bench_test_magnet_step() {
    test_magnet_step();
    vTaskDelete(NULL);
}

void bc_bench_test_stress_20ms() {
    test_stress_20ms();
    vTaskDelete(NULL);
}
void bc_bench_test_quad_magnet_stress() {
    test_quad_magnet_stress();
    vTaskDelete(NULL);
}

void bc_bench_test_2() {
    test_2();
    vTaskDelete(NULL);
}

void bc_bench_test_3() {
    test_3();
    vTaskDelete(NULL);
}

void bc_bench_test_4() {
    test_4();
    vTaskDelete(NULL);
}

void bc_bench_test_5() {
    test_5();
}

void bc_serial_print(const char* msg) {
    serial_print(msg);
}





