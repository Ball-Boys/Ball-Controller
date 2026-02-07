#include "c_bridge.h"

#include "core/global_state.h"
#include "core/peripherals.h"
#include "scripts/bench_test.cpp"

void bc_init_peripherals(int adc_clock_speed_hz) {
    init_peripherals(adc_clock_speed_hz);
}

void bc_core_1_loop() {
    while (true) {
        // slow loop on top
    }
}

void bc_bench_test_1() {
    test_1();
}

void bc_bench_test_2() {
    test_2();
}

void bc_bench_test_3() {
    test_3();
}

void bc_bench_test_4() {
    test_4();
}

void bc_bench_test_5() {
    test_5();
}





