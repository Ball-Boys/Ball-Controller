#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int magnetId;
    float current;
} bc_current_info_t;

void bc_init_peripherals(int adc_clock_speed_hz, int uart_baud_rate);
void bc_core_1_loop();

void bc_bench_test_1();
void bc_bench_test_2();
void bc_bench_test_3();
void bc_bench_test_4();
void bc_bench_test_5();

#ifdef __cplusplus
} // extern "C"
#endif
