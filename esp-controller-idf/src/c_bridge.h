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

void bc_bench_test_adc();
void bc_bench_test_adc_isolated();
void bc_bench_test_0();
void bc_bench_test_magnet_step();
void bc_bench_test_stress_20ms();
void bc_bench_test_quad_magnet_stress();

void bc_bench_test_0();
void bc_bench_test_1();
void bc_bench_test_2();
void bc_bench_test_3();
void bc_bench_test_4();
void bc_bench_test_5();

void bc_serial_print(const char* msg);
void bc_udp_sender_task(void* param);

void bc_run_state_machine_connection();
void bc_run_state_machine_testing();

#ifdef __cplusplus
} // extern "C"
#endif
