#include <vector>


std::vector<float> retreveCurrentValueFromADC(std::vector<int> mag_ids);

void setPWMOutputs(std::vector<int> magnetId, std::vector<int> value);
void zero_pwm_outputs();

void readIMU();

void serial_print(const char* msg);
void serial_printf(const char* fmt, ...);

