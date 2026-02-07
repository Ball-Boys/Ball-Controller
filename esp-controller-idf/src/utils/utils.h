#include <vector>


std::vector<int> retreveCurrentValueFromADC(std::vector<int> mag_ids);

void setPWMOutputs(std::vector<int> magnetId, std::vector<int> value);

inline void serial_print(const char* msg);

