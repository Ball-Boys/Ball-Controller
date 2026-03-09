#include <comms/wifi_client.h>
#include <stdint.h>

typedef struct __attribute__((packed)) {
    int32_t timestamp;
    float magnet_current_values[20][300]; 
    int32_t magnet_current_timestep[20][300];

} ball_data_packet;

void extract_data_from_globals(ball_data_packet* out_packet);
