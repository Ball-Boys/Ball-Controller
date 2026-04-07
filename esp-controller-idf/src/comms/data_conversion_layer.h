#include <comms/wifi_client.h>
#include <stdint.h>

typedef struct __attribute__((packed)) {
    int32_t timestamp;
    uint8_t system_state;                 // Global state enum
    uint8_t reserved[3];                  // padding to align float array boundary
    float orientation_wxyz[4];           // rotation quaternion
    float angular_velocity_xyz[3];        // measured angular velocity
    float magnet_setpoints[20];           // controller setpoints for 20 magnets
    float magnet_current_values[20]; 
    int32_t magnet_current_timestep[20];

} ball_data_packet;

void extract_data_from_globals(ball_data_packet* out_packet);
