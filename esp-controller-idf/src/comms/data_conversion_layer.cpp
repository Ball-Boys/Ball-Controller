#include <comms/data_conversion_layer.h>
#include <core/global_state.h>

#include <algorithm>
#include <cstring>


void extract_data_from_globals(ball_data_packet* out_packet) {

    if (out_packet == nullptr) {
        return;
    }

    std::memset(out_packet, 0, sizeof(*out_packet));
    out_packet->timestamp = static_cast<int32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()
        ).count()
    );

    GlobalState& global_state = GlobalState::instance();

    // System state (0..4)
    out_packet->system_state = static_cast<uint8_t>(global_state.getSystemState());

    // Orientation quaternion
    Orientation current_orientation = global_state.getOrientation();
    out_packet->orientation_wxyz[0] = current_orientation.w;
    out_packet->orientation_wxyz[1] = current_orientation.x;
    out_packet->orientation_wxyz[2] = current_orientation.y;
    out_packet->orientation_wxyz[3] = current_orientation.z;

    // Angular velocity
    AngularVelocity current_ang_vel = global_state.getAngularVelocity();
    out_packet->angular_velocity_xyz[0] = current_ang_vel.x;
    out_packet->angular_velocity_xyz[1] = current_ang_vel.y;
    out_packet->angular_velocity_xyz[2] = current_ang_vel.z;

    // Magnet setpoints from latest control outputs (indexed by magnet ID)
    auto latest_controls = global_state.getLatestControl();
    for (const auto& ctrl : latest_controls) {
        int idx = ctrl.magnetId - 1;  // magnet IDs are 1-based
        if (idx >= 0 && idx < 20) {
            out_packet->magnet_setpoints[idx] = ctrl.current_value;
        }
    }

    // Current measured values
    for (int magnet_index = 0; magnet_index < 20; magnet_index++) {
        const int magnet_id = magnet_index + 1;
        const std::vector<CurrentInfo>& current_values = global_state.getCurrentValues(magnet_id);
        const int sample_count = static_cast<int>(std::min<size_t>(100, current_values.size()));
        const int start_idx = static_cast<int>(current_values.size()) - sample_count;

        for (int i = 0; i < sample_count; i++) {
            const CurrentInfo& info = current_values[start_idx + i];
            out_packet->magnet_current_values[magnet_index][i] = info.current;
            out_packet->magnet_current_timestep[magnet_index][i] = static_cast<int32_t>(
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    info.timestamp.time_since_epoch()
                ).count()
            );
        }
    }
}


