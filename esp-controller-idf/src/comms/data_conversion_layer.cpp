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

    for (int magnet_index = 0; magnet_index < 20; magnet_index++) {
        const int magnet_id = magnet_index + 1;
        const std::vector<CurrentInfo>& current_values = global_state.getCurrentValues(magnet_id);
        const int sample_count = static_cast<int>(std::min<size_t>(300, current_values.size()));
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

