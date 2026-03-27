#include "global_state.h"
#include "magnet_config.h"

#include "utils/utils.h"
#include <esp_timer.h>
#include <freertos/mpu_wrappers.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

GlobalState &GlobalState::instance()
{
    static GlobalState singleton(MAGNET_CONFIG, LOCAL_OFFSET);
    return singleton;
}

GlobalState::GlobalState(const std::array<std::tuple<int, Vector3, ADCAddress, PWMAddress>, 20> &config, Orientation local_offset)
    : magnetList(MagnetList::fromConfig(config, fastLoopTime, local_offset)),
      offset(1.0f, 0.0f, 0.0f, 0.0f),
      // TODO: Make it so it defaults to 1.0, 0.0, 0.0
      idealDirection(1.0f, 0.0f, 0.0f)
{
    orientationHistory.reserve(kMaxOrientationHistorySize);
    angularVelocityHistory.reserve(kMaxAngularVelocityHistorySize);
    killedMutex = xSemaphoreCreateMutex();
    stateMutex = xSemaphoreCreateMutex();
    initLUT();
    
}

// ============= Orientation methods =============

Orientation GlobalState::getOrientation() const
{
    // Gets the latest orientation from the list
    if (orientationHistory.empty())
    {
        return Orientation(1.0f, 0.0f, 0.0f, 0.0f); // Default orientation if none available
    }
    return orientationHistory.back();
}

void GlobalState::setOrientation(const Orientation &value)
{
    orientationHistory.push_back(value);
    if (orientationHistory.size() > kMaxOrientationHistorySize)
    {
        orientationHistory.erase(orientationHistory.begin());
    }
}

void GlobalState::resetOrientation()
{
    orientationHistory.clear();
}

const std::vector<Orientation> &GlobalState::getOrientationHistory() const
{
    return orientationHistory;
}

const std::vector<Orientation> &GlobalState::getOrientationHistory(int last_n) const
{
    static std::vector<Orientation> subset;
    subset.clear();

    if (last_n <= 0)
    {
        return subset;
    }

    int start_idx = std::max(0, static_cast<int>(orientationHistory.size()) - last_n);
    subset.insert(subset.end(),
                  orientationHistory.begin() + start_idx,
                  orientationHistory.end());
    return subset;
}

AngularVelocity GlobalState::getAngularVelocity() const
{
    // Gets the latest angular velocity from the list
    if (angularVelocityHistory.empty())
    {
        return AngularVelocity(0.0f, 0.0f, 0.0f); // Default angular velocity if none available
    }
    return angularVelocityHistory.back();
}

void GlobalState::setAngularVelocity(const AngularVelocity &value)
{
    angularVelocityHistory.push_back(value);
    if (angularVelocityHistory.size() > kMaxAngularVelocityHistorySize)
    {
        angularVelocityHistory.erase(angularVelocityHistory.begin());
    }
}

void GlobalState::resetAngularVelocity()
{
    angularVelocityHistory.clear();
}

const std::vector<AngularVelocity> &GlobalState::getAngularVelocityHistory() const
{
    return angularVelocityHistory;
}

const std::vector<AngularVelocity> &GlobalState::getAngularVelocityHistory(int last_n) const
{
    static std::vector<AngularVelocity> subset;
    subset.clear();

    if (last_n <= 0)
    {
        return subset;
    }

    int start_idx = std::max(0, static_cast<int>(angularVelocityHistory.size()) - last_n);
    subset.insert(subset.end(),
                  angularVelocityHistory.begin() + start_idx,
                  angularVelocityHistory.end());
    return subset;
}

void GlobalState::setAngularVelocityHistory(const std::vector<AngularVelocity> &history)
{
    angularVelocityHistory = history;
}

void GlobalState::setOrientationHistory(const std::vector<Orientation> &history)
{
    orientationHistory = history;
}

// ============= Control Output methods =============

std::vector<ControlOutputs> GlobalState::getLatestControl() const
{
    std::vector<ControlOutputs> latest;
    // Reserve once to avoid repeated allocations in the fast control loop
    latest.reserve(magnetList.magnets.size());
    for (const auto &pair : magnetList.magnets)
    {
        const auto &controlHistory = pair.second.getControlHistory();
        if (!controlHistory.empty())
        {
            const auto &latestControl = controlHistory.back();
            if (latestControl.current_value != 0.0f)
            {
                latest.emplace_back(latestControl);
            }
            else
            {
                latest.emplace_back(ControlOutputs::zero(pair.first));
            }
        }
        else
        {
            // If no control history, we can consider it as zero control
            latest.emplace_back(ControlOutputs::zero(pair.first));
        }
    }
    return latest;
}

ControlOutputs GlobalState::getLatestControl(int magnetId) const
{
    const auto &magnet = magnetList.getMagnetById(magnetId);
    if (magnet.getControlHistory().empty())
    {
        throw std::runtime_error("No control outputs yet for this magnet");
    }
    return magnet.getControlHistory().back();
}

void GlobalState::setControl(const ControlOutputs &value)
{
    auto it = magnetList.magnets.find(value.magnetId);
    if (it == magnetList.magnets.end())
    {
        throw std::out_of_range("Magnet ID not found: " + std::to_string(value.magnetId));
    }
    it->second.setControlValue(value);
}

void GlobalState::setControl(const std::vector<ControlOutputs> &values)
{
    for (const auto &ctrl : values)
    {
        setControl(ctrl);
    }
}

void GlobalState::zeroControl()
{
    for (auto &pair : magnetList.magnets)
    {
        pair.second.zeroControl();
    }
}

// ============= Offset methods =============

Orientation GlobalState::getOffset() const
{
    return offset;
}

void GlobalState::setOffset(const Orientation &value)
{
    offset = value;
}

// ============= Current Values methods =============

const std::vector<std::vector<CurrentInfo>> &GlobalState::getAllCurrentValues() const
{
    static std::vector<std::vector<CurrentInfo>> allValues;
    allValues.clear();

    for (const auto &pair : magnetList.magnets)
    {
        allValues.push_back(pair.second.getCurrentHistory());
    }
    return allValues;
}

const std::vector<CurrentInfo> &GlobalState::getCurrentValues(int magnetId) const
{
    static std::vector<CurrentInfo> cached;
    const auto &magnet = magnetList.getMagnetById(magnetId);
    cached = magnet.getCurrentHistory();
    return cached;
}

const std::vector<CurrentInfo> &GlobalState::getCurrentValues(int magnetId, int last_n) const
{
    const auto &magnet = magnetList.getMagnetById(magnetId);
    static std::vector<CurrentInfo> subset;

    std::vector<CurrentInfo> history = magnet.getCurrentHistory();
    subset.clear();

    if (last_n <= 0)
    {
        return subset;
    }

    int start_idx = std::max(0, static_cast<int>(history.size()) - last_n);
    subset.insert(subset.end(),
                  history.begin() + start_idx,
                  history.end());
    return subset;
}

const std::vector<std::vector<CurrentInfo>> &GlobalState::getAllCurrentValues(int last_n) const
{
    static std::vector<std::vector<CurrentInfo>> subset;
    subset.clear();

    if (last_n <= 0)
    {
        return subset;
    }

    for (const auto &pair : magnetList.magnets)
    {
        const auto &history = pair.second.getCurrentHistory(last_n);
        int start_idx = std::max(0, static_cast<int>(history.size()) - last_n);
        std::vector<CurrentInfo> magnetSubset(
            history.begin() + start_idx,
            history.end());
        subset.push_back(magnetSubset);
    }
    return subset;
}

CurrentInfo GlobalState::getLatestCurrentValues(int magnetId) const
{
    const auto &magnet = magnetList.getMagnetById(magnetId);
    std::vector<CurrentInfo> history = magnet.getCurrentHistory();
    if (history.empty())
    {
        throw std::runtime_error("No current values yet for this magnet");
    }
    return history.back();
}

std::vector<CurrentInfo> GlobalState::currentControlLoop()
{
    int64_t loop_start = esp_timer_get_time();

    std::vector<ControlOutputs> latestControls = getLatestControl();
    std::vector<int> mag_ids;
    std::vector<int> magnets_to_zero = mag_ids;

    for (const auto &control : latestControls)
    {
        if (control.current_value != 0.0f)
        {
            mag_ids.push_back(control.magnetId);
            isMagnetRunning.insert(control.magnetId);
        }
        else
        {
            if (isMagnetRunning.count(control.magnetId) > 0)
            {
                magnets_to_zero.push_back(control.magnetId);
                isMagnetRunning.erase(control.magnetId);
            }
        }
    }

    setPWMOutputs(magnets_to_zero, std::vector<int>(magnets_to_zero.size(), 0));

    currentControlledMagnetIds = mag_ids;

    std::vector<CurrentInfo> currentInfos;

    // Method 2 to make our controller happier (do things 1 by 1)
    for (size_t i = 0; i < mag_ids.size(); ++i)
    {

        int magnetId = mag_ids[i];
        std::vector<float> currentValues = retreveCurrentValueFromADC({magnetId});
        float currentValue = currentValues[0]; // Assuming single value per magnet
        CurrentInfo currentInfo(magnetId, currentValue);
        currentInfos.push_back(currentInfo);
        magnetList.getMagnetById(magnetId).setCurrentValue(currentInfo);

        int newPWMSignal = magnetList.getMagnetById(magnetId).getNextCurrentValuePI();
        setPWMOutputs({magnetId}, {newPWMSignal});
    }

    // Read ADC values
    // std::vector<float> currents = retreveCurrentValueFromADC(mag_ids);

    // // printf("[ADC] Current values read from ADC | Time: %lld µs\n", (adc_end - adc_start));

    // std::vector<int> newPWMSignals;

    // // Process each magnet and calculate control signals
    // for (size_t i = 0; i < mag_ids.size(); ++i) {
    //     int magnetId = mag_ids[i];
    //     float currentValue = currents[i];
    //     CurrentInfo currentInfo(magnetId, currentValue);
    //     currentInfos.push_back(currentInfo);
    //     magnetList.getMagnetById(magnetId).setCurrentValue(currentInfo);

    //     int newPWMSignal = magnetList.getMagnetById(magnetId).getNextCurrentValuePI();
    //     newPWMSignals.push_back(newPWMSignal);
    // }

    // setPWMOutputs(mag_ids, newPWMSignals);

    int64_t loop_end = esp_timer_get_time();
    int64_t total_time = (loop_end - loop_start);

    return currentInfos;
}

// ============= Magnet Info helper methods =============

PWMAddress GlobalState::getPWMAddress(int magnetId) const
{
    return magnetList.getMagnetById(magnetId).pwmAddress;
}

ADCAddress GlobalState::getADCAddress(int magnetId) const
{
    return magnetList.getMagnetById(magnetId).adcAddress;
}

Vector3 GlobalState::getMagnetPosition(int magnetId) const
{
    return magnetList.getMagnetById(magnetId).position;
}

// ============= Ideal Direction methods =============

Vector3 GlobalState::getIdealDirection() const
{
    return idealDirection;
}

void GlobalState::setIdealDirection(const Vector3 &value)
{
    // TODO: remiplement this so we can actually set the ideal direction.
    // idealDirection = value;
}

void GlobalState::set_kill(bool value)
{
    xSemaphoreTake(this->killedMutex, portMAX_DELAY);
    this->killed = value;
    xSemaphoreGive(this->killedMutex);
}

bool GlobalState::isKilled() const
{
    xSemaphoreTake(this->killedMutex, portMAX_DELAY);
    bool isKilled = this->killed;
    xSemaphoreGive(this->killedMutex);
    return isKilled;
}

// ============= State Management methods =============

GlobalState::SystemState GlobalState::getSystemState() const
{
    if (stateMutex == NULL)
    {
        return systemState;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    GlobalState::SystemState state = systemState;
    xSemaphoreGive(stateMutex);
    return state;
}

void GlobalState::setSystemState(GlobalState::SystemState state)
{
    if (stateMutex == NULL)
    {
        systemState = state;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    systemState = state;
    xSemaphoreGive(stateMutex);
}

bool GlobalState::getCalibrationRequested() const
{
    if (stateMutex == NULL)
    {
        return calibrationRequested;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    bool requested = calibrationRequested;
    xSemaphoreGive(stateMutex);
    return requested;
}

void GlobalState::requestCalibration()
{
    if (stateMutex == NULL)
    {
        calibrationRequested = true;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    calibrationRequested = true;
    xSemaphoreGive(stateMutex);
}

void GlobalState::clearCalibrationRequest()
{
    if (stateMutex == NULL)
    {
        calibrationRequested = false;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    calibrationRequested = false;
    xSemaphoreGive(stateMutex);
}

bool GlobalState::getStartRequested() const
{
    if (stateMutex == NULL)
    {
        return startRequested;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    bool requested = startRequested;
    xSemaphoreGive(stateMutex);
    return requested;
}

void GlobalState::requestStart()
{
    if (stateMutex == NULL)
    {
        startRequested = true;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    startRequested = true;
    xSemaphoreGive(stateMutex);
}

void GlobalState::clearStartRequest()
{
    if (stateMutex == NULL)
    {
        startRequested = false;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    startRequested = false;
    xSemaphoreGive(stateMutex);
}

// ============= Calibration Input methods =============

bool GlobalState::getCalibrationInputAvailable() const
{
    if (stateMutex == NULL)
    {
        return calibrationInputAvailable;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    bool available = calibrationInputAvailable;
    xSemaphoreGive(stateMutex);
    return available;
}

Vector3 GlobalState::getCalibrationInput() const
{
    if (stateMutex == NULL)
    {
        return calibrationInput;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    Vector3 input = calibrationInput;
    xSemaphoreGive(stateMutex);
    return input;
}

void GlobalState::setCalibrationInput(const Vector3 &direction)
{
    if (stateMutex == NULL)
    {
        calibrationInput = direction;
        calibrationInputAvailable = true;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    calibrationInput = direction;
    calibrationInputAvailable = true;
    xSemaphoreGive(stateMutex);
}

void GlobalState::clearCalibrationInput()
{
    if (stateMutex == NULL)
    {
        calibrationInputAvailable = false;
        return;
    }
    xSemaphoreTake(stateMutex, portMAX_DELAY);
    calibrationInputAvailable = false;
    xSemaphoreGive(stateMutex);
}



void GlobalState::initLUT()
{
    float max_angle = 1.570796f; // pi/2

    for (int i = 0; i < LUT_SIZE; i++)
    {
        float angle = ((float)i / (LUT_SIZE - 1)) * max_angle;

        if (angle < 0.1f)
        {
            torque_lut[i] = 0.0f;
        }
        else
        {
            float num = 60.0f * sinf(angle);
            float den = 0.01f + 1.0f - cosf(angle);
            float val = (num / den) * expf(-2.5f * angle);
            float val_per_amp = val / 8.0f;
            torque_lut[i] = (val_per_amp > 0.0f) ? val_per_amp : 0.0f;
        }
    }
}


Matrix3 GlobalState::quatToMatrix(const Orientation &q)
{
    Matrix3 mat;
    float xx = q.x * q.x;
    float yy = q.y * q.y;
    float zz = q.z * q.z;
    float xy = q.x * q.y;
    float xz = q.x * q.z;
    float yz = q.y * q.z;
    float wx = q.w * q.x;
    float wy = q.w * q.y;
    float wz = q.w * q.z;

    mat.m[0][0] = 1.0f - 2.0f * (yy + zz);
    mat.m[0][1] = 2.0f * (xy - wz);
    mat.m[0][2] = 2.0f * (xz + wy);

    mat.m[1][0] = 2.0f * (xy + wz);
    mat.m[1][1] = 1.0f - 2.0f * (xx + zz);
    mat.m[1][2] = 2.0f * (yz - wx);

    mat.m[2][0] = 2.0f * (xz - wy);
    mat.m[2][1] = 2.0f * (yz + wx);
    mat.m[2][2] = 1.0f - 2.0f * (xx + yy);
    return mat;
}

float GlobalState::getTorqueFactor(float angle_rad)
{
    float max_angle = 1.570796f; // pi/2

    if (angle_rad >= max_angle)
        return 0.0f;
    if (angle_rad < 0.1f)
        return 0.0f;

    // 1. Map the angle to a floating-point index (0 to 127)
    float index_float = (angle_rad / max_angle) * (LUT_SIZE - 1);

    // 2. Get the integer index below the target
    int idx = (int)index_float;

    // Safety bound
    if (idx >= LUT_SIZE - 1)
        return torque_lut[LUT_SIZE - 1];

    // 3. Linear Interpolation (mix the two closest array values)
    float fraction = index_float - idx;
    float val1 = torque_lut[idx];
    float val2 = torque_lut[idx + 1];

    return val1 * (1.0f - fraction) + val2 * fraction;
}


struct Candidate
{
    int id;
    Vector3 vec;
};

std::vector<ControlOutputs> GlobalState::solve(float joy_x, float joy_y, const Orientation &q)
{
    std::vector<ControlOutputs> output;
    // 1. Apply Yaw Offset to Joystick Input
    float target_mag = sqrtf(joy_x * joy_x + joy_y * joy_y);
    if (target_mag < 0.001f)
        return output; // Deadzone

    float theta_joy = atan2f(joy_y, joy_x);
    float theta_imu = theta_joy + yaw_offset; // Apply calibration

    Vector3 desired_force_world(cosf(theta_imu) * target_mag, sinf(theta_imu) * target_mag, 0.0f);

    // 2. Coordinate Transforms
    
    Vector3 target_force_body = desired_force_world;
    Vector3 gravity_ball = Vector3(0, 0, -1.0f);

    // 3. Find Candidates
    Candidate candidates[20];
    int num_candidates = 0;

    for (int i = 1; i <= 20; i++)
    {
        Vector3 mag_pos = getMagnetPosition(i).transform(q).normalized();
        float dot_g = mag_pos.dot(gravity_ball);
        float dot_clamp = fmaxf(-1.0f, fminf(1.0f, dot_g));
        float angle = acosf(dot_clamp);

        float strength = getTorqueFactor(angle);

        if (i == 1 || i == 6)
        {
            printf("Magnet %d | Pos: (%f, %f, %f) | DotG: %f | Angle: %f | Strength: %f\n", i, mag_pos.x, mag_pos.y, mag_pos.z, dot_g, angle, strength);
        }
        if (strength <= 0.00001f)
            continue;

        Vector3 proj_component = mag_pos - (gravity_ball * dot_g);
        if (proj_component.norm() < 0.01f)
            continue;

        Vector3 force_dir = proj_component.normalized();
        Vector3 force_vec = force_dir * strength;

        candidates[num_candidates].id = i;
        candidates[num_candidates].vec = force_vec;
        num_candidates++;
    }


    // 4. Optimization Search
    float min_score = 1e9f; // Infinity
    int best_count = 0;

    // A. Single Magnets
    for (int i = 0; i < num_candidates; i++)
    {
        float mag_len_sq = candidates[i].vec.dot(candidates[i].vec);
        float projection = candidates[i].vec.dot(target_force_body);
        if (projection <= 0)
            continue;

        float current = projection / mag_len_sq;
        if (current > max_current)
            current = max_current;

        Vector3 produced = candidates[i].vec * current;
        float error_dist = (target_force_body - produced).norm();
        float score = error_dist + (current_penalty * current);

        if (score < min_score)
        {
            min_score = score;
            output.clear();
            output.push_back(ControlOutputs(candidates[i].id, current));
            best_count = 1;
        }
    }



    // B. Pairs
    float t_len = target_force_body.norm();
    if (t_len > 0.001f)
    {
        Vector3 t_hat = target_force_body / t_len;
        Vector3 ref = (fabsf(t_hat.z) < 0.9f) ? Vector3(0, 0, 1) : Vector3(0, 1, 0);
        Vector3 y_hat = ref.cross(t_hat).normalized();

        for (int i = 0; i < num_candidates; i++)
        {
            for (int j = i + 1; j < num_candidates; j++)
            {
                float Ax = candidates[i].vec.dot(t_hat);
                float Ay = candidates[i].vec.dot(y_hat);
                float Bx = candidates[j].vec.dot(t_hat);
                float By = candidates[j].vec.dot(y_hat);

                float det = (Ax * By) - (Ay * Bx);
                if (fabsf(det) < 0.0001f)
                    continue;

                float Ia = (t_len * By) / det;
                float Ib = -(t_len * Ay) / det;

                if (Ia >= 0 && Ib >= 0)
                {
                    if (Ia > max_current)
                        Ia = max_current;
                    if (Ib > max_current)
                        Ib = max_current;

                    Vector3 produced = (candidates[i].vec * Ia) + (candidates[j].vec * Ib);
                    float error_dist = (target_force_body - produced).norm();
                    float score = error_dist + (current_penalty * (Ia + Ib));

                    if (score < min_score - 0.0001f)
                    {
                        min_score = score;
                    
                        output.clear();
                        output.push_back(ControlOutputs(candidates[i].id, Ia));
                        output.push_back(ControlOutputs(candidates[j].id, Ib));
                        best_count = 2;
                    }
                }
            }
        }
    }

    // printf("Best Score: %f | Count: %d\n", min_score, best_count);
    for (const auto &ctrl : output)
    {
        printf("Output Magnet %d | Current: %f\n", ctrl.magnetId, ctrl.current_value);
    }

    return output;
}

// ---------------------------------------------------------
// CALIBRATION LOGIC
// ---------------------------------------------------------

int GlobalState::getCalibrationMagnet(const Orientation &q)
{
    Matrix3 R = quatToMatrix(q);
    Vector3 gravity_ball = R.multiplyTranspose(Vector3(0, 0, -1.0f));

    int best_id = -1;
    float max_strength = -1.0f;

    for (int i = 1; i <= 20; i++)
    {
        float dot_g = getMagnetPosition(i).dot(gravity_ball);
        float dot_clamp = fmaxf(-1.0f, fminf(1.0f, dot_g));
        float angle = acosf(dot_clamp);

        // Find the magnet that has the absolute highest pull strength
        float strength = getTorqueFactor(angle);
        if (strength > max_strength)
        {
            max_strength = strength;
            best_id = i;
        }
    }
    return best_id;
}

void GlobalState::finishCalibration(int fired_magnet_id, const Orientation &q, float joy_x, float joy_y)
{
    if (fired_magnet_id <= 0 || fired_magnet_id > 20)
        return;

    Matrix3 R = quatToMatrix(q);
    // ASK LEVI, I feel like assuming this could be problematic.
    Vector3 gravity_ball = R.multiplyTranspose(Vector3(0, 0, -1.0f));

    // 1. Calculate the theoretical force vector of the fired magnet
    float dot_g = getMagnetPosition(fired_magnet_id).dot(gravity_ball);
    Vector3 proj_component = getMagnetPosition(fired_magnet_id) - (gravity_ball * dot_g);
    Vector3 force_dir_body = proj_component.normalized();

    // Convert to IMU World Frame
    Vector3 force_dir_world = R.multiply(force_dir_body);

    // 2. Determine IMU angle
    float theta_imu = atan2f(force_dir_world.y, force_dir_world.x);

    // 3. Determine User Joystick angle
    float theta_joy = atan2f(joy_y, joy_x);

    // 4. Store Offset
    yaw_offset = theta_imu - theta_joy;
    is_calibrated = true;
}
