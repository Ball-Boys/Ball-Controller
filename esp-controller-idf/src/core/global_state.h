#pragma once
#include <array>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <unordered_map>
#include <driver/gpio.h>
#include <set>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cmath>
#include "../control/BallController.h"

struct Orientation
{
    float w;
    float x;
    float y;
    float z;

    Orientation(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}

    Orientation conjugate() const {
        return {w, -x, -y, -z};
    }

    // 2. The Squared Norm (||q||^2)
    float normSquared() const {
        return w*w + x*x + y*y + z*z;
    }

    // Quaternion Multiplication (Hamilton Product)
    Orientation operator*(const Orientation& q) const {
        return {
            w * q.w - x * q.x - y * q.y - z * q.z,
            w * q.x + x * q.w + y * q.z - z * q.y,
            w * q.y - x * q.z + y * q.w + z * q.x,
            w * q.z + x * q.y - y * q.x + z * q.w
        };
    }

    // 3. The Inverse (q^-1 = q* / ||q||^2)
    Orientation inverse() const {
        float n2 = normSquared();
        if (n2 == 0) return {0, 0, 0, 0}; // Avoid division by zero
        Orientation conj = conjugate();
        return {conj.w / n2, conj.x / n2, conj.y / n2, conj.z / n2};
    }
};

struct AngularVelocity
{
    float x;
    float y;
    float z;

    AngularVelocity(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct ControlOutputs
{
    int magnetId;
    float current_value;
    std::chrono::steady_clock::time_point timestamp;

    ControlOutputs() = delete;
    ControlOutputs(int magnetId, float current_value) : magnetId(magnetId), current_value(current_value), timestamp(std::chrono::steady_clock::now()) {}
    static ControlOutputs zero(int magnetId)
    {
        return ControlOutputs(magnetId, 0.0f);
    }
};

struct CurrentInfo
{
    int magnetId;
    float current;
    std::chrono::steady_clock::time_point timestamp;

    CurrentInfo(int magnetId, float current) : magnetId(magnetId), current(current), timestamp(std::chrono::steady_clock::now()) {}
};

struct Vector3
{
    float x;
    float y;
    float z;

    constexpr Vector3(float x = 0, float y = 0, float z = 0) : x(x), y(y), z(z) {}

    Vector3 operator+(const Vector3 &v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
    Vector3 operator-(const Vector3 &v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
    Vector3 operator*(float s) const { return Vector3(x * s, y * s, z * s); }
    Vector3 operator/(float s) const { return Vector3(x / s, y / s, z / s); }

    float dot(const Vector3 &v) const { return x * v.x + y * v.y + z * v.z; }
    Vector3 cross(const Vector3 &v) const
    {
        return Vector3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
    }
    float norm() const { return sqrtf(x * x + y * y + z * z); }
    Vector3 normalized() const
    {
        float n = norm();
        return (n > 0.0001f) ? (*this / n) : Vector3(0, 0, 0);
    }

    Vector3 transform(Orientation q) const {
        Orientation qprime = q.conjugate();
        Orientation v_quat(0, x, y, z);
        Orientation rotated = q * v_quat * qprime;
        return Vector3(rotated.x, rotated.y, rotated.z);
    }
};

struct ADCAddress
{
    const gpio_num_t adc_gpio_address;
    const int channel;

    constexpr ADCAddress(gpio_num_t adc_gpio_address, int channel) : adc_gpio_address(adc_gpio_address), channel(channel) {}
};

struct PWMAddress
{
    const int driver_i2c_address;
    const int channel;

    constexpr PWMAddress(int driver_i2c_address, int channel) : driver_i2c_address(driver_i2c_address), channel(channel) {}
};

class MagnetInfo
{

private:
    std::vector<CurrentInfo> activeCurrentHistory;
    std::vector<ControlOutputs> controlHistory;
    std::vector<CurrentInfo> flushedCurrentHistory;

    int controlIntegral = 0;
    mutable SemaphoreHandle_t currentHistoryMutex;

public:
    static constexpr size_t kMaxCurrentHistorySize = 100;         // Rolling buffer max size
    static constexpr size_t kMaxControlHistorySize = 30;         // Rolling buffer max size
    static constexpr size_t kMaxOrientationHistorySize = 50;     // Rolling buffer max size
    static constexpr size_t kMaxAngularVelocityHistorySize = 50; // Rolling buffer max size

    const int id;
    const Vector3 position;

    const float kp = 35.0f;
    const float ki = 15000.0f;
    const float dt; // 300 microseconds

    const ADCAddress adcAddress;
    const PWMAddress pwmAddress;

    MagnetInfo(int id, const Vector3 &position, const float dt, const ADCAddress &adcAddress, const PWMAddress &pwmAddress)
        : id(id), position(position), dt(dt), adcAddress(adcAddress), pwmAddress(pwmAddress)
    {
        currentHistoryMutex = xSemaphoreCreateMutex();
    }

    std::vector<CurrentInfo> getCurrentHistory() const
    {
        xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
        std::vector<CurrentInfo> copy = activeCurrentHistory;
        xSemaphoreGive(currentHistoryMutex);
        return copy;
    }

    const std::vector<ControlOutputs> &getControlHistory() const
    {
        return controlHistory;
    }

    void flushCurrentHistory()
    {
        xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
        // flushedCurrentHistory.insert(flushedCurrentHistory.end(),
        //                              activeCurrentHistory.begin(),
        //                              activeCurrentHistory.end());
        activeCurrentHistory.clear();
        // RESTING THE integral terms someitmes causes the PWM driver to cvhange i frequency (has not been observed since)
        // controlIntegral = 0;
        xSemaphoreGive(currentHistoryMutex);
    }

    const std::vector<CurrentInfo> &getFlushedCurrentHistory() const
    {
        return flushedCurrentHistory;
    }

    const std::vector<CurrentInfo> &getCurrentHistory(int last_n) const
    {
        static std::vector<CurrentInfo> subset;
        xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);

        subset.clear();

        if (last_n <= 0)
        {
            xSemaphoreGive(currentHistoryMutex);
            return subset;
        }

        int start_idx = std::max(0, static_cast<int>(activeCurrentHistory.size()) - last_n);
        subset.insert(subset.end(),
                      activeCurrentHistory.begin() + start_idx,
                      activeCurrentHistory.end());

        xSemaphoreGive(currentHistoryMutex);
        return subset;
    }

    const std::vector<ControlOutputs> &getControlHistory(int last_n) const
    {
        static std::vector<ControlOutputs> subset;
        subset.clear();

        if (last_n <= 0)
        {
            return subset;
        }

        int start_idx = std::max(0, static_cast<int>(controlHistory.size()) - last_n);
        subset.insert(subset.end(),
                      controlHistory.begin() + start_idx,
                      controlHistory.end());
        return subset;
    }

    void setCurrentValue(const CurrentInfo &value)
    {
        xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
        activeCurrentHistory.push_back(value);
        // Remove oldest entry if size exceeds max
        if (activeCurrentHistory.size() > kMaxCurrentHistorySize)
        {
            activeCurrentHistory.erase(activeCurrentHistory.begin());
        }
        xSemaphoreGive(currentHistoryMutex);
    }

    void setControlValue(const ControlOutputs &value)
    {
        controlHistory.push_back(value);
        // Remove oldest entry if size exceeds max
        if (controlHistory.size() > kMaxControlHistorySize)
        {
            controlHistory.erase(controlHistory.begin());
        }
        flushCurrentHistory();
    }

    void zeroControl()
    {
        setControlValue(ControlOutputs::zero(id));
    }

    float getNextCurrentValuePI()
    {
        float error = controlHistory.back().current_value - activeCurrentHistory.back().current;
        float new_i = ki * error * dt;

        // Anti-windup: prevent integral from growing too large
        controlIntegral += new_i;
        if (controlIntegral > 4095.0f)
            controlIntegral = 4095.0f;
        if (controlIntegral < -4095.0f)
            controlIntegral = -4095.0f;

        float p_term = kp * error;
        float i_term = controlIntegral;
        float output = p_term + i_term;

        // Clamp output to valid PWM range
        if (output > 4095.0f)
            output = 4095.0f;
        if (output < 0.0f)
            output = 0.0f;

        return (int)(output);
    }
};

struct MagnetList
{
    static constexpr std::size_t kMagnetCount = int(20);
    std::unordered_map<int, MagnetInfo> magnets;

    MagnetList() = default;
    MagnetList(std::unordered_map<int, MagnetInfo> mags) : magnets(mags) {}

    static MagnetList fromConfig(const std::array<std::tuple<int, Vector3, ADCAddress, PWMAddress>, kMagnetCount> &config, float dt, Orientation local_offset)
    {
        std::unordered_map<int, MagnetInfo> magnets;
        printf("Loading magnet configuration...\n");
        for (const auto &item : config)
        {
            int id = std::get<0>(item);
            const Vector3 &pos = std::get<1>(item);
            // TODELETE
            printf("Magnet %d position: (%f, %f, %f)\n", id, pos.x, pos.y, pos.z);
            const ADCAddress &adcAddr = std::get<2>(item);
            const PWMAddress &pwmAddr = std::get<3>(item);
            if (id <= 0 || id > static_cast<int>(kMagnetCount))
            {
                throw std::out_of_range("Magnet ID out of range in configuration");
            }
            Vector3 transformed_pos = pos.transform(local_offset);
            // TODELETE
            printf("Magnet %d transformed position: (%f, %f, %f)\n", id, transformed_pos.x, transformed_pos.y, transformed_pos.z);
            magnets.emplace(id, MagnetInfo(id, transformed_pos, dt, adcAddr, pwmAddr));
        }
        return MagnetList{magnets};
    }

    MagnetInfo &getMagnetById(int id)
    {
        auto it = magnets.find(id);
        if (it == magnets.end())
        {
            throw std::out_of_range("Magnet ID not found: " + std::to_string(id));
        }
        return it->second;
    }
    const MagnetInfo &getMagnetById(int id) const
    {
        auto it = magnets.find(id);
        if (it == magnets.end())
        {
            throw std::out_of_range("Magnet ID not found: " + std::to_string(id));
        }
        return it->second;
    }
};


struct Matrix3
{
    float m[3][3];
    Vector3 multiply(const Vector3 &v) const
    {
        return Vector3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z);
    }
    // Multiply by Transpose (Inverse for rotation matrices)
    Vector3 multiplyTranspose(const Vector3 &v) const
    {
        return Vector3(
            m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
            m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
            m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z);
    }
};

class GlobalState
{
public:
    // System state enumeration
    enum class SystemState
    {
        CONNECTION,  // Establishing WiFi connection
        STANDBY,     // Waiting for start command
        CALIBRATION, // Running calibration
        RUNNING,     // Normal operation
        TESTING      // Testing mode
    };

    static GlobalState &instance();
    static constexpr size_t kMaxOrientationHistorySize = 500;
    static constexpr size_t kMaxAngularVelocityHistorySize = 500;

    float fastLoopTime = 0.000650f; // 650 microseconds
    float slowLoopTime = 0.01f;     // 10 milliseconds

    // functions get values about the orientation
    Orientation getOrientation() const;
    void setOrientation(const Orientation &value);
    void resetOrientation();
    const std::vector<Orientation> &getOrientationHistory() const;
    const std::vector<Orientation> &getOrientationHistory(int last_n) const;
    void setOrientationHistory(const std::vector<Orientation> &history);

    AngularVelocity getAngularVelocity() const;
    void setAngularVelocity(const AngularVelocity &value);
    void resetAngularVelocity();
    const std::vector<AngularVelocity> &getAngularVelocityHistory() const;
    const std::vector<AngularVelocity> &getAngularVelocityHistory(int last_n) const;
    void setAngularVelocityHistory(const std::vector<AngularVelocity> &history);

    // functions for getting and setting control outputs
    std::vector<ControlOutputs> getLatestControl() const;
    ControlOutputs getLatestControl(int magnetId) const;
    void setControl(const ControlOutputs &value);
    void setControl(const std::vector<ControlOutputs> &values);
    void zeroControl();

    // functions for getting and setting the offset
    Orientation getOffset() const;
    void setOffset(const Orientation &value);

    const std::vector<std::vector<CurrentInfo>> &getAllCurrentValues() const;
    const std::vector<CurrentInfo> &getCurrentValues(int magnetId) const;
    const std::vector<CurrentInfo> &getCurrentValues(int magnetId, int last_n) const;
    const std::vector<std::vector<CurrentInfo>> &getAllCurrentValues(int last_n) const;
    CurrentInfo getLatestCurrentValues(int magnetId) const;

    std::vector<CurrentInfo> currentControlLoop();

    // helper functions to collect magnet info
    PWMAddress getPWMAddress(int magnetId) const;
    ADCAddress getADCAddress(int magnetId) const;
    Vector3 getMagnetPosition(int magnetId) const;

    // getters and setters for the ideal direction
    Vector3 getIdealDirection() const;
    void setIdealDirection(const Vector3 &value);

    void set_kill(bool value);
    bool isKilled() const;

    // State management
    SystemState getSystemState() const;
    void setSystemState(SystemState state);

    // WiFi command flags
    bool getCalibrationRequested() const;
    void requestCalibration();
    void clearCalibrationRequest();

    bool getStartRequested() const;
    void requestStart();
    void clearStartRequest();

    // Calibration input (joystick direction from dashboard during calibration)
    bool getCalibrationInputAvailable() const;
    Vector3 getCalibrationInput() const;
    void setCalibrationInput(const Vector3 &direction);
    void clearCalibrationInput();


    // 1. Regular Operation
    // Takes user joystick vector (x, y) and IMU quaternion. Returns number of active magnets.
    std::vector<ControlOutputs> solve(float joy_x, float joy_y, const Orientation &q);

    // 2. Calibration Phase Methods
    // Step A: Find best magnet to fire, returns its ID.
    int getCalibrationMagnet(const Orientation &q);

    // Step B: Call this after user pushes joystick in response to the fired magnet
    void finishCalibration(int fired_magnet_id, const Orientation &q, float joy_x, float joy_y);

    bool isCalibrated() const { return is_calibrated; };

    

private:

    GlobalState(const std::array<std::tuple<int, Vector3, ADCAddress, PWMAddress>, 20> &config, Orientation local_offset);
    GlobalState(const GlobalState &) = delete;
    GlobalState &operator=(const GlobalState &) = delete;

    MagnetList magnetList;
    Orientation offset;
    std::vector<Orientation> orientationHistory;
    std::vector<AngularVelocity> angularVelocityHistory;
    Vector3 idealDirection = Vector3(1.0f, 0.0f, 0.0f);
    std::vector<int> currentControlledMagnetIds;
    std::set<int> isMagnetRunning = {};

    // Timing instrumentation

    mutable SemaphoreHandle_t killedMutex;
    bool killed = false;

    // State management
    mutable SemaphoreHandle_t stateMutex;
    SystemState systemState = SystemState::CONNECTION;
    bool calibrationRequested = false;
    bool startRequested = false;

    // Calibration input from dashboard
    Orientation localAxisOffset = Orientation(0, 0, 0.7071, 0.7071);
    Vector3 calibrationInput = Vector3(0, 0, 0);
    bool calibrationInputAvailable = false;


    float max_current = 8.0f;
    float current_penalty = 1.0f;

    // Calibration State
    float yaw_offset = 0.0f;
    bool is_calibrated = false;

    static const int LUT_SIZE = 128;
    float torque_lut[LUT_SIZE];
    void initLUT();

    void initMagnets();
    Matrix3 quatToMatrix(const Orientation &q);
    float getTorqueFactor(float angle_rad);
};
