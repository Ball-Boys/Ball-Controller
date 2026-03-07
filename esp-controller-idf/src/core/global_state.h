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


struct Orientation {
    float w;
    float x;
    float y;
    float z;

    Orientation(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
};

struct AngularVelocity {
    float x;
    float y;
    float z;

    AngularVelocity(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct ControlOutputs {
    int magnetId;
    float current_value;
    std::chrono::steady_clock::time_point timestamp;

    ControlOutputs() = delete;
    ControlOutputs(int magnetId, float current_value) : magnetId(magnetId), current_value(current_value), timestamp(std::chrono::steady_clock::now()) {}
    static ControlOutputs zero(int magnetId) {
        return ControlOutputs(magnetId, 0.0f);
    }
};

struct CurrentInfo {
    int magnetId;
    float current;
    std::chrono::steady_clock::time_point timestamp;

    CurrentInfo(int magnetId, float current) : magnetId(magnetId), current(current), timestamp(std::chrono::steady_clock::now()) {}
};

struct Vector3 {
    float x;
    float y;
    float z;

    constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct ADCAddress {
    const gpio_num_t adc_gpio_address;
    const int channel;

    constexpr ADCAddress(gpio_num_t adc_gpio_address, int channel) : adc_gpio_address(adc_gpio_address), channel(channel) {}
};

struct PWMAddress {
    const int driver_i2c_address;
    const int channel;

    constexpr PWMAddress(int driver_i2c_address, int channel) : driver_i2c_address(driver_i2c_address), channel(channel) {}

};

class MagnetInfo {
    
    private:
        std::vector<CurrentInfo> activeCurrentHistory;
        std::vector<ControlOutputs> controlHistory;
        std::vector<CurrentInfo> flushedCurrentHistory;

        int controlIntegral = 0;
        mutable SemaphoreHandle_t currentHistoryMutex;
        
    public:
        static constexpr size_t kMaxCurrentHistorySize = 100;  // Rolling buffer max size
        
        const int id;
        const Vector3 position;

        
        const float kp = 35.0f;
        const float ki = 15000.0f;
        const float dt; // 300 microseconds

        const ADCAddress adcAddress;
        const PWMAddress pwmAddress;

        MagnetInfo(int id, const Vector3& position, const float dt, const ADCAddress& adcAddress, const PWMAddress& pwmAddress) 
            : id(id), position(position), dt(dt), adcAddress(adcAddress), pwmAddress(pwmAddress) {
            currentHistoryMutex = xSemaphoreCreateMutex();
        }

        std::vector<CurrentInfo> getCurrentHistory() const {
            xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
            std::vector<CurrentInfo> copy = activeCurrentHistory;
            xSemaphoreGive(currentHistoryMutex);
            return copy;
        }



        const std::vector<ControlOutputs>& getControlHistory() const {
            return controlHistory;
        }

        void flushCurrentHistory() {
            xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
            // flushedCurrentHistory.insert(flushedCurrentHistory.end(), 
            //                              activeCurrentHistory.begin(), 
            //                              activeCurrentHistory.end());
            activeCurrentHistory.clear();
            // RESTING THE integral terms someitmes causes the PWM driver to cvhange i frequency (has not been observed since)
            // controlIntegral = 0;
            xSemaphoreGive(currentHistoryMutex);
        }

        const std::vector<CurrentInfo>& getFlushedCurrentHistory() const {
            return flushedCurrentHistory;
        }

        

        const std::vector<CurrentInfo>& getCurrentHistory(int last_n) const {
            static std::vector<CurrentInfo> subset;
            xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
            
            subset.clear();
            
            if (last_n <= 0) {
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

        const std::vector<ControlOutputs>& getControlHistory(int last_n) const {
            static std::vector<ControlOutputs> subset;
            subset.clear();
            
            if (last_n <= 0) {
                return subset;
            }
            
            int start_idx = std::max(0, static_cast<int>(controlHistory.size()) - last_n);
            subset.insert(subset.end(), 
                          controlHistory.begin() + start_idx, 
                          controlHistory.end());
            return subset;
        }

        void setCurrentValue(const CurrentInfo& value) {
            xSemaphoreTake(currentHistoryMutex, portMAX_DELAY);
            activeCurrentHistory.push_back(value);
            // Remove oldest entry if size exceeds max
            if (activeCurrentHistory.size() > kMaxCurrentHistorySize) {
                activeCurrentHistory.erase(activeCurrentHistory.begin());
            }
            xSemaphoreGive(currentHistoryMutex);
        }

        void setControlValue(const ControlOutputs& value) {
            controlHistory.push_back(value);
            flushCurrentHistory();
        }

        void zeroControl() {
            setControlValue(ControlOutputs::zero(id));
        }

        float getNextCurrentValuePI() {
            float error = controlHistory.back().current_value - activeCurrentHistory.back().current;
            float new_i = ki * error * dt;

            // Anti-windup: prevent integral from growing too large
            controlIntegral += new_i;
            if (controlIntegral > 4095.0f) controlIntegral = 4095.0f;
            if (controlIntegral < -4095.0f) controlIntegral = -4095.0f;
            
            float p_term = kp * error;
            float i_term = controlIntegral;
            float output = p_term + i_term;

            // Clamp output to valid PWM range
            if (output > 4095.0f) output = 4095.0f;
            if (output < 0.0f) output = 0.0f;
            
            return (int)(output);
        }


};

struct MagnetList {
    static constexpr std::size_t kMagnetCount = int(20);
    std::unordered_map<int, MagnetInfo> magnets;

    MagnetList() = default;
    MagnetList(std::unordered_map<int, MagnetInfo> mags) : magnets(mags) {}

    static MagnetList fromConfig(const std::array<std::tuple<int, Vector3, ADCAddress, PWMAddress>, kMagnetCount>& config, float dt) {
        std::unordered_map<int, MagnetInfo> magnets;
        for (const auto& item : config) {
            int id = std::get<0>(item);
            const Vector3& pos = std::get<1>(item);
            const ADCAddress& adcAddr = std::get<2>(item);
            const PWMAddress& pwmAddr = std::get<3>(item);
            if (id <= 0 || id > static_cast<int>(kMagnetCount)) {
                throw std::out_of_range("Magnet ID out of range in configuration");
            }
            magnets.emplace(id, MagnetInfo(id, pos, dt, adcAddr, pwmAddr));
        }
        return MagnetList{magnets};
    }

    MagnetInfo& getMagnetById(int id) {
        auto it = magnets.find(id);
        if (it == magnets.end()) {
            throw std::out_of_range("Magnet ID not found: " + std::to_string(id));
        }
        return it->second;
    }
    const MagnetInfo& getMagnetById(int id) const {
        auto it = magnets.find(id);
        if (it == magnets.end()) {
            throw std::out_of_range("Magnet ID not found: " + std::to_string(id));
        }
        return it->second;
    }
};
    


class GlobalState {
public:
    static GlobalState& instance();

    float fastLoopTime = 0.000650f; // 650 microseconds
    float slowLoopTime = 0.01f; // 10 milliseconds

    // functions get values about the orientation
    Orientation getOrientation() const;
    void setOrientation(const Orientation& value);
    void resetOrientation();
    const std::vector<Orientation>& getOrientationHistory() const;
    const std::vector<Orientation>& getOrientationHistory(int last_n) const;
    void setOrientationHistory(const std::vector<Orientation>& history);

    AngularVelocity getAngularVelocity() const;
    void setAngularVelocity(const AngularVelocity& value);
    void resetAngularVelocity();
    const std::vector<AngularVelocity>& getAngularVelocityHistory() const;
    const std::vector<AngularVelocity>& getAngularVelocityHistory(int last_n) const;
    void setAngularVelocityHistory(const std::vector<AngularVelocity>& history);

    
    // functions for getting and setting control outputs
    std::vector<ControlOutputs> getLatestControl() const;
    ControlOutputs getLatestControl(int magnetId) const;
    void setControl(const ControlOutputs& value);
    void setControl(const std::vector<ControlOutputs>& values);
    void zeroControl();

    // functions for getting and setting the offset
    Orientation getOffset() const;
    void setOffset(const Orientation& value);


    const std::vector<std::vector<CurrentInfo>>& getAllCurrentValues() const;
    const std::vector<CurrentInfo>& getCurrentValues(int magnetId) const;
    const std::vector<CurrentInfo>& getCurrentValues(int magnetId, int last_n) const;
    const std::vector<std::vector<CurrentInfo>>& getAllCurrentValues(int last_n) const;
    CurrentInfo getLatestCurrentValues(int magnetId) const;
    


    std::vector<CurrentInfo> currentControlLoop();

    // helper functions to collect magnet info
    PWMAddress getPWMAddress(int magnetId) const;
    ADCAddress getADCAddress(int magnetId) const;

    // getters and setters for the ideal direction 
    Vector3 getIdealDirection() const;
    void setIdealDirection(const Vector3& value);

    void kill();
    bool isKilled() const;

private:
    GlobalState(const std::array<std::tuple<int, Vector3, ADCAddress, PWMAddress>, 20>& config);
    GlobalState(const GlobalState&) = delete;
    GlobalState& operator=(const GlobalState&) = delete;

    MagnetList magnetList;
    Orientation offset;
    std::vector<Orientation> orientationHistory;
    std::vector<AngularVelocity> angularVelocityHistory;
    Vector3 idealDirection;
    std::vector<int> currentControlledMagnetIds;
    std::set<int> isMagnetRunning = {};

    // Timing instrumentation
    

    bool killed = false;
};


