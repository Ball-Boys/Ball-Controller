#pragma once
#include <array>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <unordered_map>

#define MAGNET_INFO 

struct Orientation {
    const float w;
    const float x;
    const float y;
    const float z;

    Orientation(float w, float x, float y, float z) : w(w), x(x), y(y), z(z) {}
};

struct ControlOutputs {
    const int magnetId;
    const float dutyCycle;
    const std::chrono::steady_clock::time_point timestamp;

    ControlOutputs(int magnetId, float dutyCycle) : magnetId(magnetId), dutyCycle(dutyCycle), timestamp(std::chrono::steady_clock::now()) {}

    static ControlOutputsOutputs(int magnetId) {
        return ControlOutputs(magnetId, 0.0f);
    }
};

struct CurrentInfo {
    const int magnetId;
    const float current;
    const std::chrono::steady_clock::time_point timestamp;

    CurrentInfo(int magnetId, float current) : magnetId(magnetId), current(current), timestamp(std::chrono::steady_clock::now()) {}
};

struct Vector3 {
    const float x;
    const float y;
    const float z;

    constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct ADCAddress {
    const int adc_number;
    const int channel;

    constexpr ADCAddress(int adc_number, int channel) : adc_number(adc_number), channel(channel) {}
};

struct PWMAddress {
    const int driver_number;
    const int channel;

    constexpr PWMAddress(int driver_number, int channel) : driver_number(driver_number), channel(channel) {}

};

class MagnetInfo {
    
    private:
        std::vector<CurrentInfo> currentHistory;
        std::vector<ControlOutputs> controlHistory;
    public:
        const int id;
        const Vector3 position;

        const ADCAddress adcAddress;
        const PWMAddress pwmAddress;

        MagnetInfo(int id, const Vector3& position, const ADCAddress& adcAddress, const PWMAddress& pwmAddress) 
            : id(id), position(position), adcAddress(adcAddress), pwmAddress(pwmAddress) {}

        const getCurrentHistory() const {
            return currentHistory;
        }

        const getControlHistory() const {
            return controlHistory;
        }

        void setCurrentValue(const CurrentInfo& value) {
            currentHistory.push_back(value);
        }

        void setControlValue(const ControlOutputs& value) {
            controlHistory.push_back(value);
        }

        void zeroControl() {
            controlHistory.push_back(ControlOutputs::zero(id));
        }
};

struct MagnetList {
    static constexpr std::size_t kMagnetCount = int(20);
    std::unordered_map<int, MagnetInfo> magnets;

    MagnetList() = default;
    MagnetList(std::unordered_map<int, MagnetInfo> mags) : magnets(mags) {}

    static MagnetList fromConfig(const std::array<std::tuple<int, Vector3&, ADCAddress&, PWMAddress&>, kMagnetCount>& config) {
        std::unordered_map<int, MagnetInfo> magnets;
        for (const auto& item : config) {
            int id = std::get<0>(item);
            const Vector3& pos = std::get<1>(item);
            const ADCAddress& adcAddr = std::get<2>(item);
            const PWMAddress& pwmAddr = std::get<3>(item);
            if (id < 0 || id >= static_cast<int>(kMagnetCount)) {
                throw std::out_of_range("Magnet ID out of range in configuration");
            }
            magnets.emplace(id, MagnetInfo(id, pos, adcAddr, pwmAddr));
        }
        return MagnetList{magnets};
    }

    const MagnetInfo& getMagnetById(int id) const {
        auto it = magnets.find(id);
        if (it == magnets.end()) {
            throw std::out_of_range("Magnet ID not found");
        }
        return it->second;
    }
};
    


class GlobalState {
public:
    static GlobalState& instance();

    // functions get values about the orientation
    Orientation getOrientation() const;
    void setOrientation(const Orientation& value);
    void resetOrientation();
    const std::vector<Orientation>& getOrientationHistory() const;
    const std::vector<Orientation>& getOrientationHistory(int last_n) const;
    void setOrientationHistory(const std::vector<Orientation>& history);
    
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
    void setCurrentValue(const CurrentInfo& value);
    

    // helper functions to collect magnet info
    PWMAddress getPWMAddress(int magnetId) const;
    ADCAddress getADCAddress(int magnetId) const;

    // getters and setters for the ideal direction 
    Vector3 getIdealDirection() const;
    void setIdealDirection(const Vector3& value);




private:
    GlobalState();
    GlobalState(const GlobalState&) = delete;
    GlobalState& operator=(const GlobalState&) = delete;


    Orientation offset;
    std::vector<Orientation> orientationHistory;
    MagnetList magnetList;
    Vector3 idealDirection;
};


