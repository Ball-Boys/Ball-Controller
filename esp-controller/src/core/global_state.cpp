#include "global_state.h"
#include "magnet_config.h"

GlobalState& GlobalState::instance() {
    static GlobalState singleton(MAGNET_CONFIG);
    return singleton;
}

GlobalState::GlobalState(const std::array<std::tuple<int, Vector3, ADCAddress, PWMAddress>, 20>& config) 
    : magnetList(MagnetList::fromConfig(config)),
      offset(1.0f, 0.0f, 0.0f, 0.0f),
      idealDirection(0.0f, 0.0f, 0.0f) {
    orientationHistory.reserve(1000);
}

// ============= Orientation methods =============

Orientation GlobalState::getOrientation() const {
    // Gets the latest orientation from the list
    if (orientationHistory.empty()) {
        throw std::runtime_error("No orientation data available");
    }
    return orientationHistory.back();
}

void GlobalState::setOrientation(const Orientation& value) {
    orientationHistory.push_back(value);
}

void GlobalState::resetOrientation() {
    orientationHistory.clear();
}


const std::vector<Orientation>& GlobalState::getOrientationHistory() const {
    return orientationHistory;
}

const std::vector<Orientation>& GlobalState::getOrientationHistory(int last_n) const {
    static std::vector<Orientation> subset;
    subset.clear();
    
    if (last_n <= 0) {
        return subset;
    }
    
    int start_idx = std::max(0, static_cast<int>(orientationHistory.size()) - last_n);
    subset.insert(subset.end(), 
                  orientationHistory.begin() + start_idx, 
                  orientationHistory.end());
    return subset;
}

AngularVelocity GlobalState::getAngularVelocity() const {
    // Gets the latest angular velocity from the list
    if (angularVelocityHistory.empty()) {
        throw std::runtime_error("No angular velocity data available");
    }
    return angularVelocityHistory.back();
}

void GlobalState::setAngularVelocity(const AngularVelocity& value) {
    angularVelocityHistory.push_back(value);
}

void GlobalState::resetAngularVelocity() {
    angularVelocityHistory.clear();
}

const std::vector<AngularVelocity>& GlobalState::getAngularVelocityHistory() const {
    return angularVelocityHistory;
}

const std::vector<AngularVelocity>& GlobalState::getAngularVelocityHistory(int last_n) const {
    static std::vector<AngularVelocity> subset;
    subset.clear();
    
    if (last_n <= 0) {
        return subset;
    }
    
    int start_idx = std::max(0, static_cast<int>(angularVelocityHistory.size()) - last_n);
    subset.insert(subset.end(), 
                  angularVelocityHistory.begin() + start_idx, 
                  angularVelocityHistory.end());
    return subset;
}

void GlobalState::setAngularVelocityHistory(const std::vector<AngularVelocity>& history) {
    angularVelocityHistory = history;
}

void GlobalState::setOrientationHistory(const std::vector<Orientation>& history) {
    orientationHistory = history;
}

// ============= Control Output methods =============

std::vector<ControlOutputs> GlobalState::getLatestControl() const {
    std::vector<ControlOutputs> latest;
    for (const auto& pair : magnetList.magnets) {
        const auto& controlHistory = pair.second.getControlHistory();
        if (!controlHistory.empty()) {
            const auto& latestControl = controlHistory.back();
            if (latestControl.current_value != 0.0f) {
                latest.push_back(latestControl);
            }
        }
    }
    return latest;
}

ControlOutputs GlobalState::getLatestControl(int magnetId) const {
    const auto& magnet = magnetList.getMagnetById(magnetId);
    if (magnet.getControlHistory().empty()) {
        throw std::runtime_error("No control outputs yet for this magnet");
    }
    return magnet.getControlHistory().back();
}

void GlobalState::setControl(const ControlOutputs& value) {
    auto it = magnetList.magnets.find(value.magnetId);
    if (it == magnetList.magnets.end()) {
        throw std::out_of_range("Magnet ID not found: " + std::to_string(value.magnetId));
    }
    it->second.setControlValue(value);
}

void GlobalState::setControl(const std::vector<ControlOutputs>& values) {
    for (const auto& ctrl : values) {
        setControl(ctrl);
    }
}

void GlobalState::zeroControl() {
    for (auto& pair : magnetList.magnets) {
        pair.second.zeroControl();
    }
}

// ============= Offset methods =============

Orientation GlobalState::getOffset() const {
    return offset;
}

void GlobalState::setOffset(const Orientation& value) {
    offset = value;
}

// ============= Current Values methods =============

const std::vector<std::vector<CurrentInfo>>& GlobalState::getAllCurrentValues() const {
    static std::vector<std::vector<CurrentInfo>> allValues;
    allValues.clear();
    
    for (const auto& pair : magnetList.magnets) {
        allValues.push_back(pair.second.getCurrentHistory());
    }
    return allValues;
}

const std::vector<CurrentInfo>& GlobalState::getCurrentValues(int magnetId) const {
    const auto& magnet = magnetList.getMagnetById(magnetId);
    return magnet.getCurrentHistory();
}

const std::vector<CurrentInfo>& GlobalState::getCurrentValues(int magnetId, int last_n) const {
    const auto& magnet = magnetList.getMagnetById(magnetId);
    static std::vector<CurrentInfo> subset;
    subset.clear();
    
    if (last_n <= 0) {
        return subset;
    }
    
    int start_idx = std::max(0, static_cast<int>(magnet.getCurrentHistory().size()) - last_n);
    subset.insert(subset.end(), 
                  magnet.getCurrentHistory().begin() + start_idx, 
                  magnet.getCurrentHistory().end());
    return subset;
}

const std::vector<std::vector<CurrentInfo>>& GlobalState::getAllCurrentValues(int last_n) const {
    static std::vector<std::vector<CurrentInfo>> subset;
    subset.clear();
    
    if (last_n <= 0) {
        return subset;
    }
    
    for (const auto& pair : magnetList.magnets) {
        const auto& history = pair.second.getCurrentHistory(last_n);
        int start_idx = std::max(0, static_cast<int>(history.size()) - last_n);
        std::vector<CurrentInfo> magnetSubset(
            history.begin() + start_idx,
            history.end()
        );
        subset.push_back(magnetSubset);
    }
    return subset;
}

CurrentInfo GlobalState::getLatestCurrentValues(int magnetId) const {
    const auto& magnet = magnetList.getMagnetById(magnetId);
    if (magnet.getCurrentHistory().empty()) {
        throw std::runtime_error("No current values yet for this magnet");
    }
    return magnet.getCurrentHistory().back();
}

void GlobalState::setCurrentValue(const CurrentInfo& value) {
    auto it = magnetList.magnets.find(value.magnetId);
    if (it == magnetList.magnets.end()) {
        throw std::out_of_range("Magnet ID not found: " + std::to_string(value.magnetId));
    }
    it->second.setCurrentValue(value);
}



// ============= Magnet Info helper methods =============

PWMAddress GlobalState::getPWMAddress(int magnetId) const {
    return magnetList.getMagnetById(magnetId).pwmAddress;
}

ADCAddress GlobalState::getADCAddress(int magnetId) const {
    return magnetList.getMagnetById(magnetId).adcAddress;
}

// ============= Ideal Direction methods =============

Vector3 GlobalState::getIdealDirection() const {
    return idealDirection;
}

void GlobalState::setIdealDirection(const Vector3& value) {
    idealDirection = value;
}

void GlobalState::kill() {
    killed = true;
}

bool GlobalState::isKilled() const {
    return killed;
}
