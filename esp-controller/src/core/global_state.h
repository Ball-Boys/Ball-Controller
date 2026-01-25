#pragma once

struct Orientation {
    float w{1.0f};
    float x{0.0f};
    float y{0.0f};
    float z{0.0f};
};

struct ControlOutputs {
    int magnetId{0};
    float dutyCycle{0.0f};
};

class GlobalState {
public:
    static GlobalState& instance();

    Orientation getOrientation() const;
    void setOrientation(const Orientation& value);

    ControlOutputs getControl() const;
    void setControl(const ControlOutputs& value);

private:
    GlobalState() = default;
    GlobalState(const GlobalState&) = delete;
    GlobalState& operator=(const GlobalState&) = delete;

    Orientation orientation{};
    ControlOutputs control{};
};
