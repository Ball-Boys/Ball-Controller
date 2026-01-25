#include "global_state.h"

GlobalState& GlobalState::instance() {
    static GlobalState singleton;
    return singleton;
}

Orientation GlobalState::getOrientation() const {
    return orientation;
}

void GlobalState::setOrientation(const Orientation& value) {
    orientation = value;
}

ControlOutputs GlobalState::getControl() const {
    return control;
}

void GlobalState::setControl(const ControlOutputs& value) {
    control = value;
}
