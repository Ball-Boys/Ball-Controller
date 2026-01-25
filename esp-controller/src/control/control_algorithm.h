#pragma once

#include "../core/global_state.h"

ControlOutputs computeControl(const Orientation& current, const float targetDirection[3]);
