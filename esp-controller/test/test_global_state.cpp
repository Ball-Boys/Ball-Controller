#include <cassert>
#include <iostream>
#include <stdexcept>
#include "../src/core/global_state.h"
#include "../src/core/magnet_config.h"

namespace {

// Helper to check that a callable throws a specific exception type.
template <typename Ex, typename Fn>
void expect_throw(Fn&& fn, const char* msg) {
    bool threw = false;
    try {
        fn();
    } catch (const Ex&) {
        threw = true;
    } catch (...) {
        std::cerr << "Unexpected exception type in test: " << msg << "\n";
        assert(false);
    }
    if (!threw) {
        std::cerr << "Expected exception not thrown: " << msg << "\n";
        assert(false);
    }
}

void test_orientation() {
    auto& gs = GlobalState::instance();
    gs.resetOrientation();

    Orientation o1(0.0f, 1.0f, 0.0f, 0.0f);
    gs.setOrientation(o1);
    auto got = gs.getOrientation();
    assert(got.x == 1.0f);
    assert(got.y == 0.0f);
    assert(got.z == 0.0f);

    const auto& hist = gs.getOrientationHistory();
    assert(!hist.empty());
    assert(hist.back().x == 1.0f);

    const auto& last1 = gs.getOrientationHistory(1);
    assert(last1.size() == 1);
    assert(last1.back().x == 1.0f);
}

void test_offset() {
    auto& gs = GlobalState::instance();
    Orientation off(0.0f, 0.0f, 1.0f, 0.0f);
    gs.setOffset(off);
    auto got = gs.getOffset();
    assert(got.z == 1.0f);
}

void test_ideal_direction() {
    auto& gs = GlobalState::instance();
    Vector3 dir(0.0f, 1.0f, 0.0f);
    gs.setIdealDirection(dir);
    auto got = gs.getIdealDirection();
    assert(got.y == 1.0f);
}

void test_control_outputs() {
    auto& gs = GlobalState::instance();

    // Happy path: set and get latest per magnet
    ControlOutputs c0(0, 0.5f);
    ControlOutputs c1(1, 0.7f);
    gs.setControl(c0);
    gs.setControl(c1);

    auto latest0 = gs.getLatestControl(0);
    assert(latest0.magnetId == 0);
    assert(latest0.dutyCycle == 0.5f);

    auto latestAll = gs.getLatestControl();
    assert(latestAll.size() >= 2);

    // Error: out of range magnet id
    expect_throw<std::out_of_range>([&]() { gs.getLatestControl(-1); }, "getLatestControl out of range");
}

void test_control_zeroing() {
    auto& gs = GlobalState::instance();
    gs.zeroControl();
    auto latestAll = gs.getLatestControl();
    // After zeroing, latestAll should be empty (no history)
    assert(latestAll.empty());
    expect_throw<std::runtime_error>([&]() { gs.getLatestControl(0); }, "latest control empty");
}

void test_current_values() {
    auto& gs = GlobalState::instance();
    CurrentInfo cur0(0, 1.1f);
    CurrentInfo cur1(0, 1.2f);
    gs.setCurrentValue(cur0);
    gs.setCurrentValue(cur1);

    auto latest = gs.getLatestCurrentValues(0);
    assert(latest.current == 1.2f);

    auto slice = gs.getCurrentValues(0, 1);
    assert(slice.size() == 1);
    assert(slice.back().current == 1.2f);

    // Error: out of range magnet id
    expect_throw<std::out_of_range>([&]() { gs.getCurrentValues(-1); }, "getCurrentValues out of range");
}

void test_current_zero_and_empty() {
    auto& gs = GlobalState::instance();
    // No API to clear currents; ensure requesting latest on an empty magnet throws
    // Use a magnet id unlikely to be used above
    int unusedId = 5;
    expect_throw<std::runtime_error>([&]() { gs.getLatestCurrentValues(unusedId); }, "latest current empty");
}

void test_magnet_addresses() {
    auto& gs = GlobalState::instance();
    // We only assert that calls succeed without throwing; actual values depend on config
    auto pwm0 = gs.getPWMAddress(0);
    auto adc0 = gs.getADCAddress(0);
    (void)pwm0;
    (void)adc0;
    expect_throw<std::out_of_range>([&]() { gs.getPWMAddress(-1); }, "pwm address out of range");
}

} // namespace

int main() {
    test_orientation();
    test_offset();
    test_ideal_direction();
    test_control_outputs();
    test_control_zeroing();
    test_current_values();
    test_current_zero_and_empty();
    test_magnet_addresses();

    std::cout << "All GlobalState tests passed.\n";
    return 0;
}
