// What to write to one finger this tick, from its command and what it was last written.
// Pure, so it can be tested without a hand: main.cpp turns the answer into SDK calls.
#pragma once

#include <cstdint>

namespace drive {

// Normalized POSITION (0-1000) at or above which a finger grips at constant current. It sits
// below real grips: in the pose catalogue every genuine grip is 0.9-0.98.
constexpr uint16_t kGripThreshold = 800;
// Normalized current (0-1000) a gripping finger is written once, on entry.
constexpr int16_t kGripCurrent = 500;
// MotorCmd_.mode asking for the finger's drive off (PWM 0): teleop sends it for a finger that
// has reached its pose's target, and the FSM relays it. 1 is claimed for a whole-hand rest.
constexpr uint8_t kOffMode = 2;
// MotorState_.mode published for a finger held off. 1-3 are left for the grip states.
constexpr uint8_t kParkedState = 4;

enum class Write { kPosition, kCurrent, kPwmOff, kNone };

struct Finger {
    bool in_current = false;
    bool off = false;
};

// Decide this tick's write for one finger and update its memory. thumb_aux has no mechanical
// hold and nothing to press against, so it is positioned whatever its command says.
inline Write choose(Finger& f, uint16_t position, uint8_t mode, bool thumb_aux) {
    if (mode == kOffMode && !thumb_aux) {
        // Once: PWM 0 draws nothing and the gear train holds the finger where it is.
        const bool enter = !f.off;
        f.off = true;
        f.in_current = false;  // a later grip re-sends its current
        return enter ? Write::kPwmOff : Write::kNone;
    }
    f.off = false;
    if (position >= kGripThreshold && !thumb_aux) {
        // Only on entry: each current write dips that finger's current for a few
        // milliseconds, so rewriting it every tick makes a held grip buzz.
        const bool enter = !f.in_current;
        f.in_current = true;
        return enter ? Write::kCurrent : Write::kNone;
    }
    f.in_current = false;
    return Write::kPosition;
}

}  // namespace drive
