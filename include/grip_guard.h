// Whether each finger's grip current is still in force, and when to write it again.
//
// The server writes the grip current once, when a finger's command reaches the grip
// threshold, because every write briefly dips the force on a held object. The firmware
// acknowledges no write, and the current can be gone afterwards: the thumb sometimes drops
// to no force right after the switch, and a write lost on the link or a hand reset leaves a
// finger the same way. So a grip counts as confirmed only while the hand's own status shows
// force, and it is written again only to a still finger that shows none. Pure logic, no
// SDK: test/test_grip_guard.cpp replays traces recorded on the robot through it.
#pragma once

#include <cstdint>
#include <cstdlib>

namespace grip {

// Published per finger in MotorState_.mode, so these values are part of the state topic.
enum class State : uint8_t { kPosition = 0, kSent = 1, kConfirmed = 2, kGaveUp = 3 };

enum class Resend { kNone, kAfterFailedReads, kNoForce };

// In the stark SDK's 0-1000 units.
constexpr int kForce = 300;     // a pressing finger holds near the grip current, which main.cpp asserts is above this
constexpr int kNoForce = 100;   // above the noise of a finger without force
constexpr int kStill = 5;       // position change per read: a finger still closing reads low current while it moves
constexpr int kEndStop = 950;   // closed on nothing, where the firmware cuts the current on purpose
constexpr double kWaitS = 0.15; // no-force stretches that end by themselves end well inside this
constexpr int kMaxResends = 3;  // per grip, whatever the cause: more writes do not help a finger that never answers

struct Finger {
  State state = State::kPosition;
  int resends_this_grip = 0;
  bool gave_up = false;        // no re-sends left this grip
  uint32_t resends_total = 0;  // published in MotorState_.lost
  int last_position = -1;      // unknown until the first read
  bool no_force = false;
  double no_force_since = 0.0;
};

// Before the writes. True on the loop the command reaches the grip: write the current then.
inline bool on_command(Finger& f, bool grip) {
  if (!grip) {
    f.state = State::kPosition;
    f.resends_this_grip = 0;
    f.gave_up = false;
    f.no_force = false;
    return false;
  }
  if (f.state != State::kPosition) return false;
  f.state = State::kSent;
  return true;
}

// After a good status read: the finger's state, and whether to write the grip current again.
// `after_failed_reads`: reads failed just before this one, so a still finger without force is
// written at once instead of after the wait. `fresh`: a well-formed command arrived lately;
// with nobody commanding, the state is still tracked but a dropped grip is not re-armed.
inline Resend on_reading(Finger& f, int position, int current, double now, bool after_failed_reads, bool fresh) {
  const bool moving = f.last_position < 0 || std::abs(position - f.last_position) > kStill;
  f.last_position = position;
  if (f.state == State::kPosition) return Resend::kNone;
  const bool end_stop = position >= kEndStop && current < kNoForce;
  if (current > kForce || end_stop) {
    f.state = State::kConfirmed;
    f.no_force = false;
    return Resend::kNone;
  }
  if (current >= kNoForce) {  // some current, not the grip's: no evidence either way yet
    f.no_force = false;
    return Resend::kNone;
  }
  f.state = f.gave_up ? State::kGaveUp : State::kSent;
  if (moving || !fresh) {  // a finger still closing reads no force too
    f.no_force = false;
    return Resend::kNone;
  }
  Resend why = Resend::kNone;
  if (after_failed_reads) {
    why = Resend::kAfterFailedReads;
  } else if (!f.no_force) {
    f.no_force = true;
    f.no_force_since = now;
  } else if (now - f.no_force_since >= kWaitS) {
    why = Resend::kNoForce;
  }
  if (why == Resend::kNone) return why;
  f.no_force = false;
  if (f.resends_this_grip >= kMaxResends) {
    f.gave_up = true;
    f.state = State::kGaveUp;
    return Resend::kNone;
  }
  ++f.resends_this_grip;
  ++f.resends_total;
  return why;
}

}  // namespace grip
