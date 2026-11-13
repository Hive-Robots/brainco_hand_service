// grip_guard.h against grips recorded on danni (test/grip_traces.h). No SDK, no hardware:
//   g++ -std=c++17 -Wall -Wextra -Iinclude test/test_grip_guard.cpp -o test_grip_guard && ./test_grip_guard
#include <climits>
#include <cstdio>
#include <set>
#include <vector>

#include "grip_guard.h"
#include "grip_traces.h"

namespace {

int failures = 0;

#define CHECK(cond)                                                     \
  do {                                                                  \
    if (!(cond)) {                                                      \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);       \
      ++failures;                                                       \
    }                                                                   \
  } while (0)

struct Run {
  std::vector<int> resend_ms;
  std::vector<grip::Resend> why;
  std::vector<grip::State> states;  // after each read
};

// On the robot `now` is steady_clock, seconds since boot, so it is never 0 at a switch.
constexpr int kBootMs = 3600 * 1000;

// Feeds a trace to a finger already gripping, re-applying the grip command before every read as
// the server's write loop does. `failed_before`: reads that follow failed ones. `offset_ms`
// places the trace later in the same grip; from `stale_from_ms` on, no command arrives any more.
template <size_t N>
Run feed(grip::Finger& f, const Sample (&trace)[N], bool fresh = true, std::set<int> failed_before = {}, int offset_ms = 0,
         int stale_from_ms = INT_MAX) {
  Run run;
  for (const Sample& s : trace) {
    const int ms = s.ms + offset_ms;
    const bool alive = fresh && s.ms < stale_from_ms;
    CHECK(!grip::on_command(f, true));  // the current is written on the switch alone
    const grip::Resend why = grip::on_reading(f, s.position, s.current, (kBootMs + ms) / 1000.0, failed_before.count(s.ms) > 0, alive);
    if (why != grip::Resend::kNone) {
      run.resend_ms.push_back(ms);
      run.why.push_back(why);
    }
    run.states.push_back(f.state);
  }
  return run;
}

template <size_t N>
Run replay(const Sample (&trace)[N], bool fresh = true, std::set<int> failed_before = {}) {
  grip::Finger f;
  grip::on_command(f, true);  // the switch
  return feed(f, trace, fresh, failed_before);
}

// The first read of the recorded dead thumb with under a tenth of the grip's current: from
// there to release, the recording shows the thumb without force.
int dead_thumb_drop_index() {
  for (size_t i = 0; i < sizeof(kDeadThumb) / sizeof(Sample); ++i) {
    if (kDeadThumb[i].current < 50) return static_cast<int>(i);
  }
  return -1;
}

// Churn: no re-send, one too early, an endless one, or a dropped finger reported as holding.
// Oracle: the recording, a thumb that drops after the switch and never comes back, and the
// longest self-recovery recorded (grip_traces.h), which a re-send must wait out.
void dead_thumb_is_resent_then_left() {
  const Run run = replay(kDeadThumb);
  const int drop = dead_thumb_drop_index();
  CHECK(drop > 0);
  CHECK(!run.resend_ms.empty());
  CHECK(!run.resend_ms.empty() && run.resend_ms[0] - kDeadThumb[drop].ms > kLongestSelfRecoveryMs);
  for (grip::Resend why : run.why) CHECK(why == grip::Resend::kNoForce);
  CHECK(run.resend_ms.size() == grip::kMaxResends);  // it stops at the cap, though the thumb never answers
  CHECK(run.states.back() == grip::State::kGaveUp);
  for (size_t i = drop; i < run.states.size(); ++i) CHECK(run.states[i] != grip::State::kConfirmed);
}

// Churn: a healthy hold re-sent, the buzz #493 removed, or a finger still closing taken for a
// dropped one. Oracles: recordings of a finger pressing at the grip current, of an empty finger
// the firmware cut at its end stop, of one closing at low current before it presses, and of the
// longest stretch any finger went without force before pressing again by itself.
void healthy_holds_get_no_extra_write() {
  for (const Run& run : {replay(kHealthyIndex), replay(kEmptyEndStop), replay(kSlowClose), replay(kSelfRecovery)}) {
    CHECK(run.resend_ms.empty());
    CHECK(run.states.back() == grip::State::kConfirmed);
  }
}

static_assert(grip::kWaitS * 1000 > kLongestSelfRecoveryMs, "a re-send would hit fingers that recover by themselves");

// Churn: a dropped grip re-armed while nothing commands the hand, or its state frozen at
// holding once commands stop. Oracle: the dead thumb, which presses at first and then, from its
// drop, shows no force whatever the stream does; here commands stop at the drop.
void stale_command_never_rearms() {
  grip::Finger f;
  grip::on_command(f, true);
  const int drop = dead_thumb_drop_index();
  const Run run = feed(f, kDeadThumb, true, {}, 0, kDeadThumb[drop].ms);
  CHECK(run.resend_ms.empty());
  CHECK(run.states[drop - 1] == grip::State::kConfirmed);
  for (size_t i = drop; i < run.states.size(); ++i) CHECK(run.states[i] != grip::State::kConfirmed);
  grip::Finger g;  // failed reads right at the drop do not bypass it either
  grip::on_command(g, true);
  CHECK(feed(g, kDeadThumb, true, {kDeadThumb[drop].ms}, 0, kDeadThumb[drop].ms).resend_ms.empty());
}

// Churn: failed reads answered by writing to fingers that still press, by waiting on one that
// has no force, or by pushing an empty finger into its end stop. Oracle: what each recording
// shows at that read: force in place, no force, closed empty.
void failed_reads_write_only_to_a_finger_without_force() {
  CHECK(replay(kHealthyIndex, true, {kHealthyIndex[30].ms}).resend_ms.empty());
  const int drop = dead_thumb_drop_index();
  const Run dead = replay(kDeadThumb, true, {kDeadThumb[drop].ms});
  CHECK(!dead.resend_ms.empty() && dead.resend_ms[0] == kDeadThumb[drop].ms);
  CHECK(!dead.why.empty() && dead.why[0] == grip::Resend::kAfterFailedReads);
  const size_t n = sizeof(kEmptyEndStop) / sizeof(Sample);
  CHECK(replay(kEmptyEndStop, true, {kEmptyEndStop[n - 5].ms}).resend_ms.empty());
}

// Churn: link blips using up the repair of a real drop (three blips once left a dropped thumb
// with no re-send at all). Oracle: the recordings, a pressing index then the dead thumb's drop
// in the same grip; the drop must still be re-sent.
void link_blips_leave_the_drop_repair_intact() {
  grip::Finger f;
  grip::on_command(f, true);
  const Run pressing = feed(f, kHealthyIndex, true, {kHealthyIndex[20].ms, kHealthyIndex[40].ms, kHealthyIndex[60].ms});
  CHECK(pressing.resend_ms.empty());
  const Run dropped = feed(f, kDeadThumb, true, {}, kHealthyIndex[sizeof(kHealthyIndex) / sizeof(Sample) - 1].ms + 10);
  CHECK(!dropped.resend_ms.empty());
}

// Churn: link blips while a finger is still closing taken for a drop, using up the re-sends a
// later real drop needs. Oracle: the recordings, a thumb closing at low current before it
// presses, then the dead thumb's drop in the same grip. The blips land on reads where the
// recording shows the thumb moving (over 1% of its travel since the previous read) and not
// pressing (under a quarter of the ~600 it holds once it presses).
void blips_during_a_close_leave_the_drop_repair_intact() {
  std::set<int> closing_reads;
  const size_t n = sizeof(kSlowClose) / sizeof(Sample);
  for (size_t i = 1; i < n && closing_reads.size() < 3; ++i) {
    if (kSlowClose[i].current < 150 && std::abs(kSlowClose[i].position - kSlowClose[i - 1].position) > 10) closing_reads.insert(kSlowClose[i].ms);
  }
  CHECK(!closing_reads.empty());
  grip::Finger f;
  grip::on_command(f, true);
  CHECK(feed(f, kSlowClose, true, closing_reads).resend_ms.empty());
  CHECK(!feed(f, kDeadThumb, true, {}, kSlowClose[n - 1].ms + 10).resend_ms.empty());
}

// Churn: a finger the guard gave up on written again in the same grip, after a failed read or
// after pressing for a while, or a new grip that inherits the old cap. Oracle: the dead thumb
// never answers, so further writes in that grip cannot help; a new press is a new grip.
void gave_up_until_release() {
  grip::Finger f;
  grip::on_command(f, true);
  feed(f, kDeadThumb);
  CHECK(f.state == grip::State::kGaveUp);
  const Sample last = kDeadThumb[sizeof(kDeadThumb) / sizeof(Sample) - 1];
  CHECK(grip::on_reading(f, last.position, last.current, 10.0, /*after_failed_reads=*/true, true) == grip::Resend::kNone);
  feed(f, kHealthyIndex, true, {}, 20000);
  CHECK(f.state == grip::State::kConfirmed);
  const Run again = feed(f, kDeadThumb, true, {}, 30000);
  CHECK(again.resend_ms.empty());
  for (size_t i = dead_thumb_drop_index(); i < again.states.size(); ++i) CHECK(again.states[i] == grip::State::kGaveUp);
  CHECK(!grip::on_command(f, false));
  CHECK(f.state == grip::State::kPosition);
  CHECK(grip::on_command(f, true));
  CHECK(!feed(f, kDeadThumb).resend_ms.empty());
}

}  // namespace

int main() {
  dead_thumb_is_resent_then_left();
  healthy_holds_get_no_extra_write();
  stale_command_never_rearms();
  failed_reads_write_only_to_a_finger_without_force();
  link_blips_leave_the_drop_repair_intact();
  blips_during_a_close_leave_the_drop_repair_intact();
  gave_up_until_release();
  std::printf(failures ? "%d check(s) failed\n" : "all checks passed\n", failures);
  return failures ? 1 : 0;
}
