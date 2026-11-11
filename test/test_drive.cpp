// drive::choose, off the hand. Churn: the per-finger write choice. Oracle: the SDK's cost of
// each write (a current write dips the current, a PWM 0 write is the drive-off itself, a
// position write re-energises), so each is made exactly when the state asks for it.
#include "drive.h"

#include <cstdio>
#include <cstdlib>

using drive::Finger;
using drive::Write;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            std::fprintf(stderr, "%s:%d: CHECK(%s) failed\n", __FILE__, __LINE__, #cond); \
            std::exit(1);                                                      \
        }                                                                      \
    } while (0)

int main() {
    // Entering off writes PWM 0 once and only once, whatever the position says.
    Finger f;
    CHECK(drive::choose(f, 980, drive::kOffMode, false) == Write::kPwmOff);
    CHECK(f.off && !f.in_current);
    for (int tick = 0; tick < 100; ++tick) {
        CHECK(drive::choose(f, 980, drive::kOffMode, false) == Write::kNone);
    }
    CHECK(drive::choose(f, 100, drive::kOffMode, false) == Write::kNone);

    // Leaving off at or above the grip threshold writes the grip current again, once.
    CHECK(drive::choose(f, 980, 0, false) == Write::kCurrent);
    CHECK(!f.off && f.in_current);
    CHECK(drive::choose(f, 980, 0, false) == Write::kNone);

    // A grip that is then parked and un-parked re-sends its current: off clears in_current.
    CHECK(drive::choose(f, 980, drive::kOffMode, false) == Write::kPwmOff);
    CHECK(drive::choose(f, 980, 0, false) == Write::kCurrent);

    // Leaving off below the threshold positions the finger, every tick.
    CHECK(drive::choose(f, 980, drive::kOffMode, false) == Write::kPwmOff);
    CHECK(drive::choose(f, 300, 0, false) == Write::kPosition);
    CHECK(drive::choose(f, 300, 0, false) == Write::kPosition);
    CHECK(!f.off && !f.in_current);

    // thumb_aux takes the position path at any command, mode included.
    Finger aux;
    CHECK(drive::choose(aux, 980, drive::kOffMode, true) == Write::kPosition);
    CHECK(drive::choose(aux, 980, 0, true) == Write::kPosition);
    CHECK(!aux.off && !aux.in_current);

    // A mode this header does not know drives the finger as 0 would.
    Finger odd;
    CHECK(drive::choose(odd, 300, 7, false) == Write::kPosition);
    CHECK(drive::choose(odd, 980, 7, false) == Write::kCurrent);

    std::puts("test_drive: ok");
    return 0;
}
