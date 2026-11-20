#include "stark-sdk.h"
#include "param.h"
#include "drive.h"

#include "dds/Publisher.h"
#include "dds/Subscription.h"
#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/common/thread/recurrent_thread.hpp>

#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <algorithm>
#include <array>
#include <atomic>
#include <filesystem>

std::atomic<bool> running(true);
void signal_handler(int) { running = false; }

// Brainco Hand ID
constexpr uint8_t L_id = 0x7e;
constexpr uint8_t R_id = 0x7f;
constexpr uint32_t baudrate = 460800;
// MotorCmd_.mode on a hand's commands asking it to close to rest with its drive off.
constexpr uint8_t kRestMode = 1;
// How long a rest may hold a finger before releasing it wherever it is. Above the time the
// close takes at kRestSpeed, so it releases only a finger that is blocked.
constexpr auto kRestReleaseAfter = std::chrono::seconds(5);

// DDS array order: [thumb, thumb_aux, index, middle, ring, pinky]
const StarkFingerId kFingerIds[6] = {
    STARK_FINGER_ID_THUMB, STARK_FINGER_ID_THUMB_AUX, STARK_FINGER_ID_INDEX,
    STARK_FINGER_ID_MIDDLE, STARK_FINGER_ID_RING, STARK_FINGER_ID_PINKY,
};

// Fingertip sensors are read every this many loops: each read is one more Modbus transaction on
// the serial line the motor writes and reads share, so it slows the motor loop.
constexpr int kTouchEvery = 2;
constexpr uint8_t kAllFingertips = 0x1F;  // thumb, index, middle, ring, pinky
// The hand's touch protocol says a calibrated fingertip's readings are invalid for this long.
constexpr auto kTouchSettle = std::chrono::seconds(2);
// A requested grip current is re-written only when it moved this much: each write dips the
// finger's current for a few milliseconds.
constexpr int16_t kCurrentStep = 10;

// ------------------ Utility ------------------
std::vector<std::string> getAvailableSerialPorts() {
    std::vector<std::string> ports;
    for (const auto& entry : std::filesystem::directory_iterator("/dev")) {
        std::string path = entry.path().string();

        if (path.rfind("/dev/ttyUSB", 0) == 0) ports.push_back(path);
        if (path.rfind("/dev/ttyHAND", 0) == 0) ports.push_back(path);
        if (path.rfind("/dev/ttyUN", 0) == 0) ports.push_back(path);
    }
    spdlog::info("Available Serial Ports: {}", fmt::join(ports, ", "));
    return ports;
}


// Hand connection struct
struct HandConnection {
    DeviceHandler* handle{nullptr};
    CDeviceInfo* info{nullptr};
    std::string port;
};

// Try connecting a hand on a given port
HandConnection try_connect_hand(const std::string& port, uint8_t slave_id) {
    HandConnection ret;
    ret.port = port;

    DeviceHandler* handle = modbus_open(port.c_str(), baudrate);
    if (!handle) {
        spdlog::warn("Failed to open {} at {} baud", port, baudrate);
        return ret;
    }

    CDeviceInfo* info = stark_get_device_info(handle, slave_id);
    if (!info) {
        spdlog::warn("Failed to get device info from {} port", port);
        modbus_close(handle);
        return ret;
    }

    spdlog::info("Hand hardware_type: {}", info->hardware_type);
    spdlog::info("Hand sku_type: {}", info->sku_type);
    spdlog::info("Hand firmware_version: {}", info->firmware_version);
    spdlog::info("Hand serial_number: {}", info->serial_number);

    stark_set_finger_unit_mode(handle, slave_id, FINGER_UNIT_MODE_NORMALIZED);

    ret.handle = handle;
    ret.info = info;
    return ret;
}

// Find a hand from available ports given allowed SKUs
HandConnection find_hand(std::vector<std::string>& ports, uint8_t slave_id, const std::vector<SkuType>& allowed_skus, const std::string& hand_name) {
    for (const auto& port : ports) {
        spdlog::info("Trying port for {} hand from {} port", hand_name, port);
        auto conn = try_connect_hand(port, slave_id);
        if (!conn.handle) continue;

        if (std::find(allowed_skus.begin(), allowed_skus.end(), conn.info->sku_type) != allowed_skus.end()) {
            spdlog::info("{} hand bound to {} port", hand_name, port);
            ports.erase(std::remove(ports.begin(), ports.end(), port), ports.end());
            return conn;
        } else {
            modbus_close(conn.handle);
            free_device_info(conn.info);
            spdlog::warn("Port {} is not {} hand (sku {}). Closed.", port, hand_name, (int)conn.info->sku_type);
        }
    }
    return {};
}

// ------------------ Hand Update Loop ------------------
void publish_status(const CMotorStatusData* status,
                    unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* lowstate,
                    const std::array<drive::Finger, 6>& fingers) {
    for (int i = 0; i < 6; ++i) {
        lowstate->msg_.states()[i].q()        = status->positions[i] / 1000.f;
        lowstate->msg_.states()[i].dq()       = status->speeds[i] / 1000.f;
        lowstate->msg_.states()[i].tau_est()  = status->currents[i] / 1000.f;
        lowstate->msg_.states()[i].mode()     = fingers[i].off ? drive::kParkedState : 0;
    }
    lowstate->unlockAndPublish();
}

void update_finger(DeviceHandler* handle, uint8_t slave_id,
                   unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>* lowcmd,
                   unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* lowstate,
                   unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>* grip,
                   const std::string& ns, std::array<drive::Finger, 6>& fingers) {
    // Per finger, one of three drives, chosen in drive.h. Position+speed alone can't tell
    // "stalled against an object" from "stalled against its own end-stop" -- both max out
    // current identically. Below the grip threshold a finger tracks position+speed: smooth,
    // safe open/approach *and* positioning motion. At/above it (a firm close/grip command)
    // it switches to direct current control instead: current mode is inherently compliant,
    // so the finger closes until it meets real resistance (object or hard-stop) and holds
    // there at exactly the commanded current, rather than slamming a position target and
    // letting the servo's stall behavior decide the force. Per-finger, not a whole-hand mode
    // switch, so a partial grip (e.g. index held open while the rest close) still works in
    // one tick. A finger whose command says it has arrived (mode kOffMode) has its drive
    // switched off: PWM 0 draws nothing and the gear train holds it. thumb_aux never takes
    // current mode and is never switched off, whatever its command: it swings the thumb
    // sideways with nothing to press against, so current mode drives it into its end stop,
    // where the firmware cuts its current and leaves it limp, and without power it has no
    // hold at all.
    uint16_t positions[6], speeds[6];
    uint8_t modes[6];
    for (int i = 0; i < 6; ++i) {
        positions[i] = static_cast<uint16_t>(std::clamp(lowcmd->msg_.cmds()[i].q(), 0.f, 1.f) * 1000.f);
        speeds[i]    = static_cast<uint16_t>(std::clamp(lowcmd->msg_.cmds()[i].dq(), 0.f, 1.f) * 1000.f);
        modes[i]     = lowcmd->msg_.cmds()[i].mode();
    }

    for (int i = 0; i < 6; ++i) speeds[i] = std::max<uint16_t>(speeds[i], 1);  // API requires speed >= 1

    // Per finger, the current a grip is held at: the default, unless rt/brainco/<side>/grip_current
    // (its tau, 1.0 = 1000) has spoken within the last second. Never above the default, so that
    // topic can only weaken a grip; it bypasses the FSM, which relays q, dq and mode only.
    int16_t currents[6];
    const bool steered = !grip->isTimeout() && grip->msg_.cmds().size() == 6;
    for (int i = 0; i < 6; ++i) {
        const float ceiling = drive::kGripCurrent[i] / 1000.f;
        const float asked = steered ? grip->msg_.cmds()[i].tau() : ceiling;
        currents[i] = static_cast<int16_t>(std::clamp(asked, 0.f, ceiling) * 1000.f);
    }

    std::array<drive::Write, 6> writes;
    bool all_positioned = true;
    for (int i = 0; i < 6; ++i) {
        const bool was_off = fingers[i].off;
        writes[i] = drive::choose(fingers[i], positions[i], modes[i], kFingerIds[i] == STARK_FINGER_ID_THUMB_AUX);
        if (writes[i] == drive::Write::kNone && fingers[i].in_current &&
            std::abs(currents[i] - fingers[i].current) >= kCurrentStep)
            writes[i] = drive::Write::kCurrent;
        if (fingers[i].in_current || fingers[i].off) all_positioned = false;
        if (fingers[i].off != was_off) spdlog::info("{} finger {}: drive {}", ns, i, fingers[i].off ? "off" : "on");
    }

    if (all_positioned) {
        // Every Modbus write is its own serial transaction, and six per-finger writes
        // stretch the loop well past its 10 ms period. The batched write sets all six
        // fingers, so it would knock a gripping finger out of current mode and re-energise
        // a finger that is off: only when every finger is positioned.
        stark_set_finger_positions_and_speeds(handle, slave_id, positions, speeds, 6);
    } else {
        for (int i = 0; i < 6; ++i) {
            switch (writes[i]) {
                case drive::Write::kPosition:
                    stark_set_finger_position_with_speed(handle, slave_id, kFingerIds[i], positions[i], speeds[i]);
                    break;
                case drive::Write::kCurrent:
                    stark_set_finger_current(handle, slave_id, kFingerIds[i], currents[i]);
                    fingers[i].current = currents[i];
                    break;
                case drive::Write::kPwmOff:
                    stark_set_finger_pwm(handle, slave_id, kFingerIds[i], 0);
                    break;
                case drive::Write::kNone:
                    break;
            }
        }
    }

    auto status = stark_get_motor_status(handle, slave_id);
    if (!status) return;
    publish_status(status, lowstate, fingers);
    free_motor_status_data(status);
}

// The pose a hand rests in until its first command, and whenever the FSM sends a rest
// request (an operator ending a teleop session). Position control takes each finger there
// and PWM 0 then lets it go: a closed hand presses its fingers into each other and into their
// end stops, so holding the target would draw stall current for as long as the robot sits
// uncommanded. A stalled finger is released where it stopped, since it can get no closer.
// `timed_out` releases every finger still held: a finger blocked short of its pose can hold
// it at its lock current without the firmware ever reporting a stall.
// A rest on a finger gripping in current mode moves it to a position target, so a finger
// blocked by the object pushes past its grip current, up to the firmware's limit, until it
// stalls, and then lets the object go. hive_fsm_ctrl never sends a rest to a hand whose command
// grips; anyone driving this service or the hand directly has to keep to that too.
void hold_rest_pose(DeviceHandler* handle, uint8_t slave_id,
                     unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* lowstate,
                     const std::array<drive::Finger, 6>& fingers,
                     std::array<bool, 6>& released, bool timed_out) {
    // Closed, at the full-grasp caps the teleop controller commands, except thumb_aux: the
    // closing thumb and index stop it short of that cap and it is not released there, so its
    // rest sits below where they block it.
    constexpr uint16_t kRestPose[6] = {980, 300, 980, 980, 980, 980};
    // A third of full speed: the hand closes at every start, before any command, and may be
    // closing on the hand of someone mounting or holding it.
    constexpr uint16_t kRestSpeed = 300;
    constexpr int kReachedTol = 20;  // normalized position (0-1000)

    for (int i = 0; i < 6; ++i) {
        if (!released[i]) stark_set_finger_position_with_speed(handle, slave_id, kFingerIds[i], kRestPose[i], kRestSpeed);
    }

    auto status = stark_get_motor_status(handle, slave_id);
    if (!status) return;
    for (int i = 0; i < 6; ++i) {
        bool reached = std::abs(status->positions[i] - kRestPose[i]) <= kReachedTol;
        if (!released[i] && (reached || timed_out || status->states[i] == MOTOR_STATE_STALL)) {
            stark_set_finger_pwm(handle, slave_id, kFingerIds[i], 0);
            released[i] = true;
        }
    }
    publish_status(status, lowstate, fingers);
    free_motor_status_data(status);
}

// Revo2 Touch: one capacitive sensor per fingertip, thumb to pinky, published as five MotorState_
// entries so every reader that decodes the hand's state decodes these too: q the normal force (N),
// dq the tangential force (N), ddq its direction (degrees, 0 toward the tip, -1 with none),
// tau_est the proximity (raw counts), lost the whole sensor status word, packet counter included
// (uint32, where mode's byte would cut it). Revo2 fills only the first of the SDK's three
// channels per field.
void publish_touch(DeviceHandler* handle, uint8_t slave_id,
                   unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* touch,
                   const std::string& ns, int& failures) {
    CTouchFingerData* data = stark_get_touch_status(handle, slave_id);
    if (!data) {
        if (failures++ % 100 == 0) spdlog::warn("{} fingertip read failed ({} so far)", ns, failures);
        return;
    }
    if (touch->trylock()) {
        for (int i = 0; i < 5; ++i) {
            const CTouchFingerItem& f = data->items[i];
            auto& s = touch->msg_.states()[i];
            s.q()       = f.normal_force1 / 100.f;
            s.dq()      = f.tangential_force1 / 100.f;
            s.ddq()     = f.tangential_direction1 == 0xFFFF ? -1.f : static_cast<float>(f.tangential_direction1);
            s.tau_est() = static_cast<float>(f.self_proximity1);
            s.lost()    = f.status;
        }
        touch->unlockAndPublish();
    }
    free_touch_finger_data(data);
}

// Clears a fingertip's stuck raw value (status bit 1) with the reset and its idle force offset
// (bit 2, its force frozen until calibrated) with the calibration. Both zero against whatever
// touches the fingertips now, so nothing may; readings are invalid for kTouchSettle after.
void zero_fingertips(DeviceHandler* handle, uint8_t slave_id) {
    stark_reset_touch_sensor(handle, slave_id, kAllFingertips);
    stark_calibrate_touch_sensor(handle, slave_id, kAllFingertips);
}

// A topic each message of which is one request, whatever it carries.
class Requests : public unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_> {
public:
    using unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>::SubscriptionBase;
    // Whether one came since the last call.
    bool take() { return pending_.exchange(false); }

protected:
    void post_communication() override { pending_ = true; }

private:
    std::atomic<bool> pending_{false};
};

// Worker thread for each hand
void hand_worker(DeviceHandler* handle, uint8_t slave_id, const std::string& ns, bool has_touch) {
    spdlog::info("🚀 Starting worker for {} (slave {})", ns, (int)slave_id);

    // DDS setup
    auto lowcmd   = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>("rt/brainco/" + ns + "/cmd");
    lowcmd->msg_.cmds().resize(6);
    for (auto& finger : lowcmd->msg_.cmds()) finger.dq() = 1.;

    auto lowstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>("rt/brainco/" + ns + "/state");
    lowstate->msg_.states().resize(6);

    auto grip = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>("rt/brainco/" + ns + "/grip_current");
    grip->set_timeout_ms(1000);

    // No raw channels: the SDK's raw read assumes Revo1's layout, and on a Revo2 its Rust code
    // panics, which aborts this whole process.
    std::unique_ptr<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>> touch;
    // rt/brainco/<side>/touch_calibrate: zero the fingertips again, as at start, whenever asked.
    // The sender holds the hand open and clear of everything first; the hand itself is not moved.
    std::unique_ptr<Requests> calibrate;
    auto touch_from = std::chrono::steady_clock::now();
    if (has_touch) {
        stark_enable_touch_sensor(handle, slave_id, kAllFingertips);
        // Enabled alone, a fingertip can come up flagged and never move. The service must start
        // with nothing touching them.
        zero_fingertips(handle, slave_id);
        touch_from += kTouchSettle;
        calibrate = std::make_unique<Requests>("rt/brainco/" + ns + "/touch_calibrate");
        touch = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>("rt/brainco/" + ns + "/touch");
        touch->msg_.states().resize(5);
        spdlog::info("{} fingertip sensors on, reset and calibrated, published every {} loops", ns, kTouchEvery);
    }

    std::array<drive::Finger, 6> fingers{};
    std::array<bool, 6> released{};
    int loops = 0, touch_failures = 0;
    // Latched: once commanded, a stream that goes quiet leaves the hand at its last command,
    // so a dead FSM or multiplexer never drops what a hand carries.
    bool commanded = false;
    bool resting = false;
    auto rest_since = std::chrono::steady_clock::now();
    while (running) {
        auto start_time = std::chrono::high_resolution_clock::now();
        commanded = commanded || !lowcmd->isTimeout();
        bool rest = !commanded || lowcmd->msg_.cmds()[0].mode() == kRestMode;
        if (rest != resting) {
            resting = rest;
            if (resting) rest_since = std::chrono::steady_clock::now();
            spdlog::info("{} hand {}", ns, resting ? "closing to rest" : "following commands");
        }
        if (rest) {
            // A rest leaves every finger positioned or released, so the next command starts
            // from no grip and no drive-off: a grip re-sends its current, an off re-sends PWM 0.
            fingers.fill({});
            const bool timed_out = std::chrono::steady_clock::now() - rest_since >= kRestReleaseAfter;
            hold_rest_pose(handle, slave_id, lowstate.get(), fingers, released, timed_out);
        } else {
            update_finger(handle, slave_id, lowcmd.get(), lowstate.get(), grip.get(), ns, fingers);
            released.fill(false);
        }
        ++loops;
        if (calibrate && calibrate->take()) {
            zero_fingertips(handle, slave_id);
            touch_from = std::chrono::steady_clock::now() + kTouchSettle;
            spdlog::info("{} fingertip sensors reset and calibrated on request", ns);
        }
        if (touch && loops % kTouchEvery == 0 && std::chrono::steady_clock::now() >= touch_from)
            publish_touch(handle, slave_id, touch.get(), ns, touch_failures);
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start_time).count();
        int sleep_us = 10000 - static_cast<int>(elapsed_us); // 100Hz
        if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
    }

    spdlog::info("Worker for {} exiting (closing handle)", ns);
    if (handle) modbus_close(handle);
}

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);

    auto vm = param::helper(argc, argv);
    unitree::robot::ChannelFactory::Instance()->Init(0, vm["network_interface"].as<std::string>());

    init_logging(LogLevel::LOG_LEVEL_ERROR);

    std::vector<std::string> available_ports = getAvailableSerialPorts();
    if (available_ports.empty()) {
        spdlog::warn("No ttyUSB serial ports found.");
        return 0;
    }

    HandConnection left_conn  = find_hand(available_ports, L_id, {SkuType::SKU_TYPE_SMALL_LEFT, SkuType::SKU_TYPE_MEDIUM_LEFT}, "left");
    HandConnection right_conn = find_hand(available_ports, R_id, {SkuType::SKU_TYPE_SMALL_RIGHT, SkuType::SKU_TYPE_MEDIUM_RIGHT}, "right");

    std::thread left_thread, right_thread;
    if (left_conn.handle)  left_thread  = std::thread(hand_worker, left_conn.handle, L_id, "left", stark_uses_revo2_touch_api(left_conn.info->hardware_type));
    if (right_conn.handle) right_thread = std::thread(hand_worker, right_conn.handle, R_id, "right", stark_uses_revo2_touch_api(right_conn.info->hardware_type));

    if (left_thread.joinable())  left_thread.join();
    if (right_thread.joinable()) right_thread.join();

    if (left_conn.info)  free_device_info(left_conn.info);
    if (right_conn.info) free_device_info(right_conn.info);

    spdlog::info("exit.");
    return 0;
}