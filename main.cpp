#include "stark-sdk.h"
#include "param.h"

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
#include <cstdlib>
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
                    unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* lowstate) {
    for (int i = 0; i < 6; ++i) {
        lowstate->msg_.states()[i].q()        = status->positions[i] / 1000.f;
        lowstate->msg_.states()[i].dq()       = status->speeds[i] / 1000.f;
        lowstate->msg_.states()[i].tau_est()  = status->currents[i] / 1000.f;
    }
    lowstate->unlockAndPublish();
}

void update_finger(DeviceHandler* handle, uint8_t slave_id,
                   unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>* lowcmd,
                   unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* lowstate,
                   std::array<bool, 6>& in_current) {
    // Hybrid position/current control. Position+speed alone can't tell "stalled
    // against an object" from "stalled against its own end-stop" -- both max
    // out current identically. Below kGripThreshold we track position+speed:
    // smooth, safe open/approach *and* positioning motion. At/above it (a firm
    // close/grip command) we switch that finger to direct current control
    // instead: current mode is inherently compliant, so the finger closes until
    // it meets real resistance (object or hard-stop) and holds there at exactly
    // the commanded current, rather than slamming a position target and letting
    // the servo's stall behavior decide the force. Per-finger, not a whole-hand
    // mode switch, so a partial grip (e.g. index held open while the rest close)
    // still works in one tick.
    //
    // The threshold sits below real grips: in the pose catalogue every genuine grip
    // is 0.9-0.98. thumb_aux never takes current mode, whatever its command: it
    // swings the thumb sideways and has nothing to press against, so current mode
    // drives it into its end stop, where the firmware cuts its current and leaves
    // it limp.
    constexpr uint16_t kGripThreshold = 800;  // normalized POSITION (0-1000)
    constexpr int16_t kGripCurrent = 500;    // normalized current (0-1000); tune from testing

    uint16_t positions[6], speeds[6];
    for (int i = 0; i < 6; ++i) {
        positions[i] = static_cast<uint16_t>(std::clamp(lowcmd->msg_.cmds()[i].q(), 0.f, 1.f) * 1000.f);
        speeds[i]    = static_cast<uint16_t>(std::clamp(lowcmd->msg_.cmds()[i].dq(), 0.f, 1.f) * 1000.f);
    }

    for (int i = 0; i < 6; ++i) speeds[i] = std::max<uint16_t>(speeds[i], 1);  // API requires speed >= 1

    bool any_grip = false;
    for (int i = 0; i < 6; ++i)
        if (positions[i] >= kGripThreshold && kFingerIds[i] != STARK_FINGER_ID_THUMB_AUX) any_grip = true;

    if (!any_grip) {
        // Every Modbus write is its own serial transaction, and six per-finger writes
        // stretch the loop well past its 10 ms period. The batched write sets all six
        // fingers, so it would knock a gripping finger out of current mode: only when
        // none grips.
        stark_set_finger_positions_and_speeds(handle, slave_id, positions, speeds, 6);
        in_current.fill(false);
    } else {
        for (int i = 0; i < 6; ++i) {
            if (positions[i] >= kGripThreshold && kFingerIds[i] != STARK_FINGER_ID_THUMB_AUX) {
                // Only on entry: each current write dips that finger's current for a few
                // milliseconds, so rewriting it every tick makes a held grip buzz.
                if (!in_current[i]) stark_set_finger_current(handle, slave_id, kFingerIds[i], kGripCurrent);
                in_current[i] = true;
            } else {
                stark_set_finger_position_with_speed(handle, slave_id, kFingerIds[i], positions[i], speeds[i]);
                in_current[i] = false;
            }
        }
    }

    auto status = stark_get_motor_status(handle, slave_id);
    if (!status) return;
    publish_status(status, lowstate);
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
// blocked by the object pushes past kGripCurrent, up to the firmware's limit, until it stalls,
// and then lets the object go. hive_fsm_ctrl never sends a rest to a hand whose command grips;
// anyone driving this service or the hand directly has to keep to that too.
void hold_rest_pose(DeviceHandler* handle, uint8_t slave_id,
                     unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>* lowstate,
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
    publish_status(status, lowstate);
    free_motor_status_data(status);
}

// Worker thread for each hand
void hand_worker(DeviceHandler* handle, uint8_t slave_id, const std::string& ns) {
    spdlog::info("🚀 Starting worker for {} (slave {})", ns, (int)slave_id);

    // DDS setup
    auto lowcmd   = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorCmds_>>("rt/brainco/" + ns + "/cmd");
    lowcmd->msg_.cmds().resize(6);

    auto lowstate = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorStates_>>("rt/brainco/" + ns + "/state");
    lowstate->msg_.states().resize(6);

    std::array<bool, 6> in_current{};
    std::array<bool, 6> released{};
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
            const bool timed_out = std::chrono::steady_clock::now() - rest_since >= kRestReleaseAfter;
            hold_rest_pose(handle, slave_id, lowstate.get(), released, timed_out);
            in_current.fill(false);
        } else {
            update_finger(handle, slave_id, lowcmd.get(), lowstate.get(), in_current);
            released.fill(false);
        }
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
    if (left_conn.handle)  left_thread  = std::thread(hand_worker, left_conn.handle, L_id, "left");
    if (right_conn.handle) right_thread = std::thread(hand_worker, right_conn.handle, R_id, "right");

    if (left_thread.joinable())  left_thread.join();
    if (right_thread.joinable()) right_thread.join();

    if (left_conn.info)  free_device_info(left_conn.info);
    if (right_conn.info) free_device_info(right_conn.info);

    spdlog::info("exit.");
    return 0;
}