// Arm v2: five closed-loop stepper joints on one CAN bus (MKS SERVO42D/57D class drivers).
// Pure C++17 like gantry_core: the CAN controller lives behind CanPort (TWAI on the ESP32,
// a simulated bus in the native tests). Frame format from the MKS SERVO42D/57D CAN User Manual
// V1.0.9 (github.com/makerbase-motor/MKS-SERVO42D-57D): standard 11-bit ID = driver address,
// payload = [code, args..., CRC] with CRC = (ID + sum(payload bytes)) & 0xFF.
#pragma once

#include <stddef.h>
#include <stdint.h>

namespace gantry {
namespace mks {

constexpr int32_t COUNTS_PER_REV = 16384;  // encoder "axis" units per motor turn (command 31H)

struct Frame {
  uint16_t id = 0;
  uint8_t dlc = 0;
  uint8_t d[8] = {0};
};

uint8_t crc(uint16_t id, const uint8_t* d, size_t n);
bool crcOk(const Frame& f);

// downlink (host -> driver)
Frame absAxis(uint16_t id, int32_t axis, uint16_t rpm, uint8_t acc);  // F5H absolute move by encoder axis
Frame readAxis(uint16_t id);                                          // 31H cumulative multi-turn axis (int48)
Frame enable(uint16_t id, bool on);                                   // F3H
Frame estop(uint16_t id);                                             // F7H emergency stop
Frame goHome(uint16_t id);                                            // 91H home on the limit switch
Frame readProtect(uint16_t id);                                       // 3EH locked-rotor (stall) protection state
Frame releaseProtect(uint16_t id);                                    // 3DH

// uplink (driver -> host)
enum class Kind : uint8_t { NONE, BAD_CRC, AXIS, MOVE, HOME, PROTECT, ENABLE, OTHER };
struct Reply {
  Kind kind = Kind::NONE;
  uint8_t code = 0;
  int64_t axis = 0;   // AXIS
  uint8_t status = 0; // MOVE: 0 fail 1 start 2 done 3 end-limit; HOME: 0 fail 1 start 2 ok; PROTECT: 1 = stalled
};
Reply decode(const Frame& f);

}  // namespace mks

struct CanPort {
  virtual ~CanPort() = default;
  virtual bool send(const mks::Frame& f) = 0;
  virtual bool receive(mks::Frame& f) = 0;  // non-blocking; false when nothing is pending
};

constexpr int NUM_JOINTS = 5;

struct JointConfig {
  uint16_t can_id = 1;
  float counts_per_deg = 0;  // COUNTS_PER_REV * total ratio / 360
  float min_deg = -180, max_deg = 180;
  float home_deg = 0;        // joint angle when the driver's home switch trips
  uint16_t max_rpm = 600;    // motor side
  uint8_t acc = 20;          // MKS acceleration byte (0-255)
  int8_t sign = 1;           // motor direction vs joint direction
};

class Arm {
 public:
  enum class Err : uint8_t { OK, NOT_HOMED, LIMIT, FAULT, BUSY };
  static constexpr uint32_t POLL_MS = 20;      // position poll period
  static constexpr uint32_t TIMEOUT_MS = 250;  // a joint that stays silent this long -> comm fault

  Arm(CanPort& port, const JointConfig* cfg);  // cfg[NUM_JOINTS]

  // Coordinated move: every commanded joint arrives at the same time. speed in (0, 1] of the
  // slowest commanded joint's max speed. bad_joint is set on LIMIT / NOT_HOMED.
  Err move(const bool* has, const float* deg, float speed, int* bad_joint);
  void home();             // 91H to every joint; each is homed when it reports success
  void estop();            // F7H broadcast + forget homing (positions can be pushed when unpowered)
  void enable(bool on);    // F3H; on also releases a latched stall protection (3DH)
  void update(uint32_t now_ms);  // drain the port, decode replies, poll, watch timeouts

  float deg(int j) const;
  bool homed(int j) const { return homed_[j]; }
  bool allHomed() const;
  bool homing() const;
  bool fault(int j) const { return stall_[j] || comm_[j] || home_fail_[j]; }
  bool anyFault() const;
  bool stalled(int j) const { return stall_[j]; }
  bool commFault(int j) const { return comm_[j]; }
  bool moving() const;
  const JointConfig& cfg(int j) const { return cfg_[j]; }

 private:
  int jointOf(uint16_t id) const;
  void onReply(int j, const mks::Reply& r);
  CanPort& port_;
  JointConfig cfg_[NUM_JOINTS];
  int64_t axis_[NUM_JOINTS] = {0};
  int64_t target_[NUM_JOINTS] = {0};
  bool homed_[NUM_JOINTS] = {false};
  bool homing_[NUM_JOINTS] = {false};
  bool home_fail_[NUM_JOINTS] = {false};
  bool moving_[NUM_JOINTS] = {false};
  bool stall_[NUM_JOINTS] = {false};
  bool comm_[NUM_JOINTS] = {false};
  uint32_t last_reply_[NUM_JOINTS] = {0};
  uint32_t last_poll_ = 0;
  uint32_t now_ = 0;
  bool poll_protect_ = false;
  bool started_ = false;
};

}  // namespace gantry
