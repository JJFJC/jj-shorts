// Gantry Arm ESP32 controller core: protocol, homing state machine, soft limits, e-stop.
// Pure C++17, no Arduino / heap / exceptions, so it builds and is tested natively
// (`pio test -e native`). Hardware lives behind AxisHal / IoHal (see src/main.cpp).
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "arm_can.h"

namespace gantry {

constexpr const char* FW_VERSION = "0.1.0";
constexpr size_t kLineMax = 96;
constexpr size_t kReplyMax = 160;
constexpr int NUM_AXES = 2;
enum AxisId : uint8_t { AX_X = 0, AX_Z = 1 };

// ------------------------------------------------------------------ hardware boundary
struct AxisHal {
  virtual ~AxisHal() = default;
  virtual void moveTo(float mm, float speed_mm_s) = 0;  // trapezoid/jerk-limited move
  virtual void stop() = 0;                              // decelerate to stop
  virtual void forceStop() = 0;                         // immediate stop (e-stop)
  virtual float position() const = 0;                   // mm
  virtual void setPosition(float mm) = 0;
  virtual bool running() const = 0;
  virtual bool limitMin() const = 0;  // homing switch (min end), true = pressed
  virtual bool limitMax() const = 0;  // over-travel switch (max end)
  virtual void enable(bool on) = 0;   // driver enable (TMC2209 EN)
};

struct IoHal {
  virtual ~IoHal() = default;
  virtual bool estopPressed() const = 0;  // hardware e-stop loop open
  virtual int toolIdRaw() const = 0;      // ADC counts 0..4095 on TOOL_ID pogo pin
};

// ------------------------------------------------------------------ protocol
enum class Cmd : uint8_t { NONE, PING, HOME, MOVE, JOG, STOP, ESTOP, RESET, STATUS, LED, TOOL, LIMITS, JMOVE, JPOS, JHOME };

struct Request {
  Cmd cmd = Cmd::NONE;
  bool has[NUM_AXES] = {false, false};
  float val[NUM_AXES] = {0, 0};      // MOVE/JOG/HOME targets (HOME: has[] = which axes)
  float lo[NUM_AXES] = {0, 0};       // LIMITS
  float hi[NUM_AXES] = {0, 0};
  bool has_feed = false;
  float feed = 0;                    // F<mm/s>
  char led[12] = {0};                // LED state word
  bool jhas[NUM_JOINTS] = {false, false, false, false, false};  // JMOVE J1<deg> .. J5<deg>
  float jval[NUM_JOINTS] = {0, 0, 0, 0, 0};
};

enum class ParseErr : uint8_t { OK, EMPTY, TOO_LONG, CHECKSUM, UNKNOWN_CMD, BAD_ARG };

// Parse one line (no trailing newline required). Optional "*HH" XOR checksum of the
// bytes before '*'. Commands are case-insensitive.
ParseErr parseLine(const char* line, Request& out);
uint8_t checksum(const char* s, size_t n);
const char* parseErrName(ParseErr e);

// ------------------------------------------------------------------ axis + homing
enum class HomeState : uint8_t { IDLE, SEEK, BACKOFF, SLOW, OFFSET, DONE, FAILED };

struct AxisConfig {
  float soft_min = 0, soft_max = 0;   // mm, after homing (0 = switch position + home_offset)
  float max_speed = 100;              // mm/s
  float home_fast = 30, home_slow = 4;
  float home_backoff = 3;             // mm
  float home_offset = 2;              // mm off the switch after homing -> becomes 0
  float home_search = 0;              // max travel while seeking (mm); fail beyond
};

class Axis {
 public:
  Axis(AxisHal& hal, const AxisConfig& cfg) : hal_(hal), cfg_(cfg) {}
  void startHoming();
  void abortHoming();  // stop; an unfinished homing leaves the axis unhomed
  void update();  // call every loop
  bool homed() const { return homed_; }
  void setHomed(bool h) { homed_ = h; }
  HomeState homeState() const { return hs_; }
  bool withinSoft(float mm) const { return mm >= cfg_.soft_min - 1e-3f && mm <= cfg_.soft_max + 1e-3f; }
  float clampSoft(float mm) const { return mm < cfg_.soft_min ? cfg_.soft_min : (mm > cfg_.soft_max ? cfg_.soft_max : mm); }
  AxisConfig& cfg() { return cfg_; }
  const AxisConfig& cfg() const { return cfg_; }
  AxisHal& hal() { return hal_; }
  const AxisHal& hal() const { return hal_; }
  bool limitFault() const { return limit_fault_; }
  void clearFault() { limit_fault_ = false; }

 private:
  AxisHal& hal_;
  AxisConfig cfg_;
  HomeState hs_ = HomeState::IDLE;
  bool homed_ = false;
  bool limit_fault_ = false;
  float seek_start_ = 0;
};

// ------------------------------------------------------------------ controller
enum class SysState : uint8_t { IDLE, MOVING, HOMING, ESTOP, FAULT };
enum class Led : uint8_t { OFF, IDLE, MOVING, HOMING, FAULT };

class Controller {
 public:
  Controller(AxisHal& x, AxisHal& z, IoHal& io, const AxisConfig& cx, const AxisConfig& cz);
  // Handle one received line; writes the reply (no newline) into `reply`.
  void handleLine(const char* line, char* reply, size_t reply_len);
  // Call every loop with a monotonic millisecond clock. Returns true and fills `event`
  // when there is an asynchronous event to send (e.g. "ev HOMED X").
  bool update(uint32_t now_ms, char* event, size_t event_len);
  SysState state() const;
  Led led() const;
  int toolId() const;
  Axis& axis(int i) { return *axes_[i]; }
  void attachArm(Arm* arm) { arm_ = arm; }  // v2 stepper arm on CAN (optional)

  static constexpr uint32_t JOG_WATCHDOG_MS = 300;

 private:
  void statusLine(char* out, size_t n) const;
  Axis ax_x_, ax_z_;
  Axis* axes_[NUM_AXES];
  AxisConfig hard_[NUM_AXES];  // limits from build config; LIMITS may only narrow them
  IoHal& io_;
  bool estop_latched_ = false;
  bool jogging_ = false;
  uint32_t last_jog_ms_ = 0;
  uint32_t now_ms_ = 0;
  Led led_req_ = Led::IDLE;
  HomeState last_hs_[NUM_AXES] = {HomeState::IDLE, HomeState::IDLE};
  bool home_queue_[NUM_AXES] = {false, false};
  bool reported_fault_[NUM_AXES] = {false, false};
  Arm* arm_ = nullptr;
  bool arm_homed_reported_ = false;
  bool arm_fault_reported_[NUM_JOINTS] = {false, false, false, false, false};
};

const char* stateName(SysState s);

}  // namespace gantry
