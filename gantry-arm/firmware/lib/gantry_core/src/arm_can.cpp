#include "arm_can.h"

#include <math.h>
#include <string.h>

namespace gantry {
namespace mks {

uint8_t crc(uint16_t id, const uint8_t* d, size_t n) {
  uint32_t s = id;
  for (size_t i = 0; i < n; ++i) s += d[i];
  return static_cast<uint8_t>(s & 0xFF);
}

bool crcOk(const Frame& f) { return f.dlc >= 2 && crc(f.id, f.d, f.dlc - 1) == f.d[f.dlc - 1]; }

static Frame make(uint16_t id, const uint8_t* p, uint8_t n) {
  Frame f;
  f.id = id;
  memcpy(f.d, p, n);
  f.d[n] = crc(id, p, n);
  f.dlc = static_cast<uint8_t>(n + 1);
  return f;
}

Frame absAxis(uint16_t id, int32_t axis, uint16_t rpm, uint8_t acc) {
  if (rpm > 3000) rpm = 3000;
  if (axis > 8388607) axis = 8388607;
  if (axis < -8388607) axis = -8388607;
  uint32_t a = static_cast<uint32_t>(axis) & 0xFFFFFF;  // int24, two's complement
  const uint8_t p[7] = {0xF5, uint8_t(rpm >> 8), uint8_t(rpm), acc, uint8_t(a >> 16), uint8_t(a >> 8), uint8_t(a)};
  return make(id, p, 7);
}
Frame readAxis(uint16_t id) { const uint8_t p[1] = {0x31}; return make(id, p, 1); }
Frame enable(uint16_t id, bool on) { const uint8_t p[2] = {0xF3, uint8_t(on ? 1 : 0)}; return make(id, p, 2); }
Frame estop(uint16_t id) { const uint8_t p[1] = {0xF7}; return make(id, p, 1); }
Frame goHome(uint16_t id) { const uint8_t p[1] = {0x91}; return make(id, p, 1); }
Frame readProtect(uint16_t id) { const uint8_t p[1] = {0x3E}; return make(id, p, 1); }
Frame releaseProtect(uint16_t id) { const uint8_t p[1] = {0x3D}; return make(id, p, 1); }

Reply decode(const Frame& f) {
  Reply r;
  if (f.dlc < 2) return r;
  if (!crcOk(f)) { r.kind = Kind::BAD_CRC; return r; }
  r.code = f.d[0];
  switch (r.code) {
    case 0x31:
      if (f.dlc >= 8) {
        int64_t v = 0;
        for (int i = 1; i <= 6; ++i) v = (v << 8) | f.d[i];
        if (v & (int64_t(1) << 47)) v -= (int64_t(1) << 48);  // sign-extend int48
        r.kind = Kind::AXIS;
        r.axis = v;
      }
      return r;
    case 0xF5: r.kind = Kind::MOVE; r.status = f.d[1]; return r;
    case 0x91: r.kind = Kind::HOME; r.status = f.d[1]; return r;
    case 0x3E: r.kind = Kind::PROTECT; r.status = f.d[1]; return r;
    case 0xF3: r.kind = Kind::ENABLE; r.status = f.d[1]; return r;
    default: r.kind = Kind::OTHER; r.status = f.d[1]; return r;
  }
}

}  // namespace mks

Arm::Arm(CanPort& port, const JointConfig* cfg) : port_(port) {
  for (int j = 0; j < NUM_JOINTS; ++j) cfg_[j] = cfg[j];
}

int Arm::jointOf(uint16_t id) const {
  for (int j = 0; j < NUM_JOINTS; ++j)
    if (cfg_[j].can_id == id) return j;
  return -1;
}

float Arm::deg(int j) const {
  const JointConfig& c = cfg_[j];
  return c.home_deg + c.sign * static_cast<float>(axis_[j]) / c.counts_per_deg;
}

bool Arm::allHomed() const {
  for (int j = 0; j < NUM_JOINTS; ++j)
    if (!homed_[j]) return false;
  return true;
}
bool Arm::homing() const {
  for (int j = 0; j < NUM_JOINTS; ++j)
    if (homing_[j]) return true;
  return false;
}
bool Arm::anyFault() const {
  for (int j = 0; j < NUM_JOINTS; ++j)
    if (fault(j)) return true;
  return false;
}
bool Arm::moving() const {
  for (int j = 0; j < NUM_JOINTS; ++j)
    if (moving_[j]) return true;
  return false;
}

Arm::Err Arm::move(const bool* has, const float* deg_in, float speed, int* bad) {
  if (anyFault()) return Err::FAULT;
  if (homing()) return Err::BUSY;
  if (speed <= 0) speed = 0.5f;
  if (speed > 1) speed = 1;
  int64_t tgt[NUM_JOINTS];
  double dt_need = 0;  // seconds for the slowest commanded joint at its (scaled) max speed
  for (int j = 0; j < NUM_JOINTS; ++j) {
    if (!has[j]) continue;
    const JointConfig& c = cfg_[j];
    if (!homed_[j]) { if (bad) *bad = j; return Err::NOT_HOMED; }
    if (deg_in[j] < c.min_deg - 1e-3f || deg_in[j] > c.max_deg + 1e-3f) { if (bad) *bad = j; return Err::LIMIT; }
    tgt[j] = static_cast<int64_t>(llround(c.sign * (deg_in[j] - c.home_deg) * c.counts_per_deg));
    double delta = fabs(static_cast<double>(tgt[j] - axis_[j]));
    double counts_per_s = speed * c.max_rpm * mks::COUNTS_PER_REV / 60.0;
    double t = delta / counts_per_s;
    if (t > dt_need) dt_need = t;
  }
  for (int j = 0; j < NUM_JOINTS; ++j) {
    if (!has[j]) continue;
    const JointConfig& c = cfg_[j];
    double delta = fabs(static_cast<double>(tgt[j] - axis_[j]));
    double rpm = dt_need > 0 ? delta / dt_need * 60.0 / mks::COUNTS_PER_REV : speed * c.max_rpm;
    uint16_t r = static_cast<uint16_t>(ceil(rpm));
    if (r < 1) r = 1;
    if (r > c.max_rpm) r = c.max_rpm;
    port_.send(mks::absAxis(c.can_id, static_cast<int32_t>(tgt[j]), r, c.acc));
    target_[j] = tgt[j];
    moving_[j] = tgt[j] != axis_[j];
  }
  return Err::OK;
}

void Arm::home() {
  for (int j = 0; j < NUM_JOINTS; ++j) {
    homed_[j] = false;
    home_fail_[j] = false;
    homing_[j] = true;
    port_.send(mks::goHome(cfg_[j].can_id));
  }
}

void Arm::estop() {
  for (int j = 0; j < NUM_JOINTS; ++j) {
    port_.send(mks::estop(cfg_[j].can_id));
    homed_[j] = false;  // unpowered joints can be back-driven: home again after RESET
    homing_[j] = false;
    moving_[j] = false;
  }
}

void Arm::enable(bool on) {
  for (int j = 0; j < NUM_JOINTS; ++j) {
    if (on && stall_[j]) port_.send(mks::releaseProtect(cfg_[j].can_id));
    port_.send(mks::enable(cfg_[j].can_id, on));
    if (on) { stall_[j] = false; comm_[j] = false; last_reply_[j] = now_; }
  }
}

void Arm::onReply(int j, const mks::Reply& r) {
  last_reply_[j] = now_;
  comm_[j] = false;
  switch (r.kind) {
    case mks::Kind::AXIS:
      axis_[j] = r.axis;
      if (moving_[j] && axis_[j] == target_[j]) moving_[j] = false;
      break;
    case mks::Kind::MOVE:
      if (r.status == 2 || r.status == 0 || r.status == 3) moving_[j] = false;
      break;
    case mks::Kind::HOME:
      if (r.status == 2) {
        homing_[j] = false;
        homed_[j] = true;
        axis_[j] = 0;  // the driver zeroes its axis on the switch
        target_[j] = 0;
      } else if (r.status == 0) {
        homing_[j] = false;
        home_fail_[j] = true;
      }
      break;
    case mks::Kind::PROTECT:
      if (r.status == 1) {
        stall_[j] = true;  // locked rotor: collision or overload
        moving_[j] = false;
      }
      break;
    default:
      break;
  }
}

void Arm::update(uint32_t now_ms) {
  now_ = now_ms;
  if (!started_) {
    started_ = true;
    for (int j = 0; j < NUM_JOINTS; ++j) last_reply_[j] = now_ms;
  }
  mks::Frame f;
  while (port_.receive(f)) {
    int j = jointOf(f.id);
    if (j < 0) continue;
    mks::Reply r = mks::decode(f);
    if (r.kind == mks::Kind::BAD_CRC || r.kind == mks::Kind::NONE) continue;
    onReply(j, r);
  }
  if (now_ms - last_poll_ >= POLL_MS) {
    last_poll_ = now_ms;
    poll_protect_ = !poll_protect_;  // alternate: positions, stall state
    for (int j = 0; j < NUM_JOINTS; ++j) port_.send(poll_protect_ ? mks::readProtect(cfg_[j].can_id) : mks::readAxis(cfg_[j].can_id));
  }
  for (int j = 0; j < NUM_JOINTS; ++j)
    if (now_ms - last_reply_[j] > TIMEOUT_MS) comm_[j] = true;
}

}  // namespace gantry
