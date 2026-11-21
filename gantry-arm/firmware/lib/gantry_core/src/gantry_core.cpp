#include "gantry_core.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

namespace gantry {

static const char AXIS_CH[NUM_AXES] = {'X', 'Z'};

// ================================================================== protocol

uint8_t checksum(const char* s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) c ^= static_cast<uint8_t>(s[i]);
  return c;
}

const char* parseErrName(ParseErr e) {
  switch (e) {
    case ParseErr::OK: return "OK";
    case ParseErr::EMPTY: return "EMPTY";
    case ParseErr::TOO_LONG: return "TOO_LONG";
    case ParseErr::CHECKSUM: return "CHECKSUM";
    case ParseErr::UNKNOWN_CMD: return "UNKNOWN_CMD";
    case ParseErr::BAD_ARG: return "BAD_ARG";
  }
  return "?";
}

static bool parseFloat(const char* s, float& out) {
  if (!*s) return false;
  char* end = nullptr;
  out = strtof(s, &end);
  return end && *end == '\0';
}

static int axisIndex(char c) {
  if (c == 'X') return AX_X;
  if (c == 'Z') return AX_Z;
  return -1;
}

ParseErr parseLine(const char* line, Request& out) {
  out = Request();
  size_t n = strlen(line);
  while (n && (line[n - 1] == '\n' || line[n - 1] == '\r' || line[n - 1] == ' ')) --n;
  if (n == 0) return ParseErr::EMPTY;
  if (n >= kLineMax) return ParseErr::TOO_LONG;
  char buf[kLineMax];
  memcpy(buf, line, n);
  buf[n] = '\0';
  // optional "*HH" checksum
  char* star = strrchr(buf, '*');
  if (star) {
    size_t body = static_cast<size_t>(star - buf);
    if (strlen(star + 1) != 2 || !isxdigit((unsigned char)star[1]) || !isxdigit((unsigned char)star[2])) return ParseErr::CHECKSUM;
    unsigned want = static_cast<unsigned>(strtoul(star + 1, nullptr, 16));
    if (checksum(buf, body) != want) return ParseErr::CHECKSUM;
    *star = '\0';
  }
  for (char* p = buf; *p; ++p) *p = static_cast<char>(toupper((unsigned char)*p));
  // tokenize
  char* tok[12];
  int nt = 0;
  for (char* p = strtok(buf, " \t"); p && nt < 12; p = strtok(nullptr, " \t")) tok[nt++] = p;
  if (nt == 0) return ParseErr::EMPTY;
  const char* c = tok[0];
  struct { const char* name; Cmd cmd; } table[] = {
      {"PING", Cmd::PING}, {"HOME", Cmd::HOME},   {"MOVE", Cmd::MOVE},     {"JOG", Cmd::JOG},
      {"STOP", Cmd::STOP}, {"ESTOP", Cmd::ESTOP}, {"RESET", Cmd::RESET},   {"STATUS", Cmd::STATUS},
      {"LED", Cmd::LED},   {"TOOL", Cmd::TOOL},   {"LIMITS", Cmd::LIMITS}, {"JMOVE", Cmd::JMOVE},
      {"JPOS", Cmd::JPOS}, {"JHOME", Cmd::JHOME},
  };
  for (auto& t : table)
    if (strcmp(c, t.name) == 0) out.cmd = t.cmd;
  if (out.cmd == Cmd::NONE) return ParseErr::UNKNOWN_CMD;

  switch (out.cmd) {
    case Cmd::PING: case Cmd::STOP: case Cmd::ESTOP: case Cmd::RESET: case Cmd::STATUS: case Cmd::TOOL:
    case Cmd::JPOS: case Cmd::JHOME:
      return nt == 1 ? ParseErr::OK : ParseErr::BAD_ARG;
    case Cmd::JMOVE: {  // JMOVE J1<deg> ... J5<deg> [F<speed 0..1>]
      if (nt < 2) return ParseErr::BAD_ARG;
      bool any = false;
      for (int i = 1; i < nt; ++i) {
        float v;
        if (tok[i][0] == 'F') {
          if (out.has_feed || !parseFloat(tok[i] + 1, v) || v <= 0 || v > 1) return ParseErr::BAD_ARG;
          out.has_feed = true;
          out.feed = v;
          continue;
        }
        if (tok[i][0] != 'J' || tok[i][1] < '1' || tok[i][1] > '0' + NUM_JOINTS) return ParseErr::BAD_ARG;
        int j = tok[i][1] - '1';
        if (out.jhas[j] || !parseFloat(tok[i] + 2, v)) return ParseErr::BAD_ARG;
        out.jhas[j] = true;
        out.jval[j] = v;
        any = true;
      }
      return any ? ParseErr::OK : ParseErr::BAD_ARG;
    }
    case Cmd::HOME:
      if (nt == 1) { out.has[AX_X] = out.has[AX_Z] = true; return ParseErr::OK; }
      for (int i = 1; i < nt; ++i) {
        if (strlen(tok[i]) != 1) return ParseErr::BAD_ARG;
        int a = axisIndex(tok[i][0]);
        if (a < 0) return ParseErr::BAD_ARG;
        out.has[a] = true;
      }
      return ParseErr::OK;
    case Cmd::MOVE: case Cmd::JOG: {
      if (nt < 2) return ParseErr::BAD_ARG;
      for (int i = 1; i < nt; ++i) {
        char k = tok[i][0];
        float v;
        if (!parseFloat(tok[i] + 1, v)) return ParseErr::BAD_ARG;
        if (k == 'F' && out.cmd == Cmd::MOVE) {
          if (out.has_feed || v <= 0) return ParseErr::BAD_ARG;
          out.has_feed = true;
          out.feed = v;
          continue;
        }
        int a = axisIndex(k);
        if (a < 0 || out.has[a]) return ParseErr::BAD_ARG;
        out.has[a] = true;
        out.val[a] = v;
      }
      return (out.has[AX_X] || out.has[AX_Z]) ? ParseErr::OK : ParseErr::BAD_ARG;
    }
    case Cmd::LIMITS:
      if (nt < 2) return ParseErr::BAD_ARG;
      for (int i = 1; i < nt; ++i) {
        int a = axisIndex(tok[i][0]);
        if (a < 0 || out.has[a]) return ParseErr::BAD_ARG;
        char* comma = strchr(tok[i] + 1, ',');
        if (!comma) return ParseErr::BAD_ARG;
        *comma = '\0';
        if (!parseFloat(tok[i] + 1, out.lo[a]) || !parseFloat(comma + 1, out.hi[a]) || out.lo[a] >= out.hi[a]) return ParseErr::BAD_ARG;
        out.has[a] = true;
      }
      return ParseErr::OK;
    case Cmd::LED:
      if (nt != 2) return ParseErr::BAD_ARG;
      if (strcmp(tok[1], "IDLE") && strcmp(tok[1], "MOVING") && strcmp(tok[1], "HOMING") && strcmp(tok[1], "FAULT") && strcmp(tok[1], "OFF"))
        return ParseErr::BAD_ARG;
      strncpy(out.led, tok[1], sizeof(out.led) - 1);
      return ParseErr::OK;
    default:
      return ParseErr::UNKNOWN_CMD;
  }
}

// ================================================================== axis / homing

void Axis::startHoming() {
  limit_fault_ = false;
  homed_ = false;
  hal_.enable(true);
  if (hal_.limitMin()) {  // already on the switch: back off first
    hal_.moveTo(hal_.position() + cfg_.home_backoff, cfg_.home_fast);
    hs_ = HomeState::BACKOFF;
    return;
  }
  seek_start_ = hal_.position();
  hal_.moveTo(seek_start_ - cfg_.home_search, cfg_.home_fast);
  hs_ = HomeState::SEEK;
}

void Axis::abortHoming() {
  if (hs_ == HomeState::SEEK || hs_ == HomeState::BACKOFF || hs_ == HomeState::SLOW || hs_ == HomeState::OFFSET) {
    hs_ = HomeState::IDLE;
    homed_ = false;
  }
  hal_.stop();
}

void Axis::update() {
  switch (hs_) {
    case HomeState::SEEK:
      if (hal_.limitMin()) {
        hal_.forceStop();
        hal_.moveTo(hal_.position() + cfg_.home_backoff, cfg_.home_fast);
        hs_ = HomeState::BACKOFF;
      } else if (!hal_.running()) {
        hs_ = HomeState::FAILED;  // travelled home_search without finding the switch
      }
      return;
    case HomeState::BACKOFF:
      if (!hal_.running()) {
        if (hal_.limitMin()) { hs_ = HomeState::FAILED; return; }  // switch stuck closed
        hal_.moveTo(hal_.position() - 2.0f * cfg_.home_backoff, cfg_.home_slow);
        hs_ = HomeState::SLOW;
      }
      return;
    case HomeState::SLOW:
      if (hal_.limitMin()) {
        hal_.forceStop();
        hal_.setPosition(-cfg_.home_offset);
        hal_.moveTo(0.0f, cfg_.home_slow);
        hs_ = HomeState::OFFSET;
      } else if (!hal_.running()) {
        hs_ = HomeState::FAILED;
      }
      return;
    case HomeState::OFFSET:
      if (!hal_.running()) {
        homed_ = true;
        hs_ = HomeState::DONE;
      }
      return;
    default:
      break;
  }
  // outside homing: any end switch while homed is an over-travel fault
  if (homed_ && (hal_.limitMin() || hal_.limitMax())) {
    hal_.forceStop();
    homed_ = false;
    limit_fault_ = true;
  }
}

// ================================================================== controller

const char* stateName(SysState s) {
  switch (s) {
    case SysState::IDLE: return "IDLE";
    case SysState::MOVING: return "MOVING";
    case SysState::HOMING: return "HOMING";
    case SysState::ESTOP: return "ESTOP";
    case SysState::FAULT: return "FAULT";
  }
  return "?";
}

Controller::Controller(AxisHal& x, AxisHal& z, IoHal& io, const AxisConfig& cx, const AxisConfig& cz)
    : ax_x_(x, cx), ax_z_(z, cz), io_(io) {
  axes_[AX_X] = &ax_x_;
  axes_[AX_Z] = &ax_z_;
  hard_[AX_X] = cx;
  hard_[AX_Z] = cz;
}

static bool homingActive(HomeState h) {
  return h == HomeState::SEEK || h == HomeState::BACKOFF || h == HomeState::SLOW || h == HomeState::OFFSET;
}

SysState Controller::state() const {
  if (estop_latched_ || io_.estopPressed()) return SysState::ESTOP;
  for (int i = 0; i < NUM_AXES; ++i)
    if (axes_[i]->limitFault() || axes_[i]->homeState() == HomeState::FAILED) return SysState::FAULT;
  if (arm_ && arm_->anyFault()) return SysState::FAULT;
  if (arm_ && arm_->homing()) return SysState::HOMING;
  for (int i = 0; i < NUM_AXES; ++i)
    if (homingActive(axes_[i]->homeState()) || home_queue_[i]) return SysState::HOMING;
  for (int i = 0; i < NUM_AXES; ++i)
    if (axes_[i]->hal().running()) return SysState::MOVING;
  if (arm_ && arm_->moving()) return SysState::MOVING;
  return SysState::IDLE;
}

Led Controller::led() const {
  switch (state()) {
    case SysState::ESTOP: case SysState::FAULT: return Led::FAULT;
    case SysState::HOMING: return Led::HOMING;
    case SysState::MOVING: return led_req_ == Led::FAULT ? Led::FAULT : Led::MOVING;
    default: return led_req_;
  }
}

int Controller::toolId() const {
  // 10k pull-up to 3.3 V, tool resistor to GND: open=0 (no tool), 1k=1 gripper,
  // 2.2k=2 suction, 4.7k=3 pen, 10k=4 phone mount. Midpoint thresholds in ADC counts.
  int r = io_.toolIdRaw();
  if (r < 190) return -1;    // shorted: fault
  if (r < 555) return 1;     // ~372
  if (r < 1020) return 2;    // ~738
  if (r < 1680) return 3;    // ~1309
  if (r < 3070) return 4;    // ~2048
  return 0;                  // open
}

void Controller::statusLine(char* out, size_t n) const {
  char h[3] = {'-', '-', 0}, l[5] = {'-', '-', '-', '-', 0};
  for (int i = 0; i < NUM_AXES; ++i) {
    if (axes_[i]->homed()) h[i] = AXIS_CH[i];
    if (axes_[i]->hal().limitMin()) l[2 * i] = AXIS_CH[i];
    if (axes_[i]->hal().limitMax()) l[2 * i + 1] = AXIS_CH[i];
  }
  snprintf(out, n, "st S=%s X=%.3f Z=%.3f H=%s L=%s T=%d", stateName(state()), (double)ax_x_.hal().position(),
           (double)ax_z_.hal().position(), h, l, toolId());
}

void Controller::handleLine(const char* line, char* reply, size_t rn) {
  Request r;
  ParseErr e = parseLine(line, r);
  if (e != ParseErr::OK) { snprintf(reply, rn, "err %s", parseErrName(e)); return; }
  const bool estop = estop_latched_ || io_.estopPressed();
  switch (r.cmd) {
    case Cmd::PING: snprintf(reply, rn, "ok PONG %s", FW_VERSION); return;
    case Cmd::STATUS: statusLine(reply, rn); return;
    case Cmd::TOOL: snprintf(reply, rn, "ok TOOL %d", toolId()); return;
    case Cmd::JPOS: {
      if (!arm_) { snprintf(reply, rn, "err NO_ARM"); return; }
      char h[NUM_JOINTS + 1], f[NUM_JOINTS + 1];
      for (int j = 0; j < NUM_JOINTS; ++j) {
        h[j] = arm_->homed(j) ? char('1' + j) : '-';
        f[j] = arm_->stalled(j) ? 'S' : arm_->commFault(j) ? 'C' : arm_->fault(j) ? 'H' : '-';
      }
      h[NUM_JOINTS] = f[NUM_JOINTS] = 0;
      snprintf(reply, rn, "ok JPOS %.3f %.3f %.3f %.3f %.3f H=%s F=%s", (double)arm_->deg(0), (double)arm_->deg(1), (double)arm_->deg(2),
               (double)arm_->deg(3), (double)arm_->deg(4), h, f);
      return;
    }
    case Cmd::LED:
      led_req_ = !strcmp(r.led, "FAULT") ? Led::FAULT : !strcmp(r.led, "OFF") ? Led::OFF : Led::IDLE;
      snprintf(reply, rn, "ok");
      return;
    case Cmd::ESTOP:
      estop_latched_ = true;
      for (auto* a : axes_) { a->abortHoming(); a->hal().forceStop(); a->hal().enable(false); }
      ax_x_.setHomed(false);  // belt axis can be pushed when unpowered; Z (T8x2) self-locks
      jogging_ = false;
      home_queue_[AX_X] = home_queue_[AX_Z] = false;
      if (arm_) { arm_->estop(); arm_homed_reported_ = false; }
      snprintf(reply, rn, "ok");
      return;
    case Cmd::RESET:
      if (io_.estopPressed()) { snprintf(reply, rn, "err ESTOP_HW"); return; }
      estop_latched_ = false;
      for (auto* a : axes_) { a->clearFault(); a->hal().enable(true); }
      if (arm_) arm_->enable(true);  // also releases a latched stall
      snprintf(reply, rn, "ok");
      return;
    default:
      break;
  }
  if (estop) { snprintf(reply, rn, "err ESTOP"); return; }
  switch (r.cmd) {
    case Cmd::STOP:
      for (auto* a : axes_) a->abortHoming();
      jogging_ = false;
      home_queue_[AX_X] = home_queue_[AX_Z] = false;
      snprintf(reply, rn, "ok");
      return;
    case Cmd::HOME:
      // Z first (lift the arm clear), then X
      if (r.has[AX_Z]) {
        ax_z_.startHoming();
        home_queue_[AX_X] = r.has[AX_X];
        if (r.has[AX_X]) ax_x_.setHomed(false);
      } else {
        ax_x_.startHoming();
      }
      snprintf(reply, rn, "ok");
      return;
    case Cmd::JHOME:
      if (!arm_) { snprintf(reply, rn, "err NO_ARM"); return; }
      arm_->home();
      arm_homed_reported_ = false;
      snprintf(reply, rn, "ok");
      return;
    case Cmd::JMOVE: {
      if (!arm_) { snprintf(reply, rn, "err NO_ARM"); return; }
      int bad = -1;
      Arm::Err e = arm_->move(r.jhas, r.jval, r.has_feed ? r.feed : 0.5f, &bad);
      switch (e) {
        case Arm::Err::OK: snprintf(reply, rn, "ok"); return;
        case Arm::Err::NOT_HOMED: snprintf(reply, rn, "err NOT_HOMED J%d", bad + 1); return;
        case Arm::Err::LIMIT: snprintf(reply, rn, "err LIMIT J%d", bad + 1); return;
        case Arm::Err::BUSY: snprintf(reply, rn, "err BUSY"); return;
        default: snprintf(reply, rn, "err ARM_FAULT"); return;
      }
    }
    case Cmd::LIMITS:
      for (int i = 0; i < NUM_AXES; ++i)
        if (r.has[i] && (r.lo[i] < hard_[i].soft_min || r.hi[i] > hard_[i].soft_max)) {
          snprintf(reply, rn, "err LIMIT %c", AXIS_CH[i]);
          return;
        }
      for (int i = 0; i < NUM_AXES; ++i)
        if (r.has[i]) { axes_[i]->cfg().soft_min = r.lo[i]; axes_[i]->cfg().soft_max = r.hi[i]; }
      snprintf(reply, rn, "ok");
      return;
    case Cmd::MOVE: case Cmd::JOG: {
      if (state() == SysState::HOMING) { snprintf(reply, rn, "err BUSY"); return; }
      for (int i = 0; i < NUM_AXES; ++i) {
        if (!r.has[i]) continue;
        if (!axes_[i]->homed()) { snprintf(reply, rn, "err NOT_HOMED %c", AXIS_CH[i]); return; }
        if (r.cmd == Cmd::MOVE && !axes_[i]->withinSoft(r.val[i])) { snprintf(reply, rn, "err LIMIT %c", AXIS_CH[i]); return; }
      }
      if (r.cmd == Cmd::MOVE) {
        jogging_ = false;
        // coordinated: scale per-axis speed so both axes arrive together
        float d[NUM_AXES] = {0, 0}, dmax = 0;
        for (int i = 0; i < NUM_AXES; ++i)
          if (r.has[i]) {
            d[i] = r.val[i] - axes_[i]->hal().position();
            if (d[i] < 0) d[i] = -d[i];
            if (d[i] > dmax) dmax = d[i];
          }
        for (int i = 0; i < NUM_AXES; ++i) {
          if (!r.has[i]) continue;
          float vmax = axes_[i]->cfg().max_speed;
          float f = r.has_feed ? r.feed : 0.5f * vmax;
          float v = dmax > 0 ? f * d[i] / dmax : f;
          if (v > vmax) v = vmax;
          if (v < 0.5f) v = 0.5f;
          axes_[i]->hal().moveTo(r.val[i], v);
        }
      } else {
        for (int i = 0; i < NUM_AXES; ++i) {
          if (!r.has[i]) continue;
          float v = r.val[i];
          float vmax = axes_[i]->cfg().max_speed;
          if (v == 0) { axes_[i]->hal().stop(); continue; }
          float s = v < 0 ? -v : v;
          if (s > vmax) s = vmax;
          // jog = move to the soft limit in that direction; refreshed JOGs keep it going
          axes_[i]->hal().moveTo(v > 0 ? axes_[i]->cfg().soft_max : axes_[i]->cfg().soft_min, s);
        }
        jogging_ = true;
        last_jog_ms_ = now_ms_;
      }
      snprintf(reply, rn, "ok");
      return;
    }
    default:
      snprintf(reply, rn, "err UNKNOWN_CMD");
  }
}

bool Controller::update(uint32_t now_ms, char* ev, size_t en) {
  now_ms_ = now_ms;
  if (io_.estopPressed() && !estop_latched_) {
    estop_latched_ = true;
    for (auto* a : axes_) { a->abortHoming(); a->hal().forceStop(); a->hal().enable(false); }
    ax_x_.setHomed(false);
    jogging_ = false;
    home_queue_[AX_X] = home_queue_[AX_Z] = false;
    if (arm_) { arm_->estop(); arm_homed_reported_ = false; }
    snprintf(ev, en, "ev ESTOP");
    return true;
  }
  for (auto* a : axes_) a->update();
  if (arm_) {
    arm_->update(now_ms);
    for (int j = 0; j < NUM_JOINTS; ++j) {
      if (arm_->fault(j) && !arm_fault_reported_[j]) {
        arm_fault_reported_[j] = true;
        snprintf(ev, en, "ev %s J%d", arm_->stalled(j) ? "JSTALL" : arm_->commFault(j) ? "JCOMM" : "JHOME_FAIL", j + 1);
        return true;
      }
      if (!arm_->fault(j)) arm_fault_reported_[j] = false;
    }
    if (!arm_homed_reported_ && arm_->allHomed()) {
      arm_homed_reported_ = true;
      snprintf(ev, en, "ev JHOMED");
      return true;
    }
  }
  if (jogging_ && now_ms - last_jog_ms_ > JOG_WATCHDOG_MS) {
    jogging_ = false;
    for (auto* a : axes_) a->hal().stop();
    snprintf(ev, en, "ev JOG_TIMEOUT");
    return true;
  }
  for (int i = 0; i < NUM_AXES; ++i) {
    HomeState h = axes_[i]->homeState();
    if (h != last_hs_[i]) {
      last_hs_[i] = h;
      if (h == HomeState::DONE) {
        if (i == AX_Z && home_queue_[AX_X]) { home_queue_[AX_X] = false; ax_x_.startHoming(); }
        snprintf(ev, en, "ev HOMED %c", AXIS_CH[i]);
        return true;
      }
      if (h == HomeState::FAILED) {
        home_queue_[AX_X] = false;
        snprintf(ev, en, "ev HOME_FAIL %c", AXIS_CH[i]);
        return true;
      }
    }
    if (axes_[i]->limitFault() && !reported_fault_[i]) {
      reported_fault_[i] = true;
      snprintf(ev, en, "ev LIMIT_HIT %c", AXIS_CH[i]);
      return true;
    }
    if (!axes_[i]->limitFault()) reported_fault_[i] = false;
  }
  return false;
}

}  // namespace gantry
