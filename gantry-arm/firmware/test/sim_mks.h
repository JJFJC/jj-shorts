// Simulated MKS SERVO42D/57D drivers on a CAN bus, for the native tests and the Python bridge.
// Each driver answers like the real one (per the MKS CAN manual V1.0.9): F5H starts a move
// (reply status 1) and later reports done (status 2); 31H returns the int48 axis; 91H homes
// (status 1, then 2 once the switch is found); 3EH reports the stall flag; F7H stops at once.
#pragma once

#include <stdint.h>
#include <string.h>

#include <deque>

#include "arm_can.h"

namespace sim {

using gantry::mks::Frame;

struct Driver {
  uint16_t id = 1;
  double axis = 0;        // counts
  double target = 0;
  double rpm = 0;
  bool moving = false, homing = false, enabled = true, stalled = false;
  double home_switch_at = -5000;  // counts (machine) where the home switch closes
  double offset = 0;              // axis zero shift after homing
  bool block_at = false;          // simulate a collision: stalls when crossing block_axis
  double block_axis = 0;
  bool silent = false;            // simulate a dead driver / cable
};

struct Bus : gantry::CanPort {
  Driver drv[gantry::NUM_JOINTS];
  std::deque<Frame> to_host;
  std::deque<Frame> sent;  // everything the host sent (for tests)
  Bus() {
    for (int j = 0; j < gantry::NUM_JOINTS; ++j) drv[j].id = static_cast<uint16_t>(j + 1);
  }
  static Frame reply(uint16_t id, const uint8_t* p, uint8_t n) {
    Frame f;
    f.id = id;
    memcpy(f.d, p, n);
    f.d[n] = gantry::mks::crc(id, p, n);
    f.dlc = static_cast<uint8_t>(n + 1);
    return f;
  }
  Driver* find(uint16_t id) {
    for (auto& d : drv)
      if (d.id == id) return &d;
    return nullptr;
  }
  bool send(const Frame& f) override {
    sent.push_back(f);
    Driver* d = find(f.id);
    if (!d || d->silent || !gantry::mks::crcOk(f)) return true;
    uint8_t c = f.d[0];
    if (c == 0xF5) {
      if (d->stalled || !d->enabled) { const uint8_t p[2] = {0xF5, 0}; to_host.push_back(reply(d->id, p, 2)); return true; }
      int32_t a = (int32_t(f.d[4]) << 16) | (int32_t(f.d[5]) << 8) | f.d[6];
      if (a & 0x800000) a -= 0x1000000;
      d->target = a;
      d->rpm = (f.d[1] << 8) | f.d[2];
      d->moving = d->target != d->axis;
      const uint8_t p[2] = {0xF5, uint8_t(d->moving ? 1 : 2)};
      to_host.push_back(reply(d->id, p, 2));
    } else if (c == 0x31) {
      int64_t v = static_cast<int64_t>(d->axis - d->offset >= 0 ? d->axis - d->offset + 0.5 : d->axis - d->offset - 0.5);
      uint64_t u = static_cast<uint64_t>(v) & 0xFFFFFFFFFFFFULL;
      const uint8_t p[7] = {0x31, uint8_t(u >> 40), uint8_t(u >> 32), uint8_t(u >> 24), uint8_t(u >> 16), uint8_t(u >> 8), uint8_t(u)};
      to_host.push_back(reply(d->id, p, 7));
    } else if (c == 0x91) {
      d->homing = true;
      d->moving = false;
      const uint8_t p[2] = {0x91, 1};
      to_host.push_back(reply(d->id, p, 2));
    } else if (c == 0x3E) {
      const uint8_t p[2] = {0x3E, uint8_t(d->stalled ? 1 : 0)};
      to_host.push_back(reply(d->id, p, 2));
    } else if (c == 0x3D) {
      d->stalled = false;
    } else if (c == 0xF7) {
      d->moving = d->homing = false;
    } else if (c == 0xF3) {
      d->enabled = f.d[1] != 0;
      const uint8_t p[2] = {0xF3, 1};
      to_host.push_back(reply(d->id, p, 2));
    }
    return true;
  }
  bool receive(Frame& f) override {
    if (to_host.empty()) return false;
    f = to_host.front();
    to_host.pop_front();
    return true;
  }
  void tick(double dt) {
    for (auto& d : drv) {
      if (d.silent) continue;
      if (d.homing) {  // seek toward negative at a fixed speed until the switch closes
        d.axis -= 2000 * dt;
        if (d.axis <= d.home_switch_at) {
          d.homing = false;
          d.offset = d.axis;  // axis reads 0 on the switch
          const uint8_t p[2] = {0x91, 2};
          to_host.push_back(reply(d.id, p, 2));
        }
        continue;
      }
      if (!d.moving || d.stalled || !d.enabled) continue;
      double tgt = d.target + d.offset;
      double step = d.rpm * gantry::mks::COUNTS_PER_REV / 60.0 * dt;
      double prev = d.axis;
      if (tgt > d.axis) d.axis = (d.axis + step >= tgt) ? tgt : d.axis + step;
      else d.axis = (d.axis - step <= tgt) ? tgt : d.axis - step;
      if (d.block_at) {
        double b = d.block_axis + d.offset;
        if ((prev < b && d.axis >= b) || (prev > b && d.axis <= b)) { d.axis = b; d.stalled = true; d.moving = false; continue; }
      }
      if (d.axis == tgt) {
        d.moving = false;
        const uint8_t p[2] = {0xF5, 2};
        to_host.push_back(reply(d.id, p, 2));
      }
    }
  }
};

}  // namespace sim
