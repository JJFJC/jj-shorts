// C ABI around the real controller core + simulated axes, so the Python driver can be
// contract-tested against the actual firmware logic (no hand-written protocol mock).
// Built on the fly by software/lerobot_robot_gantry_arm/tests/firmware_bridge.py.
#include <string.h>

#include "arm_can.h"
#include "gantry_core.h"
#include "../sim_mks.h"

using namespace gantry;

namespace {
struct SimAxis : AxisHal {
  float pos = 0, target = 0, speed = 0, offset = 0;
  bool run = false, en = true;
  float switch_min_at = 0.0f, switch_max_at = 1e9f;
  void moveTo(float mm, float s) override { target = mm; speed = s; run = (mm != pos); }
  void stop() override { run = false; }
  void forceStop() override { run = false; }
  float position() const override { return pos; }
  void setPosition(float mm) override { offset += mm - pos; pos = mm; target = mm; }
  bool running() const override { return run; }
  bool limitMin() const override { return (pos - offset) <= switch_min_at; }
  bool limitMax() const override { return (pos - offset) >= switch_max_at; }
  void enable(bool on) override { en = on; }
  void tick(float dt) {
    if (!run || !en) return;
    float step = speed * dt;
    if (target > pos) pos = (pos + step >= target) ? target : pos + step;
    else pos = (pos - step <= target) ? target : pos - step;
    if (pos == target) run = false;
  }
};
struct SimIo : IoHal {
  bool estop = false;
  int tool = 4095;
  bool estopPressed() const override { return estop; }
  int toolIdRaw() const override { return tool; }
};
AxisConfig cx() {
  AxisConfig c; c.soft_min = 0; c.soft_max = 800; c.max_speed = 200; c.home_fast = 40; c.home_slow = 5;
  c.home_backoff = 3; c.home_offset = 2; c.home_search = 900; return c;
}
AxisConfig cz() {
  AxisConfig c; c.soft_min = 0; c.soft_max = 500; c.max_speed = 20; c.home_fast = 15; c.home_slow = 3;
  c.home_backoff = 2; c.home_offset = 1; c.home_search = 600; return c;
}
const JointConfig* joints() {  // same table as src/main.cpp
  static JointConfig j[NUM_JOINTS];
  const float ratio[NUM_JOINTS] = {10, 20, 25, 15, 5};
  const float lo[NUM_JOINTS] = {-150, -100, -140, -110, -180};
  const float hi[NUM_JOINTS] = {150, 135, 140, 110, 180};
  for (int i = 0; i < NUM_JOINTS; ++i) {
    j[i].can_id = static_cast<uint16_t>(i + 1);
    j[i].counts_per_deg = mks::COUNTS_PER_REV * ratio[i] / 360.0f;
    j[i].min_deg = lo[i]; j[i].max_deg = hi[i]; j[i].home_deg = lo[i];
    j[i].max_rpm = 600; j[i].acc = 20;
  }
  return j;
}
struct Board {
  SimAxis x, z;
  SimIo io;
  sim::Bus bus;
  Arm arm{bus, joints()};
  Controller c{x, z, io, cx(), cz()};
  uint32_t t = 0;
  Board() { x.pos = 350; z.pos = 220; c.attachArm(&arm); }
};
}  // namespace

extern "C" {
void* gb_create() { return new Board(); }
void gb_destroy(void* b) { delete static_cast<Board*>(b); }
void gb_handle(void* b, const char* line, char* out, int n) { static_cast<Board*>(b)->c.handleLine(line, out, (size_t)n); }
// advance simulated time; returns 1 and fills `ev` if an event fired
int gb_tick(void* bp, float dt, char* ev, int n) {
  Board* b = static_cast<Board*>(bp);
  b->x.tick(dt); b->z.tick(dt);
  // the CAN side runs in 5 ms steps (poll period 20 ms); report the first event of this tick
  int got = 0;
  uint32_t steps = (uint32_t)(dt * 1000.0f) / 5;
  if (steps == 0) steps = 1;
  for (uint32_t i = 0; i < steps; ++i) {
    b->bus.tick(0.005);
    b->t += 5;
    if (!got && b->c.update(b->t, ev, (size_t)n)) got = 1;
    else if (got) { char tmp[160]; b->c.update(b->t, tmp, sizeof(tmp)); }
  }
  return got;
}
void gb_set_estop(void* b, int on) { static_cast<Board*>(b)->io.estop = on != 0; }
void gb_set_tool_raw(void* b, int raw) { static_cast<Board*>(b)->io.tool = raw; }
// put an obstacle in a joint's way at `deg` (joint angle): the simulated driver stalls there
void gb_arm_block(void* bp, int joint, float deg) {
  Board* b = static_cast<Board*>(bp);
  const JointConfig& c = b->arm.cfg(joint);
  b->bus.drv[joint].block_at = true;
  b->bus.drv[joint].block_axis = c.sign * (deg - c.home_deg) * c.counts_per_deg;
}
void gb_arm_unblock(void* bp, int joint) { static_cast<Board*>(bp)->bus.drv[joint].block_at = false; }
}
