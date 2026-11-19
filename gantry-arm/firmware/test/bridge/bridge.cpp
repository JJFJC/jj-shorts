// C ABI around the real controller core + simulated axes, so the Python driver can be
// contract-tested against the actual firmware logic (no hand-written protocol mock).
// Built on the fly by software/lerobot_robot_gantry_arm/tests/firmware_bridge.py.
#include <string.h>

#include "gantry_core.h"

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
struct Board {
  SimAxis x, z;
  SimIo io;
  Controller c{x, z, io, cx(), cz()};
  uint32_t t = 0;
  Board() { x.pos = 350; z.pos = 220; }
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
  b->t += (uint32_t)(dt * 1000.0f);
  return b->c.update(b->t, ev, (size_t)n) ? 1 : 0;
}
void gb_set_estop(void* b, int on) { static_cast<Board*>(b)->io.estop = on != 0; }
void gb_set_tool_raw(void* b, int raw) { static_cast<Board*>(b)->io.tool = raw; }
}
