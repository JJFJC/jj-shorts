// Native tests for the controller core. Only the hardware boundary (AxisHal / IoHal) is faked:
// FakeAxis integrates motion at the commanded speed and closes its limit switch below a
// configurable position, like a real carriage running into a microswitch.
#include <string.h>
#include <unity.h>

#include "gantry_core.h"

using namespace gantry;

struct FakeAxis : AxisHal {
  float pos = 0, target = 0, speed = 0;
  bool run = false, en = true;
  float switch_min_at = -1e9f;  // switch closes when pos <= this (machine coordinates)
  float switch_max_at = 1e9f;
  float offset = 0;  // setPosition() shifts reported coordinates
  bool stuck_min = false;
  void moveTo(float mm, float s) override { target = mm; speed = s; run = (mm != pos); }
  void stop() override { run = false; }
  void forceStop() override { run = false; }
  float position() const override { return pos; }
  void setPosition(float mm) override { offset += mm - pos; pos = mm; target = mm; }
  bool running() const override { return run; }
  bool limitMin() const override { return stuck_min || (pos - offset) <= switch_min_at; }
  bool limitMax() const override { return (pos - offset) >= switch_max_at; }
  void enable(bool on) override { en = on; }
  void tick(float dt) {
    if (!run || !en) return;
    float step = speed * dt;
    if (target > pos) { pos = (pos + step >= target) ? target : pos + step; }
    else { pos = (pos - step <= target) ? target : pos - step; }
    if (pos == target) run = false;
  }
};

struct FakeIo : IoHal {
  bool estop = false;
  int tool = 4095;
  bool estopPressed() const override { return estop; }
  int toolIdRaw() const override { return tool; }
};

static AxisConfig cfgX() {
  AxisConfig c;
  c.soft_min = 0; c.soft_max = 800; c.max_speed = 200; c.home_fast = 40; c.home_slow = 5;
  c.home_backoff = 3; c.home_offset = 2; c.home_search = 900;
  return c;
}
static AxisConfig cfgZ() {
  AxisConfig c;
  c.soft_min = 0; c.soft_max = 500; c.max_speed = 20; c.home_fast = 15; c.home_slow = 3;
  c.home_backoff = 2; c.home_offset = 1; c.home_search = 600;
  return c;
}

struct Rig {
  FakeAxis x, z;
  FakeIo io;
  Controller c{x, z, io, cfgX(), cfgZ()};
  uint32_t t = 0;
  char reply[kReplyMax];
  char ev[kReplyMax];
  char last_ev[8][kReplyMax];
  int n_ev = 0;
  Rig() {
    x.pos = 350; z.pos = 220;   // power-on somewhere in the middle
    x.switch_min_at = 0; z.switch_min_at = 0;
  }
  const char* send(const char* line) { c.handleLine(line, reply, sizeof reply); return reply; }
  void run(float seconds, float dt = 0.005f) {
    for (float s = 0; s < seconds; s += dt) {
      x.tick(dt); z.tick(dt);
      t += (uint32_t)(dt * 1000);
      if (c.update(t, ev, sizeof ev) && n_ev < 8) strcpy(last_ev[n_ev++], ev);
    }
  }
  bool sawEvent(const char* e) const {
    for (int i = 0; i < n_ev; ++i) if (strcmp(last_ev[i], e) == 0) return true;
    return false;
  }
  void homeAll() { send("HOME"); run(120); }
};

extern "C" void setUp(void) {}
extern "C" void tearDown(void) {}

// ------------------------------------------------------------------ parser

void test_parse_move_with_feed() {
  Request r;
  TEST_ASSERT_EQUAL(ParseErr::OK, parseLine("move x100.5 z-2 f50", r));
  TEST_ASSERT_TRUE(r.cmd == Cmd::MOVE);
  TEST_ASSERT_TRUE(r.has[AX_X] && r.has[AX_Z] && r.has_feed);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 100.5f, r.val[AX_X]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, -2.0f, r.val[AX_Z]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 50.0f, r.feed);
}

void test_parse_rejects_garbage() {
  Request r;
  TEST_ASSERT_EQUAL(ParseErr::EMPTY, parseLine("   \r\n", r));
  TEST_ASSERT_EQUAL(ParseErr::UNKNOWN_CMD, parseLine("FLY X1", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("MOVE", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("MOVE X1 X2", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("MOVE Xabc", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("MOVE Y10", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("MOVE X1 F-3", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("JOG X1 F3", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("STATUS now", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("LIMITS X5,1", r));
  TEST_ASSERT_EQUAL(ParseErr::BAD_ARG, parseLine("LED PURPLE", r));
  char longline[200];
  memset(longline, 'A', sizeof longline - 1);
  longline[sizeof longline - 1] = 0;
  TEST_ASSERT_EQUAL(ParseErr::TOO_LONG, parseLine(longline, r));
}

void test_parse_checksum() {
  Request r;
  const char* body = "MOVE X10 Z5";
  char line[64];
  snprintf(line, sizeof line, "%s*%02X", body, checksum(body, strlen(body)));
  TEST_ASSERT_EQUAL(ParseErr::OK, parseLine(line, r));
  line[strlen(line) - 1] = (line[strlen(line) - 1] == '0') ? '1' : '0';
  TEST_ASSERT_EQUAL(ParseErr::CHECKSUM, parseLine(line, r));
  TEST_ASSERT_EQUAL(ParseErr::CHECKSUM, parseLine("PING*Z", r));
}

void test_parse_home_axes_and_limits() {
  Request r;
  TEST_ASSERT_EQUAL(ParseErr::OK, parseLine("HOME Z", r));
  TEST_ASSERT_TRUE(!r.has[AX_X] && r.has[AX_Z]);
  TEST_ASSERT_EQUAL(ParseErr::OK, parseLine("HOME", r));
  TEST_ASSERT_TRUE(r.has[AX_X] && r.has[AX_Z]);
  TEST_ASSERT_EQUAL(ParseErr::OK, parseLine("LIMITS X10,700 Z0,450", r));
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 10, r.lo[AX_X]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4, 450, r.hi[AX_Z]);
}

// ------------------------------------------------------------------ homing state machine

void test_home_all_z_first_then_x() {
  Rig g;
  TEST_ASSERT_EQUAL_STRING("ok", g.send("HOME"));
  g.run(0.05);
  TEST_ASSERT_TRUE(g.z.run);   // Z starts first
  TEST_ASSERT_FALSE(g.x.run);  // X waits for Z
  g.run(120);
  TEST_ASSERT_TRUE(g.sawEvent("ev HOMED Z"));
  TEST_ASSERT_TRUE(g.sawEvent("ev HOMED X"));
  TEST_ASSERT_TRUE(g.c.axis(AX_X).homed() && g.c.axis(AX_Z).homed());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 0.0f, g.x.position());
  // logical 0 sits home_offset above the switch, so the switch is released
  TEST_ASSERT_FALSE(g.x.limitMin());
  TEST_ASSERT_EQUAL_STRING("IDLE", stateName(g.c.state()));
}

void test_home_starting_on_switch_backs_off_first() {
  Rig g;
  g.x.pos = -0.5f;  // parked on the switch
  g.send("HOME X");
  g.run(0.02);
  TEST_ASSERT_TRUE(g.x.target > g.x.pos);  // first move is away from the switch
  g.run(60);
  TEST_ASSERT_TRUE(g.c.axis(AX_X).homed());
}

void test_home_fails_when_switch_never_closes() {
  Rig g;
  g.z.switch_min_at = -1e9f;  // broken / unplugged switch
  g.send("HOME Z");
  g.run(80);
  TEST_ASSERT_TRUE(g.sawEvent("ev HOME_FAIL Z"));
  TEST_ASSERT_FALSE(g.c.axis(AX_Z).homed());
  TEST_ASSERT_EQUAL_STRING("FAULT", stateName(g.c.state()));
}

void test_home_fails_when_switch_stuck_closed() {
  Rig g;
  g.x.stuck_min = true;
  g.send("HOME X");
  g.run(30);
  TEST_ASSERT_TRUE(g.sawEvent("ev HOME_FAIL X"));
}

// ------------------------------------------------------------------ soft limits, moves

void test_move_requires_homing() {
  Rig g;
  TEST_ASSERT_EQUAL_STRING("err NOT_HOMED X", g.send("MOVE X100"));
}

void test_soft_limits_reject_out_of_range() {
  Rig g;
  g.homeAll();
  TEST_ASSERT_EQUAL_STRING("err LIMIT X", g.send("MOVE X801"));
  TEST_ASSERT_EQUAL_STRING("err LIMIT Z", g.send("MOVE X10 Z-0.5"));
  TEST_ASSERT_FALSE(g.x.run);  // a rejected command must not move either axis
  TEST_ASSERT_EQUAL_STRING("ok", g.send("MOVE X800 Z500"));
}

void test_coordinated_move_arrives_together() {
  Rig g;
  g.homeAll();
  g.send("MOVE X400 Z100 F100");
  // X would do 400 mm at 100 mm/s; Z is capped at 20 mm/s -> both must still end together-ish
  TEST_ASSERT_FLOAT_WITHIN(0.01, 100.0f, g.x.speed);
  TEST_ASSERT_FLOAT_WITHIN(0.01, 20.0f, g.z.speed);
  g.run(6);
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 400, g.x.position());
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 100, g.z.position());
}

void test_limits_command_only_narrows() {
  Rig g;
  g.homeAll();
  TEST_ASSERT_EQUAL_STRING("err LIMIT X", g.send("LIMITS X-5,700"));
  TEST_ASSERT_EQUAL_STRING("ok", g.send("LIMITS X50,700"));
  TEST_ASSERT_EQUAL_STRING("err LIMIT X", g.send("MOVE X20"));
}

void test_jog_stops_at_soft_limit_and_watchdog() {
  Rig g;
  g.homeAll();
  g.send("LIMITS X0,30");
  for (int i = 0; i < 40; ++i) { g.send("JOG X50"); g.run(0.1); }  // keep refreshing
  TEST_ASSERT_FLOAT_WITHIN(1e-3, 30, g.x.position());              // parked on the soft limit
  g.send("MOVE X0");
  g.run(2);
  g.send("JOG X20");
  g.run(1.0);  // no refresh -> watchdog
  TEST_ASSERT_TRUE(g.sawEvent("ev JOG_TIMEOUT"));
  TEST_ASSERT_TRUE(g.x.position() < 8.0f);  // stopped ~0.3 s after the last JOG
}

void test_overtravel_switch_faults_axis() {
  Rig g;
  g.homeAll();
  g.x.switch_max_at = 300;  // a bumped switch earlier than the soft limit
  g.send("MOVE X600");
  g.run(10);
  TEST_ASSERT_TRUE(g.sawEvent("ev LIMIT_HIT X"));
  TEST_ASSERT_FALSE(g.c.axis(AX_X).homed());
  TEST_ASSERT_EQUAL_STRING("err NOT_HOMED X", g.send("MOVE X10"));
}

// ------------------------------------------------------------------ e-stop

void test_hardware_estop_latches_until_reset() {
  Rig g;
  g.homeAll();
  g.send("MOVE X500");
  g.run(0.5);
  g.io.estop = true;
  g.run(0.05);
  TEST_ASSERT_TRUE(g.sawEvent("ev ESTOP"));
  TEST_ASSERT_FALSE(g.x.run);
  TEST_ASSERT_FALSE(g.x.en);
  TEST_ASSERT_EQUAL_STRING("err ESTOP", g.send("MOVE X10"));
  TEST_ASSERT_EQUAL_STRING("err ESTOP_HW", g.send("RESET"));  // button still pressed
  g.io.estop = false;
  TEST_ASSERT_EQUAL_STRING("err ESTOP", g.send("HOME"));      // still latched
  TEST_ASSERT_EQUAL_STRING("ok", g.send("RESET"));
  TEST_ASSERT_FALSE(g.c.axis(AX_X).homed());  // X lost position, Z (self-locking) kept it
  TEST_ASSERT_TRUE(g.c.axis(AX_Z).homed());
}

void test_status_and_led_and_tool_id() {
  Rig g;
  g.io.tool = 380;  // 1k resistor -> gripper
  g.homeAll();
  TEST_ASSERT_EQUAL_STRING("ok TOOL 1", g.send("TOOL"));
  g.send("STATUS");
  TEST_ASSERT_EQUAL_STRING("st S=IDLE X=0.000 Z=0.000 H=XZ L=---- T=1", g.reply);
  TEST_ASSERT_TRUE(g.c.led() == Led::IDLE);
  g.send("LED FAULT");  // PC-side fault (e.g. collision detected) lights the arm red
  TEST_ASSERT_TRUE(g.c.led() == Led::FAULT);
  g.send("LED IDLE");
  g.send("MOVE X100");
  TEST_ASSERT_TRUE(g.c.led() == Led::MOVING);
  g.io.tool = 4095;
  TEST_ASSERT_EQUAL_STRING("ok TOOL 0", g.send("TOOL"));
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_parse_move_with_feed);
  RUN_TEST(test_parse_rejects_garbage);
  RUN_TEST(test_parse_checksum);
  RUN_TEST(test_parse_home_axes_and_limits);
  RUN_TEST(test_home_all_z_first_then_x);
  RUN_TEST(test_home_starting_on_switch_backs_off_first);
  RUN_TEST(test_home_fails_when_switch_never_closes);
  RUN_TEST(test_home_fails_when_switch_stuck_closed);
  RUN_TEST(test_move_requires_homing);
  RUN_TEST(test_soft_limits_reject_out_of_range);
  RUN_TEST(test_coordinated_move_arrives_together);
  RUN_TEST(test_limits_command_only_narrows);
  RUN_TEST(test_jog_stops_at_soft_limit_and_watchdog);
  RUN_TEST(test_overtravel_switch_faults_axis);
  RUN_TEST(test_hardware_estop_latches_until_reset);
  RUN_TEST(test_status_and_led_and_tool_id);
  return UNITY_END();
}
