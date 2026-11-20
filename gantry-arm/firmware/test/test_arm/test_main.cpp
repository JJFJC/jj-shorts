// Native tests for the CAN arm layer (arm_can.h) against simulated MKS drivers (sim_mks.h).
#include <stdio.h>
#include <string.h>
#include <unity.h>

#include "arm_can.h"
#include "gantry_core.h"
#include "../sim_mks.h"

using namespace gantry;

void setUp() {}
void tearDown() {}

// joint table like the v2 arm: total ratios 10 (belt), 20, 25, 15, 5
static void joints(JointConfig* c) {
  const float ratio[NUM_JOINTS] = {10, 20, 25, 15, 5};
  const float lo[NUM_JOINTS] = {-150, -100, -140, -110, -180};
  const float hi[NUM_JOINTS] = {150, 135, 140, 110, 180};
  for (int j = 0; j < NUM_JOINTS; ++j) {
    c[j] = JointConfig();
    c[j].can_id = static_cast<uint16_t>(j + 1);
    c[j].counts_per_deg = mks::COUNTS_PER_REV * ratio[j] / 360.0f;
    c[j].min_deg = lo[j];
    c[j].max_deg = hi[j];
    c[j].home_deg = lo[j];  // home switches at the negative end
    c[j].max_rpm = 600;
  }
}

struct Rig {
  sim::Bus bus;
  JointConfig cfg[NUM_JOINTS];
  Arm* arm;
  uint32_t t = 0;
  Rig() {
    joints(cfg);
    arm = new Arm(bus, cfg);
  }
  ~Rig() { delete arm; }
  void run(double seconds) {
    for (double s = 0; s < seconds; s += 0.005) {
      bus.tick(0.005);
      t += 5;
      arm->update(t);
    }
  }
  void homeAll() {
    arm->home();
    run(4.0);
  }
};

// ---------------------------------------------------------------- codec (examples from the MKS CAN manual V1.0.9)

static void test_frames_match_the_manual_examples() {
  mks::Frame f = mks::absAxis(1, 0x4000, 600, 2);  // manual 11.4.1: "01 F5 02 58 02 00 40 00 92"
  const uint8_t want[8] = {0xF5, 0x02, 0x58, 0x02, 0x00, 0x40, 0x00, 0x92};
  TEST_ASSERT_EQUAL_UINT8(8, f.dlc);
  TEST_ASSERT_EQUAL_UINT8_ARRAY(want, f.d, 8);
  mks::Frame g = mks::absAxis(1, -0x4000, 600, 2);  // "01 F5 02 58 02 FF C0 00 11"
  const uint8_t want2[8] = {0xF5, 0x02, 0x58, 0x02, 0xFF, 0xC0, 0x00, 0x11};
  TEST_ASSERT_EQUAL_UINT8_ARRAY(want2, g.d, 8);
  mks::Frame r = mks::readAxis(1);  // manual 5.1: CRC = (0x01 + 0x30)... for 31H: 0x32
  TEST_ASSERT_EQUAL_UINT8(2, r.dlc);
  TEST_ASSERT_EQUAL_HEX8(0x32, r.d[1]);
  TEST_ASSERT_EQUAL_HEX8(0xF8, mks::estop(1).d[1]);  // manual: "01 F7 F8"
}

static void test_decode_int48_axis_and_reject_bad_crc() {
  uint8_t p[7] = {0x31, 0xFF, 0xFF, 0xFF, 0xFF, 0xC0, 0x00};  // -0x4000
  mks::Frame f;
  f.id = 3;
  memcpy(f.d, p, 7);
  f.d[7] = mks::crc(3, p, 7);
  f.dlc = 8;
  mks::Reply r = mks::decode(f);
  TEST_ASSERT_TRUE(r.kind == mks::Kind::AXIS);
  TEST_ASSERT_TRUE(r.axis == -0x4000);
  f.d[7] ^= 0x55;
  TEST_ASSERT_TRUE(mks::decode(f).kind == mks::Kind::BAD_CRC);
}

// ---------------------------------------------------------------- arm

static void test_move_needs_homing_and_homing_zeroes_at_the_switch() {
  Rig g;
  bool has[NUM_JOINTS] = {false, true, false, false, false};
  float deg[NUM_JOINTS] = {0, 10, 0, 0, 0};
  int bad = -1;
  TEST_ASSERT_TRUE(g.arm->move(has, deg, 0.5f, &bad) == Arm::Err::NOT_HOMED);
  TEST_ASSERT_EQUAL_INT(1, bad);
  g.homeAll();
  TEST_ASSERT_TRUE(g.arm->allHomed());
  TEST_ASSERT_FLOAT_WITHIN(0.01f, -100.0f, g.arm->deg(1));  // reads the switch angle after homing
}

static void test_coordinated_move_arrives_together_and_reports_degrees() {
  Rig g;
  g.homeAll();
  bool has[NUM_JOINTS] = {true, true, true, true, true};
  float deg[NUM_JOINTS] = {30, 45, 60, -20, 90};
  int bad = -1;
  TEST_ASSERT_TRUE(g.arm->move(has, deg, 0.5f, &bad) == Arm::Err::OK);
  // all joints started; run until the first one finishes and check the others are nearly done
  double t_end[NUM_JOINTS] = {0};
  for (double s = 0; s < 30.0; s += 0.005) {
    g.bus.tick(0.005);
    g.t += 5;
    g.arm->update(g.t);
    for (int j = 0; j < NUM_JOINTS; ++j)
      if (t_end[j] == 0 && !g.bus.drv[j].moving) t_end[j] = s;
    bool all = true;
    for (int j = 0; j < NUM_JOINTS; ++j) all = all && t_end[j] > 0;
    if (all) break;
  }
  double lo = 1e9, hi = 0;
  for (int j = 0; j < NUM_JOINTS; ++j) { lo = t_end[j] < lo ? t_end[j] : lo; hi = t_end[j] > hi ? t_end[j] : hi; }
  TEST_ASSERT_TRUE(hi - lo < 0.05 * hi + 0.02);  // within 5 % of the move time
  g.run(0.1);
  for (int j = 0; j < NUM_JOINTS; ++j) TEST_ASSERT_FLOAT_WITHIN(0.01f, deg[j], g.arm->deg(j));
  TEST_ASSERT_FALSE(g.arm->moving());
}

static void test_soft_limits_reject_the_whole_move() {
  Rig g;
  g.homeAll();
  size_t sent_before = g.bus.sent.size();
  bool has[NUM_JOINTS] = {true, true, false, false, false};
  float deg[NUM_JOINTS] = {10, 150, 0, 0, 0};  // J2 max is 135
  int bad = -1;
  TEST_ASSERT_TRUE(g.arm->move(has, deg, 0.5f, &bad) == Arm::Err::LIMIT);
  TEST_ASSERT_EQUAL_INT(1, bad);
  for (size_t i = sent_before; i < g.bus.sent.size(); ++i) TEST_ASSERT_NOT_EQUAL(0xF5, g.bus.sent[i].d[0]);  // nothing moved
}

static void test_stall_is_reported_as_a_fault_and_cleared_by_enable() {
  Rig g;
  g.homeAll();
  g.bus.drv[2].block_at = true;  // something in the elbow's way at -100 deg
  g.bus.drv[2].block_axis = (-100.0 - -140.0) * g.cfg[2].counts_per_deg;
  bool has[NUM_JOINTS] = {false, false, true, false, false};
  float deg[NUM_JOINTS] = {0, 0, 0, 0, 0};
  int bad = -1;
  TEST_ASSERT_TRUE(g.arm->move(has, deg, 1.0f, &bad) == Arm::Err::OK);
  g.run(3.0);
  TEST_ASSERT_TRUE(g.arm->stalled(2));
  TEST_ASSERT_TRUE(g.arm->anyFault());
  TEST_ASSERT_TRUE(g.arm->move(has, deg, 1.0f, &bad) == Arm::Err::FAULT);  // refuses until cleared
  g.bus.drv[2].block_at = false;
  g.arm->enable(true);
  g.run(0.1);
  TEST_ASSERT_FALSE(g.arm->anyFault());
}

static void test_silent_driver_becomes_a_comm_fault() {
  Rig g;
  g.homeAll();
  g.bus.drv[4].silent = true;
  g.run(0.5);
  TEST_ASSERT_TRUE(g.arm->commFault(4));
  TEST_ASSERT_FALSE(g.arm->commFault(0));
}

static void test_estop_broadcasts_and_forgets_homing() {
  Rig g;
  g.homeAll();
  g.arm->estop();
  int f7 = 0;
  for (auto& f : g.bus.sent)
    if (f.d[0] == 0xF7) ++f7;
  TEST_ASSERT_EQUAL_INT(NUM_JOINTS, f7);
  TEST_ASSERT_FALSE(g.arm->allHomed());
}

// ---------------------------------------------------------------- controller line protocol with the arm attached

struct NullAxis : AxisHal {
  float pos = 0;
  bool en = true;
  void moveTo(float mm, float) override { pos = mm; }
  void stop() override {}
  void forceStop() override {}
  float position() const override { return pos; }
  void setPosition(float mm) override { pos = mm; }
  bool running() const override { return false; }
  bool limitMin() const override { return false; }
  bool limitMax() const override { return false; }
  void enable(bool on) override { en = on; }
};
struct NullIo : IoHal {
  bool estop = false;
  bool estopPressed() const override { return estop; }
  int toolIdRaw() const override { return 4095; }
};

struct Board {
  NullAxis x, z;
  NullIo io;
  Controller c{x, z, io, AxisConfig(), AxisConfig()};
  Rig r;
  char reply[kReplyMax], ev[kReplyMax];
  Board() { c.attachArm(r.arm); }
  const char* send(const char* line) { c.handleLine(line, reply, sizeof(reply)); return reply; }
  // run the bus + controller; returns the last event seen (or "")
  const char* run(double seconds, const char* want = nullptr) {
    static char last[kReplyMax];
    last[0] = 0;
    for (double s = 0; s < seconds; s += 0.005) {
      r.bus.tick(0.005);
      r.t += 5;
      while (c.update(r.t, ev, sizeof(ev))) {
        strncpy(last, ev, sizeof(last) - 1);
        if (want && strstr(ev, want)) return last;
      }
    }
    return last;
  }
};

static void test_parse_jmove() {
  Request q;
  TEST_ASSERT_TRUE(parseLine("JMOVE J1 10 J2-5", q) == ParseErr::BAD_ARG);  // value must follow the joint
  TEST_ASSERT_TRUE(parseLine("jmove j1-12.5 j3 40 f0.3", q) == ParseErr::BAD_ARG);
  TEST_ASSERT_TRUE(parseLine("jmove j1-12.5 j340 f0.3", q) == ParseErr::OK);
  TEST_ASSERT_TRUE(q.jhas[0] && q.jhas[2] && !q.jhas[1]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, -12.5f, q.jval[0]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 40.0f, q.jval[2]);
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.3f, q.feed);
  TEST_ASSERT_TRUE(parseLine("JMOVE J6 10", q) == ParseErr::BAD_ARG);
  TEST_ASSERT_TRUE(parseLine("JMOVE J1 10 F1.5", q) == ParseErr::BAD_ARG);
}

static void test_controller_jhome_jmove_jpos() {
  Board b;
  TEST_ASSERT_EQUAL_STRING("err NOT_HOMED J2", b.send("JMOVE J20"));
  TEST_ASSERT_EQUAL_STRING("ok", b.send("JHOME"));
  TEST_ASSERT_EQUAL_STRING("ev JHOMED", b.run(5.0, "JHOMED"));
  TEST_ASSERT_EQUAL_STRING("ok", b.send("JMOVE J10 J20 J30 J40 J50 F0.5"));
  b.run(8.0);
  b.send("JPOS");
  float a[NUM_JOINTS];
  char h[8], f[8];
  TEST_ASSERT_EQUAL_INT(7, sscanf(b.reply, "ok JPOS %f %f %f %f %f H=%7s F=%7s", &a[0], &a[1], &a[2], &a[3], &a[4], h, f));
  for (int j = 0; j < NUM_JOINTS; ++j) TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.0f, a[j]);  // within ~1 encoder count
  TEST_ASSERT_EQUAL_STRING("12345", h);
  TEST_ASSERT_EQUAL_STRING("-----", f);
  TEST_ASSERT_EQUAL_STRING("err LIMIT J2", b.send("JMOVE J2150"));
}

static void test_controller_stall_faults_estop_and_reset() {
  Board b;
  b.send("JHOME");
  b.run(5.0, "JHOMED");
  b.r.bus.drv[2].block_at = true;
  b.r.bus.drv[2].block_axis = 20.0 * b.r.cfg[2].counts_per_deg;  // 20 deg past the switch
  TEST_ASSERT_EQUAL_STRING("ok", b.send("JMOVE J30"));
  TEST_ASSERT_EQUAL_STRING("ev JSTALL J3", b.run(5.0, "JSTALL"));
  TEST_ASSERT_TRUE(b.c.state() == SysState::FAULT);
  TEST_ASSERT_EQUAL_STRING("err ARM_FAULT", b.send("JMOVE J30"));
  b.r.bus.drv[2].block_at = false;
  TEST_ASSERT_EQUAL_STRING("ok", b.send("RESET"));
  b.run(0.1);
  TEST_ASSERT_TRUE(b.c.state() != SysState::FAULT);
  TEST_ASSERT_EQUAL_STRING("ok", b.send("ESTOP"));
  TEST_ASSERT_EQUAL_STRING("err ESTOP", b.send("JMOVE J30"));
  b.send("RESET");
  TEST_ASSERT_EQUAL_STRING("err NOT_HOMED J3", b.send("JMOVE J30"));  // home again after an e-stop
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_frames_match_the_manual_examples);
  RUN_TEST(test_decode_int48_axis_and_reject_bad_crc);
  RUN_TEST(test_move_needs_homing_and_homing_zeroes_at_the_switch);
  RUN_TEST(test_coordinated_move_arrives_together_and_reports_degrees);
  RUN_TEST(test_soft_limits_reject_the_whole_move);
  RUN_TEST(test_stall_is_reported_as_a_fault_and_cleared_by_enable);
  RUN_TEST(test_silent_driver_becomes_a_comm_fault);
  RUN_TEST(test_estop_broadcasts_and_forgets_homing);
  RUN_TEST(test_parse_jmove);
  RUN_TEST(test_controller_jhome_jmove_jpos);
  RUN_TEST(test_controller_stall_faults_estop_and_reset);
  return UNITY_END();
}
