// Gantry Arm gantry controller (ESP32 DevKit v1).
// Hardware glue only: steppers (FastAccelStepper), TMC2209 over UART (TMCStepper),
// limit switches, e-stop loop, TOOL_ID ADC and WS2812 status LEDs.
// All logic (protocol, homing, soft limits, e-stop latch) is in lib/gantry_core and tested natively.
//
// Line protocol over USB serial @115200: see ../docs/PROTOCOL.md
#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <FastAccelStepper.h>
#include <TMCStepper.h>

#include "gantry_core.h"
#include "driver/twai.h"  // ESP32 built-in CAN controller (+ SN65HVD230 3.3 V transceiver)

using namespace gantry;

// ------------------------------------------------------------------ pins (ESP32 DevKit v1)
constexpr int PIN_X_STEP = 25, PIN_X_DIR = 26;
constexpr int PIN_Z_STEP = 27, PIN_Z_DIR = 14;
constexpr int PIN_EN = 13;                       // shared TMC2209 EN, active low
constexpr int PIN_TMC_RX = 16, PIN_TMC_TX = 17;  // one UART bus, addresses 0 (X) and 1 (Z)
constexpr int PIN_X_MIN = 32, PIN_X_MAX = 33;    // NC switches to GND, internal pull-up
constexpr int PIN_Z_MIN = 34, PIN_Z_MAX = 35;    // input-only pins: external 10k pull-ups
constexpr int PIN_ESTOP = 39;                    // NC e-stop loop to GND; open (HIGH) = pressed
constexpr int PIN_TOOL_ID = 36;                  // ADC1_CH0 (VP), 10k pull-up to 3.3 V on the PCB
constexpr int PIN_LED = 4;                       // WS2812 chain: 6 on the upper arm (+24 head rings if fitted)
constexpr int NUM_LEDS = 30;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_21, PIN_CAN_RX = GPIO_NUM_22;  // arm v2: MKS SERVO42D/57D bus, 500 kbit/s

// ------------------------------------------------------------------ mechanics
constexpr float X_STEPS_PER_MM = 200.0f * 16 / (20 * 2.0f);  // GT2 20T pulley, 1/16 microstep -> 80
constexpr float Z_STEPS_PER_MM = 200.0f * 16 / 2.0f;         // T8x2 single-start, 1/16 -> 1600
constexpr float X_ACCEL = 1000.0f;                           // mm/s^2 (jerk-limit via FastAccelStepper ramp)
constexpr float Z_ACCEL = 100.0f;
constexpr float R_SENSE = 0.11f;

FastAccelStepperEngine engine;
HardwareSerial& TMC_UART = Serial2;
TMC2209Stepper drvX(&TMC_UART, R_SENSE, 0b00);
TMC2209Stepper drvZ(&TMC_UART, R_SENSE, 0b01);
Adafruit_NeoPixel leds(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// Debounced NC switch: pressed when the input reads HIGH for 3 consecutive samples (wire break = pressed).
struct Switch {
  int pin;
  uint8_t hist = 0;
  bool state = true;
  void sample() {
    hist = (hist << 1) | (digitalRead(pin) == HIGH);
    if ((hist & 0x7) == 0x7) state = true;
    else if ((hist & 0x7) == 0) state = false;
  }
};
Switch swXmin{PIN_X_MIN}, swXmax{PIN_X_MAX}, swZmin{PIN_Z_MIN}, swZmax{PIN_Z_MAX}, swEstop{PIN_ESTOP};

class StepperAxis : public AxisHal {
 public:
  StepperAxis(float spmm, float accel, Switch& mn, Switch& mx) : spmm_(spmm), accel_(accel), min_(mn), max_(mx) {}
  void attach(int step, int dir) {
    s_ = engine.stepperConnectToPin(step);
    s_->setDirectionPin(dir);
    s_->setAcceleration(static_cast<int32_t>(accel_ * spmm_));
  }
  void moveTo(float mm, float v) override {
    s_->setSpeedInHz(static_cast<uint32_t>(v * spmm_));
    s_->moveTo(static_cast<int32_t>(lroundf(mm * spmm_)));
  }
  void stop() override { s_->stopMove(); }
  void forceStop() override { s_->forceStop(); }
  float position() const override { return s_->getCurrentPosition() / spmm_; }
  void setPosition(float mm) override { s_->setCurrentPosition(static_cast<int32_t>(lroundf(mm * spmm_))); }
  bool running() const override { return s_->isRunning(); }
  bool limitMin() const override { return min_.state; }
  bool limitMax() const override { return max_.state; }
  void enable(bool on) override { digitalWrite(PIN_EN, on ? LOW : HIGH); }

 private:
  FastAccelStepper* s_ = nullptr;
  float spmm_, accel_;
  Switch& min_;
  Switch& max_;
};

class BoardIo : public IoHal {
 public:
  bool estopPressed() const override { return swEstop.state; }
  int toolIdRaw() const override { return analogRead(PIN_TOOL_ID); }
};

StepperAxis axX(X_STEPS_PER_MM, X_ACCEL, swXmin, swXmax);
StepperAxis axZ(Z_STEPS_PER_MM, Z_ACCEL, swZmin, swZmax);
BoardIo io;

AxisConfig cfgX() {
  AxisConfig c;
  c.soft_min = 0; c.soft_max = 800; c.max_speed = 200;
  c.home_fast = 40; c.home_slow = 5; c.home_backoff = 3; c.home_offset = 2; c.home_search = 900;
  return c;
}
AxisConfig cfgZ() {
  AxisConfig c;  // T8x2: 20 mm/s = 600 rpm, fine on 24 V
  c.soft_min = 0; c.soft_max = 500; c.max_speed = 20;
  c.home_fast = 15; c.home_slow = 3; c.home_backoff = 2; c.home_offset = 1; c.home_search = 600;
  return c;
}
Controller* ctl = nullptr;

// ---------------------------------------------------------------- arm v2 (CAN)
struct TwaiPort : CanPort {
  bool ok = false;
  bool begin() {
    twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(PIN_CAN_TX, PIN_CAN_RX, TWAI_MODE_NORMAL);
    g.rx_queue_len = 32;
    twai_timing_config_t t = TWAI_TIMING_CONFIG_500KBITS();  // set the drivers to 500K (MKS command 8AH)
    twai_filter_config_t f = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    ok = twai_driver_install(&g, &t, &f) == ESP_OK && twai_start() == ESP_OK;
    return ok;
  }
  bool send(const mks::Frame& fr) override {
    if (!ok) return false;
    twai_message_t m = {};
    m.identifier = fr.id;
    m.data_length_code = fr.dlc;
    memcpy(m.data, fr.d, fr.dlc);
    return twai_transmit(&m, 0) == ESP_OK;  // non-blocking; the 20 ms poll keeps the TX queue short
  }
  bool receive(mks::Frame& fr) override {
    if (!ok) return false;
    twai_message_t m;
    if (twai_receive(&m, 0) != ESP_OK) return false;
    fr.id = static_cast<uint16_t>(m.identifier);
    fr.dlc = m.data_length_code;
    memcpy(fr.d, m.data, 8);
    return true;
  }
};
TwaiPort canPort;

// joint table, from v2/sizing.py: total ratio (belt x gearbox), limits, home switch angle (VERIFY on the build)
const JointConfig* armJoints() {
  static JointConfig j[NUM_JOINTS];
  const float ratio[NUM_JOINTS] = {10, 20, 25, 15, 5};
  const float lo[NUM_JOINTS] = {-150, -100, -140, -110, -180};
  const float hi[NUM_JOINTS] = {150, 135, 140, 110, 180};
  const uint16_t rpm[NUM_JOINTS] = {600, 500, 600, 600, 600};
  for (int i = 0; i < NUM_JOINTS; ++i) {
    j[i].can_id = static_cast<uint16_t>(i + 1);
    j[i].counts_per_deg = mks::COUNTS_PER_REV * ratio[i] / 360.0f;
    j[i].min_deg = lo[i];
    j[i].max_deg = hi[i];
    j[i].home_deg = lo[i];  // home switch at the negative end of each joint (driver IN_1)
    j[i].max_rpm = rpm[i];
    j[i].acc = 20;
  }
  return j;
}

void setupDriver(TMC2209Stepper& d, uint16_t ma) {
  d.begin();
  d.toff(4);
  d.blank_time(24);
  d.rms_current(ma);
  d.microsteps(16);
  d.pwm_autoscale(true);
  d.en_spreadCycle(false);  // StealthChop: quiet, fine at these speeds
}

void showLeds(Led l, uint32_t ms) {
  uint32_t c = 0;
  switch (l) {
    case Led::IDLE: c = leds.Color(0, 90, 255); break;                                            // steady cyan-blue
    case Led::MOVING: { uint8_t g = 60 + (uint8_t)(60 * (1 + sinf(ms * 0.004f))); c = leds.Color(0, g, 40); break; }  // green pulse
    case Led::HOMING: {                                                                            // amber chase
      leds.clear();
      for (int i = 0; i < NUM_LEDS; ++i) if (((ms / 80) + i) % 6 == 0) leds.setPixelColor(i, leds.Color(255, 120, 0));
      leds.show();
      return;
    }
    case Led::FAULT: c = ((ms / 150) % 2) ? leds.Color(255, 0, 0) : 0; break;                    // fast red blink
    case Led::OFF: c = 0; break;
  }
  leds.fill(c);
  leds.show();
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);  // drivers off until configured
  for (int p : {PIN_X_MIN, PIN_X_MAX}) pinMode(p, INPUT_PULLUP);
  for (int p : {PIN_Z_MIN, PIN_Z_MAX, PIN_ESTOP}) pinMode(p, INPUT);  // external pull-ups
  analogReadResolution(12);
  TMC_UART.begin(115200, SERIAL_8N1, PIN_TMC_RX, PIN_TMC_TX);
  setupDriver(drvX, 900);
  setupDriver(drvZ, 700);
  engine.init();
  axX.attach(PIN_X_STEP, PIN_X_DIR);
  axZ.attach(PIN_Z_STEP, PIN_Z_DIR);
  static Controller c(axX, axZ, io, cfgX(), cfgZ());
  ctl = &c;
  if (canPort.begin()) {  // arm v2 present: attach the CAN arm; without it the gantry runs alone
    static Arm arm(canPort, armJoints());
    c.attachArm(&arm);
  }
  leds.begin();
  leds.setBrightness(60);
  digitalWrite(PIN_EN, LOW);
  Serial.printf("ev BOOT %s\n", FW_VERSION);
}

void loop() {
  static char line[kLineMax + 1];
  static size_t n = 0;
  static bool discard = false;  // inside an over-long line: drop bytes until its newline
  static uint32_t last_led = 0;
  for (Switch* s : {&swXmin, &swXmax, &swZmin, &swZmax, &swEstop}) s->sample();
  while (Serial.available()) {
    char ch = static_cast<char>(Serial.read());
    if (ch == '\n') {
      if (discard) {
        Serial.println("err TOO_LONG");  // never execute the tail of a corrupt line
      } else {
        line[n] = 0;
        char reply[kReplyMax];
        ctl->handleLine(line, reply, sizeof reply);
        Serial.println(reply);
      }
      n = 0;
      discard = false;
    } else if (!discard) {
      if (n < kLineMax) line[n++] = ch;
      else discard = true;
    }
  }
  char ev[kReplyMax];
  if (ctl->update(millis(), ev, sizeof ev)) Serial.println(ev);
  if (millis() - last_led > 20) {
    last_led = millis();
    showLeds(ctl->led(), last_led);
  }
}
