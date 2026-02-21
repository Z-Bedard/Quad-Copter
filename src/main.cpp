#include <Arduino.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Wire.h>
#include <Bluepad32.h>

// IMU Signal Pins
constexpr uint8_t IMU_data = 21;
constexpr uint8_t IMU_clk = 22;

// PWM Motor Control Pins
const int ESC_RL = 16;
const int ESC_FR = 17;
const int ESC_RR = 18;
const int ESC_FL = 19;
enum Motor : uint8_t { FR=0, RL=1, RR=2, FL=3 };
const int PWM_FREQ = 50;        // 50 Hz servo signal
const int PWM_RES_BITS = 16;    // 0–65535 duty

// Limits
constexpr int MIN_US = 1000;
constexpr int MAX_US = 2000;
constexpr int IDLE_US = 1400;

// Initial Values for pitch and roll
constexpr float KP_ROLL  = 50.0f;
constexpr float KP_PITCH = 50.0f;

// Initial Speed
volatile int motor_us[4] = {1500, 1500, 1500, 1500};

// Calculate the Roll and Pitch of the drone given IMU Accel data ONLY (Not Flight Ready)
static inline void accelToRollPitch(float ax, float ay, float az, float &roll, float &pitch) {
  // roll about X-axis, pitch about Y-axis (common convention)
  roll  = atan2f(ay, az);
  pitch = atan2f(-ax, sqrtf(ay*ay + az*az));
}

// Struct for pitch and roll that comes from the calculation function
struct LevelOut {
  float roll_rad;
  float pitch_rad;
};

// Take the Pitch and Roll calculations and determine speeds for the motors
LevelOut levelFromAccel(float ax, float ay, float az, int base_us, float Kp_roll, float Kp_pitch, int out_us[4]) {
  float roll, pitch;
  accelToRollPitch(ax, ay, az, roll, pitch);

  // target is level: roll = 0, pitch = 0
  float roll_err  = 0.0f - roll;
  float pitch_err = 0.0f - pitch;

  // controller outputs are in "microseconds" worth of correction
  float rollCmd  = Kp_roll  * roll_err;   // us
  float pitchCmd = Kp_pitch * pitch_err;  // us

  // mixer (X quad, no yaw)
  float fr = base_us + pitchCmd - rollCmd;
  float fl = base_us + pitchCmd + rollCmd;
  float rr = base_us - pitchCmd - rollCmd;
  float rl = base_us - pitchCmd + rollCmd;

  // clamp and write to array
  out_us[FR] = (int)fr;
  out_us[FL] = (int)fl;
  out_us[RR] = (int)rr;
  out_us[RL] = (int)rl;

  return {roll, pitch};
}

// RC Setup
static ControllerPtr gControllers[BP32_MAX_GAMEPADS] = { nullptr };
static ControllerPtr gCtl = nullptr;

struct RcCmd {
  int throttle_us;   // 1000..2000
  bool armed;
  bool failsafe;
};

static RcCmd gRc = { 1400, false, true };

static inline int throttleUsFromTrigger(int t) { // 0..1023 -> 1000..2000
  if (t < 0) t = 0;
  if (t > 1023) t = 1023;
  return 1000 + (t * 1000) / 1023;
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (gControllers[i] == nullptr) {
      gControllers[i] = ctl;
      if (!gCtl) gCtl = ctl;
      Serial.printf("Controller connected (slot %d): %s\n", i, ctl->getModelName().c_str());
      return;
    }
  }
  Serial.println("Controller connected but no empty slot");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (gControllers[i] == ctl) {
      gControllers[i] = nullptr;
      if (gCtl == ctl) gCtl = nullptr;
      Serial.printf("Controller disconnected (slot %d)\n", i);
      return;
    }
  }
}

static inline void rcInit() {
  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableVirtualDevice(false);
  // BP32.forgetBluetoothKeys();  // only when you need to clear pairings
}

static inline void rcUpdate() {
  BP32.update();

  // default failsafe if no controller
  if (!gCtl || !gCtl->isConnected()) {
    gRc.failsafe = true;
    gRc.armed = false;
    gRc.throttle_us = 1400;
    return;
  }

  if (!gCtl->hasData()) return;

  gRc.failsafe = false;

  // throttle from R2 trigger
  gRc.throttle_us = throttleUsFromTrigger(gCtl->throttle());

  // simple arming (PS4 mappings you used earlier):
  // Cross (X) disarms, Circle arms (only if throttle low)
  uint16_t btn = gCtl->buttons();
  if (btn & 0x0001) {            // Cross
    gRc.armed = false;
  }
  if (btn & 0x0002) {            // Circle
    if (gRc.throttle_us <= 1050) gRc.armed = true;
  }
}

// Convert raw seconds to usable duty cycle
static inline uint32_t usToDuty(uint32_t us) {
  const uint32_t period_us = 1000000UL / PWM_FREQ; // 20000
  if (us > period_us) us = period_us;
  return (us * ((1UL << PWM_RES_BITS) - 1)) / period_us;
}

// Set Speed of Motor
void setThrottleUs(Motor m, uint16_t us) {
  // clamp to typical ESC range
  if (us < 1000) us = 1000;
  if (us > 2000) us = 2000;
  ledcWrite((uint8_t)m, usToDuty(us));
}

// For shut off
static inline void writeAllMotorsUs(int us) {
  setThrottleUs(FR, us);
  setThrottleUs(RL, us);
  setThrottleUs(RR, us);
  setThrottleUs(FL, us);
}

uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR; // 0x68
BMI270 imu; // Declare a BMI270 Datatype so we can use the BMI lib

void setup() {
  Serial.begin(115200);

  rcInit();
  Wire.begin(IMU_data, IMU_clk);
  Wire.setClock(400000); // 400 kHz

  Serial.println("Starting IMU Setup...");

  while(imu.beginI2C(i2cAddress) != BMI2_OK){
    Serial.println("IMU not detected, check wiring");
    delay(1000);
  }

  Serial.println("IMU Detected");

  ledcSetup(FR, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(RL, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(RR, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(FL, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(ESC_FR, FR);
  ledcAttachPin(ESC_RL, RL);
  ledcAttachPin(ESC_RR, RR);
  ledcAttachPin(ESC_FL, FL);

  for (int i = 0; i < 50; i++) { // 50 * 20ms = 1s
    setThrottleUs(FR, 1000);
    setThrottleUs(RL, 1000);
    setThrottleUs(RR, 1000);
    setThrottleUs(FL, 1000);
    delay(20);
  }

  delay(3000);
}

void loop() {
  rcUpdate();

  if (gRc.failsafe || !gRc.armed) {
    writeAllMotorsUs(1000);
    delay(20);
    return;
  }

  imu.getSensorData();

  int cmd_us[4];
  int base_us = gRc.throttle_us;   // replaces BASE_US
  if (base_us < IDLE_US) base_us = IDLE_US;

  LevelOut ang = levelFromAccel(imu.data.accelX, imu.data.accelY, imu.data.accelZ,
                              base_us, KP_ROLL, KP_PITCH, cmd_us);

  // clamp + apply
  for (int m = 0; m < 4; m++) {
    if (cmd_us[m] < MIN_US) cmd_us[m] = MIN_US;
    if (cmd_us[m] > MAX_US) cmd_us[m] = MAX_US;
    setThrottleUs((Motor)m, cmd_us[m]);
  }

  // Serial.print("\t");
  Serial.printf("roll=%.3f pitch=%.3f | FR %d FL %d RR %d RL %d\n",
              ang.roll_rad, ang.pitch_rad,
              cmd_us[FR], cmd_us[FL], cmd_us[RR], cmd_us[RL]);

  // Serial printout of current state struct
  Serial.printf("failsafe=%d armed=%d throttle_us=%d\n",
              (int)gRc.failsafe, (int)gRc.armed, gRc.throttle_us);

  delay(20);
}
