#include <Arduino.h>
#include "SparkFun_BMI270_Arduino_Library.h"
#include <Wire.h>
#include <Bluepad32.h>

// IMU Signal Pins
constexpr uint8_t IMU_data = 21;
constexpr uint8_t IMU_clk = 22;

HardwareSerial PiSerial(2);
constexpr uint8_t PI_RX_PIN = 25;
constexpr uint8_t PI_TX_PIN = 26;

// PWM Motor Control Pins
const int ESC_RL = 16;
const int ESC_FR = 17;
const int ESC_RR = 18;
const int ESC_FL = 19;

enum Motor : uint8_t { FR = 0, RL = 1, RR = 2, FL = 3 };

const int PWM_FREQ = 250;
const int PWM_RES_BITS = 16;

// Limits
constexpr int MIN_US = 1000;
constexpr int MIN_US_IN_FLIGHT = 1400;
constexpr int MAX_US = 2000;
constexpr int IDLE_US = 1500;
constexpr int HOVER_US = 1590;

constexpr int MAX_ROLL_DEG = 35;
constexpr int MAX_PITCH_DEG = 35;
constexpr float MAX_YAW_RATE_DPS = 90.0f;
constexpr int STICK_DEADBAND = 30;

// Autonomous Limits (AUTOP)
constexpr float MAX_ROLL_DEG_AUTOP = 5.0f;
constexpr float MAX_PITCH_DEG_AUTOP = 5.0f;
constexpr float MAX_YAW_RATE_DPS_AUTOP = 30.0f;

// Angle-hold gains
constexpr float KP_ROLL = 110.0f;
constexpr float KP_PITCH = 110.0f;
constexpr float KP_YAW_RATE = 2.5f;

// Small integral term to cancel steady hover bias
constexpr float KI_ROLL = 18.0f;
constexpr float KI_PITCH = 18.0f;
constexpr float ITERM_MAX_US = 120.0f;

// Gyro damping gains (deg/s -> us)
constexpr float KD_ROLL_RATE = 1.8f;
constexpr float KD_PITCH_RATE = 1.8f;

// Complementary filter settings
constexpr float COMPLEMENTARY_ALPHA = 0.98f;
constexpr float DT_FALLBACK_S = 0.004f;
constexpr float DT_MIN_S = 0.001f;
constexpr float DT_MAX_S = 0.02f;

// Gyro bias calibration
constexpr int GYRO_CAL_SAMPLES = 500;
constexpr int GYRO_CAL_DELAY_MS = 2;

// Hover trim offsets in degrees.
constexpr float ROLL_TRIM_DEG = 0.0f;
constexpr float PITCH_TRIM_DEG = 0.0f;

// Autonomous Control
constexpr uint32_t PI_CMD_TIMEOUT_MS = 300;
enum class ControlMode : uint8_t {
  MANUAL,
  PI_ASSISTED
};

// Shared state protection
portMUX_TYPE rcMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE controlMux = portMUX_INITIALIZER_UNLOCKED;

// IMU
uint8_t i2cAddress = BMI2_I2C_PRIM_ADDR;
BMI270 imu;

// RC Setup
static ControllerPtr gControllers[BP32_MAX_GAMEPADS] = { nullptr };
static ControllerPtr gCtl = nullptr;
static bool gJustArmed = false;
static ControlMode gControlMode = ControlMode::MANUAL;

struct RcCmd {
  int throttle_us;          // 1000..2000
  float yaw;                // -1..+1
  float roll_target_deg;
  float pitch_target_deg;
  bool armed;
  bool failsafe;
};

struct PiCmd {
  uint32_t sequence;
  float roll_target_deg;
  float pitch_target_deg;
  float yaw_rate_dps;
  int throttle_us;
  uint32_t last_update_ms;
  bool valid;
};

static RcCmd gRc = {1400, 0.0f, 0.0f, 0.0f, false, true};
static PiCmd gPiCmd = {0, 0.0f, 0.0f, 0.0f, 1000, 0, false};

// Debug values
volatile float gDbgRoll = 0.0f;
volatile float gDbgPitch = 0.0f;
volatile int gDbgCmd[4] = {1000, 1000, 1000, 1000};

// Calculate the Roll and Pitch of the drone given IMU Accel data ONLY
static inline void accelToRollPitch(float ax, float ay, float az, float& roll, float& pitch) {
  roll  = atan2f(ay, az);
  pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
}

struct LevelOut {
  float roll_rad;
  float pitch_rad;
  float roll_rate_dps;
  float pitch_rate_dps;
  float yaw_rate_dps;
};

struct AttitudeEstimate {
  float roll_rad;
  float pitch_rad;
};

struct GyroBias {
  float x_dps;
  float y_dps;
  float z_dps;
};

static AttitudeEstimate gAttitude = {0.0f, 0.0f};
static GyroBias gGyroBias = {0.0f, 0.0f, 0.0f};
static float gRollIntegralUs = 0.0f;
static float gPitchIntegralUs = 0.0f;

static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static GyroBias calibrateGyroBias() {
  GyroBias bias = {0.0f, 0.0f, 0.0f};

  Serial.println("Calibrating gyro bias. Keep the drone still...");
  delay(250);

  for (int i = 0; i < GYRO_CAL_SAMPLES; i++) {
    imu.getSensorData();
    bias.x_dps += imu.data.gyroX;
    bias.y_dps += imu.data.gyroY;
    bias.z_dps += imu.data.gyroZ;
    delay(GYRO_CAL_DELAY_MS);
  }

  bias.x_dps /= GYRO_CAL_SAMPLES;
  bias.y_dps /= GYRO_CAL_SAMPLES;
  bias.z_dps /= GYRO_CAL_SAMPLES;

  Serial.printf("Gyro bias: X %.3f Y %.3f Z %.3f deg/s\n",
                bias.x_dps, bias.y_dps, bias.z_dps);
  return bias;
}

static AttitudeEstimate updateAttitudeEstimate(float ax, float ay, float az,
                                               float gx_dps, float gy_dps,
                                               float dt_s) {
  float accelRoll, accelPitch;
  accelToRollPitch(ax, ay, az, accelRoll, accelPitch);

  gAttitude.roll_rad += gx_dps * DEG_TO_RAD * dt_s;
  gAttitude.pitch_rad += gy_dps * DEG_TO_RAD * dt_s;

  gAttitude.roll_rad =
    COMPLEMENTARY_ALPHA * gAttitude.roll_rad + (1.0f - COMPLEMENTARY_ALPHA) * accelRoll;
  gAttitude.pitch_rad =
    COMPLEMENTARY_ALPHA * gAttitude.pitch_rad + (1.0f - COMPLEMENTARY_ALPHA) * accelPitch;

  return gAttitude;
}

LevelOut levelFromSensors(const AttitudeEstimate& attitude,
                          float gx_dps, float gy_dps, float gz_dps,
                          float dt_s,
                          int base_us,
                          float Kp_roll, float Kp_pitch,
                          float Ki_roll, float Ki_pitch,
                          float Kd_roll_rate, float Kd_pitch_rate,
                          float roll_target_deg, float pitch_target_deg,
                          float yaw_target_dps,
                          int out_us[4]) {

  float roll_target  = (roll_target_deg + ROLL_TRIM_DEG) * (M_PI / 180.0f);
  float pitch_target = (pitch_target_deg + PITCH_TRIM_DEG) * (M_PI / 180.0f);

  float roll_err  = roll_target - attitude.roll_rad;
  float pitch_err = pitch_target - attitude.pitch_rad;

  // Only integrate near level so stick commands don't wind up the controller.
  if (fabsf(roll_target_deg) < 2.0f) {
    gRollIntegralUs += (-Ki_roll * roll_err) * dt_s;
    gRollIntegralUs = clampf(gRollIntegralUs, -ITERM_MAX_US, ITERM_MAX_US);
  } else {
    gRollIntegralUs *= 0.98f;
  }

  if (fabsf(pitch_target_deg) < 2.0f) {
    gPitchIntegralUs += (Ki_pitch * pitch_err) * dt_s;
    gPitchIntegralUs = clampf(gPitchIntegralUs, -ITERM_MAX_US, ITERM_MAX_US);
  } else {
    gPitchIntegralUs *= 0.98f;
  }

  float rollCmd = (-Kp_roll * roll_err) + gRollIntegralUs - (Kd_roll_rate * gx_dps);
  float pitchCmd = (Kp_pitch * pitch_err) + gPitchIntegralUs - (Kd_pitch_rate * gy_dps);
  float yawCmd = KP_YAW_RATE * (yaw_target_dps - gz_dps);

  // X quad mixer
  float fr = base_us + pitchCmd - rollCmd - yawCmd;
  float fl = base_us + pitchCmd + rollCmd + yawCmd;
  float rr = base_us - pitchCmd - rollCmd + yawCmd;
  float rl = base_us - pitchCmd + rollCmd - yawCmd;

  out_us[FR] = (int)fr;
  out_us[FL] = (int)fl;
  out_us[RR] = (int)rr;
  out_us[RL] = (int)rl;

  return {attitude.roll_rad, attitude.pitch_rad, gx_dps, gy_dps, gz_dps};
}

static void sendTelemetry(const AttitudeEstimate& attitude, float gx_dps, float gy_dps, float gz_dps, const RcCmd& rc) {
  float roll_deg = attitude.roll_rad * RAD_TO_DEG;
  float pitch_deg = attitude.pitch_rad * RAD_TO_DEG;

  PiSerial.printf(
    "TEL,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%.3f,%.3f,%.3f\n",
    roll_deg,
    pitch_deg,
    gx_dps,
    gy_dps,
    gz_dps,
    rc.armed ? 1 : 0,
    rc.failsafe ? 1 : 0,
    rc.throttle_us,
    rc.roll_target_deg,
    rc.pitch_target_deg,
    rc.yaw
  );
}

static void handleSerialCommands() {
  static String line;

  while(PiSerial.available() > 0) {
    char c = PiSerial.read();

    if(c == '\n') {
      Serial.printf("RXRAW,%s\n", line.c_str());
      if(line.startsWith("CMD,")) {
        unsigned long sequence;
        float roll;
        float pitch;
        float yawRate;
        int throttle;

        int parsed = sscanf(
          line.c_str(),
          "CMD,%lu,%f,%f,%f,%d",
          &sequence,
          &roll,
          &pitch,
          &yawRate,
          &throttle
        );

        if (parsed == 5) {
          float clampedRoll = clampf(roll, -MAX_ROLL_DEG_AUTOP, MAX_ROLL_DEG_AUTOP);
          float clampedPitch = clampf(pitch, -MAX_PITCH_DEG_AUTOP, MAX_PITCH_DEG_AUTOP);
          float clampedYawRate = clampf(yawRate, -MAX_YAW_RATE_DPS_AUTOP, MAX_YAW_RATE_DPS_AUTOP);

          if(throttle < MIN_US) {
            throttle = MIN_US;
          }
          if(throttle > MAX_US) {
            throttle = MAX_US;
          }

          portENTER_CRITICAL(&controlMux);

          gPiCmd.sequence = sequence;
          gPiCmd.roll_target_deg = clampedRoll;
          gPiCmd.pitch_target_deg = clampedPitch;
          gPiCmd.yaw_rate_dps = clampedYawRate;
          gPiCmd.throttle_us = throttle;
          gPiCmd.last_update_ms = millis();
          gPiCmd.valid = true;

          portEXIT_CRITICAL(&controlMux);

          PiSerial.printf(
            "ACK,%lu,%.2f,%.2f,%.2f,%d\n",
            sequence,
            gPiCmd.roll_target_deg,
            gPiCmd.pitch_target_deg,
            gPiCmd.yaw_rate_dps,
            gPiCmd.throttle_us
          );
        }
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}

static bool isPiCmdValid() {
  bool valid;
  uint32_t lastUpdateMs;

  portENTER_CRITICAL(&controlMux);
  valid = gPiCmd.valid;
  lastUpdateMs = gPiCmd.last_update_ms;
  portEXIT_CRITICAL(&controlMux);

  if(!valid) {
    return false;
  }

  uint32_t age_ms = millis() - lastUpdateMs;

  if(age_ms > PI_CMD_TIMEOUT_MS) {
    portENTER_CRITICAL(&controlMux);
    gPiCmd.valid = false;
    portEXIT_CRITICAL(&controlMux);
    return false;
  }

  return true;
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
}

static inline int applyDeadband(int v, int db) {
  if (v > -db && v < db) return 0;
  return v;
}

static inline float stickNorm(int v) {
  if (v < -512) v = -512;
  if (v >  511) v =  511;
  return (float)v / 512.0f;
}

static inline int throttleUsFromLeftY(int ly) {
  ly = applyDeadband(ly, STICK_DEADBAND);
  float y = stickNorm(ly);

  if (y >= 0.0f) {
    return HOVER_US + (int)(y * (MAX_US - HOVER_US));
  }

  return HOVER_US + (int)(y * (HOVER_US - MIN_US));
}

static inline float stickToTargetDeg(int raw, float max_deg) {
  raw = applyDeadband(raw, STICK_DEADBAND);
  float n = stickNorm(raw);
  return n * max_deg;
}

// Convert raw microseconds to LEDC duty cycle
static inline uint32_t usToDuty(uint32_t us) {
  const uint32_t period_us = 1000000UL / PWM_FREQ;
  if (us > period_us) us = period_us;
  return (us * ((1UL << PWM_RES_BITS) - 1)) / period_us;
}

void setThrottleUs(Motor m, uint16_t us) {
  if (us < MIN_US) us = MIN_US;
  if (us > MAX_US) us = MAX_US;
  ledcWrite((uint8_t)m, usToDuty(us));
}

static inline void writeAllMotorsUs(int us) {
  setThrottleUs(FR, us);
  setThrottleUs(RL, us);
  setThrottleUs(RR, us);
  setThrottleUs(FL, us);
}

static inline void rcUpdate() {
  BP32.update();

  RcCmd newRc;

  // Copy current state as base
  portENTER_CRITICAL(&rcMux);
  newRc = gRc;
  portEXIT_CRITICAL(&rcMux);

  if (!gCtl || !gCtl->isConnected()) {
    newRc.failsafe = true;
    newRc.armed = false;
    newRc.throttle_us = 1400;
    newRc.yaw = 0.0f;
    newRc.roll_target_deg = 0.0f;
    newRc.pitch_target_deg = 0.0f;

    portENTER_CRITICAL(&rcMux);
    gRc = newRc;
    gJustArmed = false;
    portEXIT_CRITICAL(&rcMux);
    return;
  }

  if (!gCtl->hasData()) return;

  newRc.failsafe = false;

  int lx = applyDeadband(gCtl->axisX(), STICK_DEADBAND);
  int ly = applyDeadband(-gCtl->axisY(), STICK_DEADBAND);
  int rx = applyDeadband(gCtl->axisRX(), STICK_DEADBAND);
  int ry = applyDeadband(gCtl->axisRY(), STICK_DEADBAND);

  newRc.throttle_us = throttleUsFromLeftY(ly);
  newRc.yaw = stickNorm(lx);
  newRc.roll_target_deg = -stickToTargetDeg(rx, MAX_ROLL_DEG);
  newRc.pitch_target_deg = stickToTargetDeg(ry, MAX_PITCH_DEG);

  uint16_t btn = gCtl->buttons();
  static bool modeWasSwitched = false;
  bool modeSwitchReq = (btn & 0x0008) != 0;

  if (btn & 0x0001) {   // Cross = disarm
    newRc.armed = false;
    portENTER_CRITICAL(&rcMux);
    gRc = newRc;
    gJustArmed = false;
    portEXIT_CRITICAL(&rcMux);
    return;
  }

  if (btn & 0x0002) {   // Circle = arm
    portENTER_CRITICAL(&rcMux);
    if (!gRc.armed) gJustArmed = true;
    newRc.armed = true;
    gRc = newRc;
    portEXIT_CRITICAL(&rcMux);
    return;
  }

  if(modeSwitchReq && !modeWasSwitched) {
    ControlMode currentMode;

    portENTER_CRITICAL(&controlMux);
    currentMode = gControlMode;
    portEXIT_CRITICAL(&controlMux);

    if(currentMode == ControlMode::MANUAL) {
      if(isPiCmdValid()) {
        portENTER_CRITICAL(&controlMux);
        gControlMode = ControlMode::PI_ASSISTED;
        portEXIT_CRITICAL(&controlMux);
        Serial.println("LOG: MODE - PI_ASSISTED");
      } else {
        Serial.println("LOG: MODE REJECTED - PI_COMMAND_STALE");
      }
    } else {
      portENTER_CRITICAL(&controlMux);
      gControlMode = ControlMode::MANUAL;
      portEXIT_CRITICAL(&controlMux);
      Serial.println("LOG: MODE - MANUAL");
    }
  }

  modeWasSwitched = modeSwitchReq;

  portENTER_CRITICAL(&rcMux);
  gRc = newRc;
  portEXIT_CRITICAL(&rcMux);
}

// ---------------- Tasks ----------------

void rcTask(void* pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(20);   // 50 Hz
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    rcUpdate();
    vTaskDelayUntil(&lastWake, period);
  }
}

void controlTask(void* pvParameters) {
  const TickType_t period = pdMS_TO_TICKS(4);    // 250 Hz
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastMicros = micros();

  uint32_t telemetryCounter = 0;

  while (true) {
    handleSerialCommands();
    static bool wasPiCmdValid = false;
    bool piCmdValid = isPiCmdValid();

    PiCmd piLocal;
    ControlMode controlModeLocal;

    portENTER_CRITICAL(&controlMux);
    piLocal = gPiCmd;
    controlModeLocal = gControlMode;
    portEXIT_CRITICAL(&controlMux);

    RcCmd rcLocal;
    portENTER_CRITICAL(&rcMux);
    rcLocal = gRc;

    if (gJustArmed) gJustArmed = false;
    portEXIT_CRITICAL(&rcMux);

    float activeRollTargetDeg = rcLocal.roll_target_deg;
    float activePitchTargetDeg = rcLocal.pitch_target_deg;
    float activeYawRateDps = rcLocal.yaw * MAX_YAW_RATE_DPS;
    int activeThrottleUs = rcLocal.throttle_us;

    if(controlModeLocal == ControlMode::PI_ASSISTED && piCmdValid) {
      activeRollTargetDeg = piLocal.roll_target_deg;
      activePitchTargetDeg = piLocal.pitch_target_deg;
      activeYawRateDps = piLocal.yaw_rate_dps;
      activeThrottleUs = piLocal.throttle_us;
    }

    static uint32_t activePrintCounter = 0;

    activePrintCounter++;

    if (activePrintCounter >= 25) {
      activePrintCounter = 0;

      Serial.printf(
        "LOG: ACTIVE,%s,%.2f,%.2f,%.2f,%d\n",
        controlModeLocal == ControlMode::MANUAL ? "MANUAL" : "PI_ASSISTED",
        activeRollTargetDeg,
        activePitchTargetDeg,
        activeYawRateDps,
        activeThrottleUs
      );
    }

    if(wasPiCmdValid && !piCmdValid) {
      Serial.println("PI_TIMEOUT");

      if(controlModeLocal== ControlMode::PI_ASSISTED) {
        portENTER_CRITICAL(&controlMux);
        gControlMode = ControlMode::MANUAL;
        portEXIT_CRITICAL(&controlMux);
        Serial.println("LOG: MODE - MANUAL");
      }
    }

    wasPiCmdValid = piCmdValid;

    imu.getSensorData();
    uint32_t nowMicros = micros();
    float dt_s = (nowMicros - lastMicros) / 1000000.0f;
    lastMicros = nowMicros;
    dt_s = clampf(dt_s, DT_MIN_S, DT_MAX_S);

    if(!isfinite(dt_s)) {
      dt_s = DT_FALLBACK_S;
    }

    float gyroX_dps = -(imu.data.gyroX - gGyroBias.x_dps);
    float gyroY_dps = imu.data.gyroY - gGyroBias.y_dps;
    float gyroZ_dps = imu.data.gyroZ - gGyroBias.z_dps;
    AttitudeEstimate attitude = updateAttitudeEstimate(
      imu.data.accelX,
      imu.data.accelY,
      imu.data.accelZ,
      gyroX_dps,
      gyroY_dps,
      dt_s);

    gDbgRoll = attitude.roll_rad;
    gDbgPitch = attitude.pitch_rad;

    telemetryCounter++; 
    if(telemetryCounter >= 10) {
      telemetryCounter = 0;

      sendTelemetry(attitude, gyroX_dps, gyroY_dps, gyroZ_dps, rcLocal);
    }

    if (rcLocal.failsafe || !rcLocal.armed) {
      gRollIntegralUs = 0.0f;
      gPitchIntegralUs = 0.0f;
      writeAllMotorsUs(1000);
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    int cmd_us[4];

    // Enforce continuous armed idle floor
    int base_us = activeThrottleUs;
    if (base_us < MIN_US_IN_FLIGHT) base_us = MIN_US_IN_FLIGHT;

    LevelOut ang = levelFromSensors(
      attitude,
      gyroX_dps, gyroY_dps, gyroZ_dps,
      dt_s,
      base_us,
      KP_ROLL, KP_PITCH,
      KI_ROLL, KI_PITCH,
      KD_ROLL_RATE, KD_PITCH_RATE,
      activeRollTargetDeg, activePitchTargetDeg,
      activeYawRateDps,
      cmd_us
    );

    for (int m = 0; m < 4; m++) {
      if (cmd_us[m] < MIN_US_IN_FLIGHT) cmd_us[m] = MIN_US_IN_FLIGHT;
      if (cmd_us[m] > MAX_US) cmd_us[m] = MAX_US;
      setThrottleUs((Motor)m, cmd_us[m]);
    }

    static uint32_t motorPrintCounter = 0;

    if (++motorPrintCounter >= 25) {
      motorPrintCounter = 0;

      PiSerial.printf(
        "LOG: MOTORS,FR=%d,FL=%d,RR=%d,RL=%d\n",
        cmd_us[FR],
        cmd_us[FL],
        cmd_us[RR],
        cmd_us[RL]
      );
    }

    gDbgRoll = ang.roll_rad;
    gDbgPitch = ang.pitch_rad;
    for (int i = 0; i < 4; i++) {
      gDbgCmd[i] = cmd_us[i];
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void setup() {
  Serial.begin(115200);

  PiSerial.begin(115200, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

  rcInit();

  Wire.begin(IMU_data, IMU_clk);
  Wire.setClock(400000);

  Serial.println("Starting IMU Setup...");

  while (imu.beginI2C(i2cAddress) != BMI2_OK) {
    Serial.println("IMU not detected, check wiring");
    delay(200);
  }

  Serial.println("IMU Detected");

  imu.getSensorData();
  float initialRoll = 0.0f;
  float initialPitch = 0.0f;
  accelToRollPitch(imu.data.accelX, imu.data.accelY, imu.data.accelZ, initialRoll, initialPitch);
  gAttitude = {initialRoll, initialPitch};
  gGyroBias = calibrateGyroBias();

  ledcSetup(FR, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(RL, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(RR, PWM_FREQ, PWM_RES_BITS);
  ledcSetup(FL, PWM_FREQ, PWM_RES_BITS);

  ledcAttachPin(ESC_FR, FR);
  ledcAttachPin(ESC_RL, RL);
  ledcAttachPin(ESC_RR, RR);
  ledcAttachPin(ESC_FL, FL);

  // Hold minimum throttle for ESC init
  for (int i = 0; i < 50; i++) {
    writeAllMotorsUs(1000);
    delay(20);
  }

  xTaskCreatePinnedToCore(rcTask, "RC Task", 4096, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(controlTask, "Control Task", 4096, nullptr, 3, nullptr, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
