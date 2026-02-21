# Quad-Copter

# 🛩️ ESP32 Quad-Copter Flight Controller

A custom-built quad-copter flight controller implemented from scratch using:

- ESP32 (Freenove ESP32-WROOM)
- Bosch BMI270 IMU (I2C)
- 4x Brushless motors + ESCs
- Custom stabilization stack (no PX4 / Betaflight)

This project implements a ground-up embedded flight control system including IMU bring-up, attitude estimation, motor mixing, and PWM control.

---

## 🚀 Project Goals

- Build a fully custom flight controller from scratch  
- Implement real-time sensor processing on ESP32  
- Develop roll/pitch stabilization without external flight stacks  
- Gain hands-on experience with:
  - Embedded C++
  - Real-time control systems
  - PWM motor control
  - Sensor fusion
  - Hardware debugging

---

## 🧠 System Architecture

```
BMI270 IMU (I2C @ 400kHz)
        ↓
Accelerometer + Gyro Data
        ↓
Attitude Estimation (Roll/Pitch)
        ↓
P Controller
        ↓
Motor Mixer (X-configuration)
        ↓
ESC PWM (1000–2000 µs pulses)
        ↓
Brushless Motors
```

---

## ⚙️ Hardware

### Microcontroller
- ESP32-WROOM (Freenove Dev Board)
- LEDC hardware PWM (4 channels)

### IMU
- Bosch BMI270 (6-axis accel + gyro)
- I2C @ 400kHz

### Motors
- 2212 Brushless Motors (~920kV)
- 30A ESCs
- 3S LiPo Battery (11.1V nominal)

---

## 🔄 Flight Stack Development Roadmap

### Phase 1 – Sensor Bring-Up
- Raw accel/gyro reads  
- Axis verification  
- Bias calibration  

### Phase 2 – Attitude Estimation
- Accelerometer tilt estimation  
- Madgwick AHRS integration  
- Gyro fusion  

### Phase 3 – Control System
- P → PI → PID control  
- Rate vs angle mode  
- Loop timing stabilization  

### Phase 4 – Flight Readiness
- Fixed-rate control loop  
- Motor idle stabilization test (no props)  
- First hover attempt  

---

## 🎯 Future Enhancements (Proposed)

- Yaw control integration  
- Gyro-based rate mode  
- Full PID tuning  
- Remote controller input (SBUS / BLE)  
- Telemetry logging  
- Battery voltage monitoring  
- Custom PCB flight controller  

---

## 👨‍💻 Author

Zachary Bedard  
