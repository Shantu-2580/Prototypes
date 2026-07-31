# Self-Balancing Bot

A two-wheeled self-balancing robot using ESP32, MPU6050 (accelerometer + gyroscope), TB6612FNG motor driver, and Bluetooth gamepad control via Bluepad32.

## Hardware Requirements

| Component | Notes |
|-----------|-------|
| ESP32 DevKit | 30+ GPIO pins |
| MPU6050 | I2C address 0x68 (SDA=21, SCL=22) |
| TB6612FNG | Dual H-bridge motor driver |
| 2x DC Motors | With encoder support (optional) |
| LiPo Battery | 2S-3S (7.4V-11.1V) with voltage regulator |
| Bluetooth Gamepad | Tested with Xbox/PS4/8BitDo controllers |

## Pinout

```
ESP32          TB6612FNG          MPU6050
────────────────────────────────────────────
GPIO 33   →    PWMA
GPIO 14   →    AIN1
GPIO 15   →    AIN2
GPIO 32   →    PWMB
GPIO 26   →    BIN1
GPIO 27   →    BIN2
GPIO 21   →    SDA
GPIO 22   →    SCL
3.3V      →    VCC
GND       →    GND
```

## Firmware Files

### `Codes/P1/P1.ino` — Original Version
**Status:** Reference implementation (has placeholder values)

**Features:**
- Basic PID balance control (100Hz loop)
- Complementary filter for sensor fusion (MPU6050 accel + gyro)
- Bluetooth gamepad control via Bluepad32
- TB6612FNG motor driver interface
- Fall detection safety cutoff (>45°)

**Known Issues:**
- ❌ `Kp`, `Ki`, `Kd`, `base_setpoint` set to `Add_Values_Later` — **won't compile**
- ❌ No calibration routine — `base_setpoint` must be manually tuned per robot
- ❌ No integral anti-windup — causes overshoot after disturbances
- ❌ No motor deadband compensation — motors stall at low PWM
- ❌ No gamepad deadzone — joystick drift causes unwanted movement
- ❌ No controller watchdog — robot keeps moving if controller dies
- ❌ No MPU6050 verification — silent failure if sensor disconnected
- ❌ No serial telemetry — blind PID tuning

---

### `Codes/P1/Program2.ino` — Optimized Version
**Status:** Production-ready with all fixes

**All Original Features Plus:**

| Category | Improvements |
|----------|--------------|
| **Compilable** | Real PID defaults: `Kp=25.0`, `Ki=0.8`, `Kd=1.2` |
| **Calibration** | 2-second startup routine averages accelerometer to find `base_setpoint` automatically |
| **Anti-Windup** | Dual protection: integral clamped at ±400 + integration pauses when output saturates (±255) |
| **Motor Deadband** | `MIN_PWM=25` ensures TB6612FNG overcomes static friction |
| **Gamepad Deadzone** | Ignores joystick input < 20 (prevents center drift) |
| **Watchdog** | Auto-stops motors if controller disconnects >500ms |
| **Sensor Verify** | Checks WHO_AM_I register (0x68) at startup, halts if missing |
| **DLPF Config** | Hardware low-pass filter set to 44Hz (accel) / 42Hz (gyro) |
| **Telemetry** | 1Hz serial output: angle, setpoint, PID terms for tuning |
| **I2C Optimized** | Single 14-byte read transaction per loop |

---

## PID Tuning Guide (Program2)

Open Serial Monitor at **115200 baud**. You'll see:
```
Angle: 0.12 | Setpoint: 0.00 | Out: -15.3 | P: -3.0 I: -0.2 D: -12.1
```

### Tuning Order:
1. **Kp only** (set `Ki=0, Kd=0`): Increase until sustained oscillation, then reduce ~30%
2. **Kd**: Add to dampen oscillations (start ~0.5-2.0)
3. **Ki**: Small value (0.5-2.0) to eliminate steady-state drift

### Tips:
- Hold robot upright during 2-second calibration (LED on ESP32 blinks during this)
- Test on carpet first — smoother surface = easier balance
- If robot falls backward consistently, increase `base_setpoint` slightly (or add weight forward)
- `Kp` typically 20-50, `Kd` 0.5-3, `Ki` 0.5-2 for this hardware

---

## Building & Flashing

### Arduino IDE
1. Install ESP32 board package: `https://dl.espressif.com/dl/package_esp32_index.json`
2. Install **Bluepad32** library via Library Manager
3. Open `Program2.ino`, select board **ESP32 Dev Module**, upload

### PlatformIO (Recommended)
```ini
; platformio.ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
lib_deps =
    ricardoquesada/Bluepad32 @ ^2.0.0
monitor_speed = 115200
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|--------------|-----|
| Won't compile | Using `P1.ino` | Use `Program2.ino` |
| Falls immediately | PID not tuned | Follow tuning guide above |
| Drifts forward/back | Wrong `base_setpoint` | Re-calibrate (reset ESP32) |
| Oscillates violently | Kp too high / Kd too low | Reduce Kp, increase Kd |
| Slow to correct | Kp too low | Increase Kp |
| Motors hum but don't turn | Deadband too low | Increase `MIN_PWM` (try 30-40) |
| Controller doesn't pair | Bluepad32 keys stored | `BP32.forgetBluetoothKeys()` runs on startup |
| MPU6050 not detected | Wiring / I2C address | Check SDA/SCL, try 0x69 (AD0=HIGH) |

---

## License

MIT — Feel free to use, modify, distribute.