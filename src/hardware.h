// hardware.h - drivers: Motor (L298N), Ultrasonic (HC-SR04), Gyro (MPU-6050)
#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "esp_timer.h"
#include "config.h"

static inline float clampf(float x, float lo = -1.0f, float hi = 1.0f) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ---------------------------------------------------------------------
// One DC motor on an L298N channel: 2 direction pins + 1 PWM (enable) pin
// ---------------------------------------------------------------------
class Motor {
 public:
  void begin(int in1, int in2, int en, int channel, bool invert) {
    _in1 = in1; _in2 = in2; _en = en; _ch = channel; _invert = invert;
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);
    digitalWrite(_in1, LOW);
    digitalWrite(_in2, LOW);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcAttach(_en, PWM_FREQ, PWM_BITS);
#else
    ledcSetup(_ch, PWM_FREQ, PWM_BITS);
    ledcAttachPin(_en, _ch);
#endif
    writeDuty(0);
  }

  // speed: -1.0 (full reverse) ... 0 (stop) ... +1.0 (full forward)
  void set(float speed) {
    speed = clampf(speed);
    if (_invert) speed = -speed;
    if (speed == 0.0f) {
      digitalWrite(_in1, LOW);
      digitalWrite(_in2, LOW);
      writeDuty(0);
      return;
    }
    float duty = MIN_DUTY + (1.0f - MIN_DUTY) * fabsf(speed);
    digitalWrite(_in1, speed > 0 ? HIGH : LOW);
    digitalWrite(_in2, speed > 0 ? LOW : HIGH);
    writeDuty((uint32_t)(duty * ((1 << PWM_BITS) - 1)));
  }

 private:
  void writeDuty(uint32_t d) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(_en, d);
#else
    ledcWrite(_ch, d);
#endif
  }
  int _in1 = 0, _in2 = 0, _en = 0, _ch = 0;
  bool _invert = false;
};

// ---------------------------------------------------------------------
// HC-SR04 with hysteresis: 2 readings in a row are needed to change state
// ---------------------------------------------------------------------
class Ultrasonic {
 public:
  volatile float distance = -1;   // cm, -1 = nothing within range
  volatile bool detected = false;

  void begin() {
    pinMode(TRIG_PIN, OUTPUT);
    digitalWrite(TRIG_PIN, LOW);
    pinMode(ECHO_PIN, INPUT);
  }

  void update() {
    uint32_t now = millis();
    if (now - _last < MEASURE_MS) return;
    _last = now;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    unsigned long t = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);  // 0 = timeout
    float d = (t == 0) ? -1.0f : t / 58.3f;                      // us -> cm
    distance = d;

    if (d >= 0 && d <= STOP_DIST_CM) {
      _near++;
      _far = 0;
    } else if (d < 0 || d >= CLEAR_DIST_CM) {
      _far++;
      _near = 0;
    }
    // between the two thresholds: keep the current state
    if (_near >= 2) detected = true;
    else if (_far >= 2) detected = false;
  }

 private:
  uint32_t _last = 0;
  int _near = 0, _far = 0;
};

// ---------------------------------------------------------------------
// MPU-6050 gyro, integrated to an angle (degrees) for accurate turns
// ---------------------------------------------------------------------
class Gyro {
 public:
  bool ok = false;
  volatile float angle = 0;

  void begin() {
    Wire.begin(I2C_SDA, I2C_SCL, 400000);
    Wire.beginTransmission(MPU_ADDR);
    if (Wire.endTransmission() != 0) {
      Serial.println("MPU-6050 not found -> turns will be timed instead");
      return;
    }
    writeReg(0x6B, 0x00);   // wake up
    writeReg(0x1B, 0x10);   // gyro range +/-1000 deg/s
    delay(100);
    Serial.println("Calibrating gyro - keep the rover still...");
    long total = 0;
    for (int i = 0; i < 200; i++) {
      total += readRaw();
      delay(5);
    }
    _bias = total / 200.0f;
    _lastUs = micros();
    ok = true;
  }

  void reset() { angle = 0; }

  void update() {
    uint32_t now = micros();
    float dt = (now - _lastUs) * 1e-6f;
    _lastUs = now;
    if (!ok) return;
    float rate = (readRaw() - _bias) / 32.8f;   // deg/s (32.8 LSB per deg/s)
    if (fabsf(rate) >= GYRO_NOISE_DPS) angle = angle + rate * dt;
  }

 private:
  void writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    Wire.endTransmission();
  }
  int16_t readRaw() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(GYRO_REG);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU_ADDR, 2);
    int16_t hi = Wire.read();
    int16_t lo = Wire.read();
    return (int16_t)((hi << 8) | lo);
  }
  float _bias = 0;
  uint32_t _lastUs = 0;
};

// ---------------------------------------------------------------------
// KY-003 Hall sensor (A3144): output goes LOW when a magnet passes.
// Pulses are counted in an interrupt; rpm() comes from the time between
// the last two pulses (accurate even at low speed).
// ---------------------------------------------------------------------
class HallSensor {
 public:
  void begin(int pin) {
    pinMode(pin, INPUT);   // the KY-003 board has its own pull-up resistor
    attachInterruptArg(pin, isr, this, FALLING);
  }

  uint32_t count() const { return _count; }

  float rpm() const {
    if (_count < 2) return 0;
    uint32_t age = (uint32_t)esp_timer_get_time() - _lastUs;
    uint32_t period = _periodUs;
    if (age > HALL_RPM_TIMEOUT_US || period == 0) return 0;
    return 60000000.0f / ((float)period * HALL_MAGNETS_PER_REV);
  }

 private:
  static void IRAM_ATTR isr(void* arg) {
    HallSensor* s = static_cast<HallSensor*>(arg);
    uint32_t now = (uint32_t)esp_timer_get_time();
    uint32_t dt = now - s->_lastUs;
    if (dt >= HALL_DEBOUNCE_US) {          // ignore contact bounce / noise
      s->_periodUs = dt;
      s->_lastUs = now;
      s->_count = s->_count + 1;
    }
  }
  volatile uint32_t _count = 0, _lastUs = 0, _periodUs = 0;
};
