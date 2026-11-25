// rover.h - Rover logic: AUTOMATIC state machine + MANUAL mode
//
// update() runs in the Arduino loop() task. The web handlers run in other
// tasks, so they only call command() / requestMode() / statusJson(), which
// just write/read simple volatile fields; all the real work happens in update().
#pragma once
#include <Arduino.h>
#include "config.h"
#include "hardware.h"

enum State : uint8_t { ST_MANUAL, ST_FORWARD, ST_TURN, ST_PAUSE, ST_STUCK };

static const char* stateName(State s) {
  switch (s) {
    case ST_MANUAL:  return "MANUAL";
    case ST_FORWARD: return "FORWARD";
    case ST_TURN:    return "TURN";
    case ST_PAUSE:   return "PAUSE";
    default:         return "STUCK";
  }
}

class Rover {
 public:
  Motor motorL, motorR;
  Ultrasonic front;
  Gyro gyro;
  HallSensor hallL, hallR;   // wheel rotation (pulse count + rpm)

  // Call once in setup(). Motors start stopped, in manual mode:
  // the rover never drives off by itself after boot.
  void begin() {
    motorL.begin(L_IN1, L_IN2, L_EN, PWM_CH_L, L_INVERT);
    motorR.begin(R_IN1, R_IN2, R_EN, PWM_CH_R, R_INVERT);
    front.begin();
    hallL.begin(HALL_L_PIN);
    hallR.begin(HALL_R_PIN);
    gyro.begin();            // calibrates ~1 s: keep the rover still at power-on
    stop();
  }

  // Tank-style control: each side from -1.0 to +1.0
  void drive(float left, float right) {
    motorL.set(left * LEFT_TRIM);
    motorR.set(right * RIGHT_TRIM);
  }
  void stop() { drive(0, 0); }

  // ----- called from the web handlers (other task) -----
  void command(float l, float r) {
    _cmdL = clampf(l);
    _cmdR = clampf(r);
    _lastCmd = millis();
  }
  void requestMode(bool wantAuto) { _modeReq = wantAuto ? 1 : 2; }

  void statusJson(char* out, size_t n) {
    float d = front.distance;
    snprintf(out, n,
             "{\"mode\":\"%s\",\"state\":\"%s\",\"front\":%s,\"distance\":%d,\"imu\":%s,"
             "\"rpmL\":%d,\"rpmR\":%d,\"pulsesL\":%lu,\"pulsesR\":%lu}",
             _auto ? "auto" : "manual", stateName(_state),
             front.detected ? "true" : "false", d < 0 ? -1 : (int)d,
             gyro.ok ? "true" : "false",
             (int)hallL.rpm(), (int)hallR.rpm(),
             (unsigned long)hallL.count(), (unsigned long)hallR.count());
  }

  // ----- main loop, call every ~20 ms -----
  void update() {
    front.update();   // HC-SR04 read blocks < 12 ms
    gyro.update();

    int8_t req = _modeReq;
    if (req != 0) {
      _modeReq = 0;
      applyMode(req == 1);
    }
    if (_auto) autoStep();
    else manualStep();
  }

 private:
  // ---------- mode switching ----------
  void applyMode(bool wantAuto) {
    if (wantAuto == _auto) return;
    stop();
    _auto = wantAuto;
    if (_auto) {
      _turnAttempts = 0;
      enter(ST_FORWARD);
    } else {
      _state = ST_MANUAL;
      _cmdL = _cmdR = 0;
      Serial.println("mode: manual");
    }
  }

  // ---------- manual ----------
  void manualStep() {
    // Dead-man: phone lost / page closed -> stop
    if (millis() - _lastCmd > CMD_TIMEOUT_MS) {
      stop();
      return;
    }
    float l = _cmdL, r = _cmdR;
    if (MANUAL_SAFETY) {
      float f = (l + r) / 2;   // forward component
      float t = (l - r) / 2;   // turning component
      if (f > 0 && front.detected) f = 0;   // don't drive into an obstacle, turning is still allowed
      l = f + t;
      r = f - t;
    }
    // NOTE: there is no rear sensor, so reversing is not protected.
    drive(fabsf(l) > DEADZONE ? l : 0, fabsf(r) > DEADZONE ? r : 0);
  }

  // ---------- automatic ----------
  void enter(State s) {
    _state = s;
    _t0 = millis();
    Serial.printf("state: %s\n", stateName(s));
    switch (s) {
      case ST_FORWARD:
        drive(CRUISE_SPEED, CRUISE_SPEED);
        break;
      case ST_TURN: {
        gyro.reset();
        float t = TURN_SPEED * _turnDir;
        drive(t, -t);          // spin in place
        break;
      }
      default:                 // ST_PAUSE, ST_STUCK
        stop();
    }
  }

  void pauseThen(State next) {
    _after = next;
    enter(ST_PAUSE);
  }

  void startTurn(int angle = 0) {
    _turnDir = random(0, 2) ? 1 : -1;                       // +1 = right, -1 = left
    _turnTarget = angle ? angle : random(TURN_ANGLE_MIN, TURN_ANGLE_MAX + 1);
  }

  void autoStep() {
    uint32_t elapsed = millis() - _t0;
    switch (_state) {
      case ST_FORWARD:
        if (elapsed >= ATTEMPT_RESET_MS) _turnAttempts = 0;
        if (front.detected) {
          _turnAttempts++;
          if (_turnAttempts > MAX_TURN_ATTEMPTS) {
            enter(ST_STUCK);
          } else {
            startTurn();
            pauseThen(ST_TURN);
          }
        }
        break;

      case ST_TURN: {
        bool done;
        if (gyro.ok) done = fabsf(gyro.angle) >= _turnTarget;
        else         done = elapsed >= (uint32_t)_turnTarget * TURN_MS_PER_DEG;
        if (done || elapsed >= TURN_MAX_MS) pauseThen(ST_FORWARD);
        break;
      }

      case ST_PAUSE:
        if (elapsed >= PAUSE_MS) enter(_after);
        break;

      case ST_STUCK:
        if (elapsed >= STUCK_WAIT_MS) {
          _turnAttempts = 0;
          startTurn(180);
          pauseThen(ST_TURN);
        }
        break;

      default:
        break;
    }
  }

  // ---------- state ----------
  volatile bool _auto = false;
  volatile State _state = ST_MANUAL;
  volatile int8_t _modeReq = 0;            // 0 = none, 1 = auto, 2 = manual

  volatile float _cmdL = 0, _cmdR = 0;
  volatile uint32_t _lastCmd = 0;

  uint32_t _t0 = 0;
  State _after = ST_FORWARD;
  int _turnDir = 1;
  int _turnTarget = 120;                   // degrees
  int _turnAttempts = 0;
};
