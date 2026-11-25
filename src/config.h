// config.h - everything you may want to edit is here.
#pragma once
#include <Arduino.h>
#include "esp_camera.h"   // for framesize_t

// =================================
// Wi-Fi access point (CHANGE THE PASSWORD, 8 characters minimum)
// =================================
#define AP_SSID     "Rover"
#define AP_PASSWORD "change-me-123"

// =================================
// Pins (GPIO numbers)
// Chosen to avoid: boot/strapping pins (0, 3, 45, 46), USB (19, 20),
// UART0 (43, 44), flash/PSRAM (26-37) and the camera pins below.
// Remove the ENA/ENB jumpers on the L298N so PWM controls the speed.
// =================================
constexpr int  L_EN = 42, L_IN1 = 1,  L_IN2 = 2;    // left wheel  (ENA, IN1, IN2)
constexpr int  R_EN = 39, R_IN1 = 41, R_IN2 = 40;   // right wheel (ENB, IN3, IN4)
constexpr bool L_INVERT = false;                     // true if that wheel spins backwards
constexpr bool R_INVERT = false;

constexpr int TRIG_PIN = 38;   // HC-SR04 TRIG
constexpr int ECHO_PIN = 47;   // HC-SR04 ECHO - needs a 5V -> 3.3V divider!
constexpr int I2C_SDA  = 21;   // MPU-6050
constexpr int I2C_SCL  = 14;

// KY-003 Hall sensors, one per drive wheel (S pin). Power them at 5 V and put a
// 5V -> 3.3V divider on each signal wire (1 kOhm in series, 2 kOhm to GND), like ECHO.
// GPIO 3 and 46 are boot-strapping pins: harmless for this use, and the safest
// pins left over once the camera, motors, PSRAM and USB/UART are accounted for.
constexpr int HALL_L_PIN = 3;    // left wheel
constexpr int HALL_R_PIN = 46;   // right wheel

// OV2640 camera pins (same wiring as the common ESP32-S3 camera boards).
// PWDN and RESET are not used: tie PWDN to GND and RESET to 3.3V on the module.
constexpr int CAM_XCLK = 15, CAM_SIOD = 4, CAM_SIOC = 5;
constexpr int CAM_Y2 = 11, CAM_Y3 = 9, CAM_Y4 = 8, CAM_Y5 = 10;   // D0..D3
constexpr int CAM_Y6 = 12, CAM_Y7 = 18, CAM_Y8 = 17, CAM_Y9 = 16; // D4..D7
constexpr int CAM_VSYNC = 6, CAM_HREF = 7, CAM_PCLK = 13;

constexpr framesize_t CAM_FRAMESIZE = FRAMESIZE_VGA;  // 640x480 (QVGA = lighter)
constexpr int  CAM_JPEG_QUALITY = 12;                 // 10 = best, 63 = worst
constexpr bool CAM_VFLIP   = false;                   // flip if the image is upside down
constexpr bool CAM_HMIRROR = false;

// =================================
// Drive tuning
// =================================
constexpr float LEFT_TRIM    = 1.0f;   // if it drifts to one side, lower the FASTER side
constexpr float RIGHT_TRIM   = 1.0f;   // (e.g. 0.92) so it drives straight
constexpr float CRUISE_SPEED = 0.60f;  // 0.0 - 1.0
constexpr float TURN_SPEED   = 0.55f;
constexpr float MIN_DUTY     = 0.35f;  // below this the motors stall under load
constexpr int   PWM_FREQ     = 1000;   // Hz
constexpr int   PWM_BITS     = 10;     // duty range 0..1023
constexpr int   PWM_CH_L     = 0;      // LEDC channels (camera uses channel 7)
constexpr int   PWM_CH_R     = 1;

// =================================
// Ultrasonic sensor (HC-SR04)
// =================================
constexpr float STOP_DIST_CM    = 25;     // closer than this -> obstacle
constexpr float CLEAR_DIST_CM   = 35;     // farther than this -> clear
constexpr uint32_t MEASURE_MS   = 60;     // HC-SR04 needs >= 50 ms between pings
constexpr uint32_t ECHO_TIMEOUT_US = 12000;  // ~2 m max range

// =================================
// Hall sensors (wheel rotation)
// =================================
constexpr int      HALL_MAGNETS_PER_REV   = 1;        // magnets glued on each wheel
constexpr uint32_t HALL_DEBOUNCE_US       = 3000;     // ignore pulses closer than this
constexpr uint32_t HALL_RPM_TIMEOUT_US    = 1500000;  // no pulse for this long -> 0 rpm

// =================================
// Gyro (MPU-6050)
// =================================
constexpr uint8_t MPU_ADDR  = 0x68;
constexpr uint8_t GYRO_REG  = 0x47;    // 0x43 = X, 0x45 = Y, 0x47 = Z (vertical if flat)
constexpr float GYRO_NOISE_DPS = 0.5f; // ignore rates smaller than this (deg/s)

// =================================
// Automatic mode
// =================================
constexpr uint32_t LOOP_MS          = 20;
constexpr uint32_t PAUSE_MS         = 150;   // short stop between direction changes
constexpr int      TURN_ANGLE_MIN   = 90;    // degrees
constexpr int      TURN_ANGLE_MAX   = 160;
constexpr uint32_t TURN_MS_PER_DEG  = 8;     // only used if the gyro is missing
constexpr uint32_t TURN_MAX_MS      = 3000;  // safety: never turn longer than this
constexpr uint32_t ATTEMPT_RESET_MS = 1000;  // drive this long clear -> stuck counter resets
constexpr int      MAX_TURN_ATTEMPTS = 3;
constexpr uint32_t STUCK_WAIT_MS    = 2000;

// =================================
// Manual mode
// =================================
constexpr uint32_t CMD_TIMEOUT_MS = 500;   // no command for this long -> motors stop
constexpr float    DEADZONE       = 0.08f; // ignore tiny joystick values
constexpr bool     MANUAL_SAFETY  = true;  // block driving FORWARD into an obstacle
