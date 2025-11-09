/*
 * Pins.h
 * Defines all hardware pin assignments for the ESP32.
 * This is the single source of truth for all hardware connections.
 */

#pragma once
#include <cstdint>

// === SENSOR ARRAY (QTR-8RC) ===
// 8 sensor inputs + 1 emitter control
#define QTR_PIN_1 36 // Sensor 1 (Right-most)
#define QTR_PIN_2 39
#define QTR_PIN_3 34
#define QTR_PIN_4 35
#define QTR_PIN_5 32
#define QTR_PIN_6 33
#define QTR_PIN_7 25
#define QTR_PIN_8 26 // Sensor 8 (Left-most)
#define QTR_EMITTER_PIN 27

// Total number of sensors
const uint8_t SensorCount = 8;

// === MOTOR CONTROL (L298N) ===
// 6 pins: 2 PWM (Speed), 4 Digital (Direction)

// Left Motor (Motor A)
#define MOTOR_L_IN1 23
#define MOTOR_L_IN2 22
#define MOTOR_L_ENA 21 // PWM Speed Control

// Right Motor (Motor B)
#define MOTOR_R_IN3 19
#define MOTOR_R_IN4 18
#define MOTOR_R_ENB 5 // PWM Speed Control

// === ENCODERS (N-20 Motors) ===
// 4 pins: 2 inputs per motor

// Left Encoder
#define ENCODER_L_A 17
#define ENCODER_L_B 16

// Right Encoder
#define ENCODER_R_A 4
#define ENCODER_R_B 15

// === MISC ===
#define ONBOARD_LED 2 // Built-in LED on most ESP32 boards
#define USER_BUTTON 0 // Button to start runs (requires pull-up resistor)