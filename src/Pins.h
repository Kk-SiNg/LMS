/*
 * Pins.h
 * Defines all hardware pin assignments for the ESP32.
 * This is the single source of truth for all hardware connections.
 */

#pragma once
#include <cstdint>

// === SENSOR ARRAY (QTR-8RC) ===
// 8 sensor inputs + 1 emitter control
#define QTR_PIN_1 2 // Sensor 1 (Right-most)
#define QTR_PIN_2 4
#define QTR_PIN_3 16
#define QTR_PIN_4 17
#define QTR_PIN_5 18
#define QTR_PIN_6 19
#define QTR_PIN_7 21
#define QTR_PIN_8 22 // Sensor 8 (Left-most)
#define QTR_EMITTER_PIN 13

// Total number of sensors
const uint8_t SensorCount = 8;

// === MOTOR CONTROL (L298N) ===
// 6 pins: 2 PWM (Speed), 4 Digital (Direction)

// Left Motor (Motor A)
#define MOTOR_L_IN1 14
#define MOTOR_L_IN2 12
#define MOTOR_L_ENA 27 // PWM Speed Control

// Right Motor (Motor B)
#define MOTOR_R_IN3 33
#define MOTOR_R_IN4 32
#define MOTOR_R_ENB 26 // PWM Speed Control

// === ENCODERS (N-20 Motors) ===
// 4 pins: 2 inputs per motor

// Left Encoder
#define ENCODER_L_A 34
#define ENCODER_L_B 35

// Right Encoder
#define ENCODER_R_A 36
#define ENCODER_R_B 39

// === MISC ===
#define ONBOARD_LED 5// Built-in LED on most ESP32 boards
#define USER_BUTTON 15 // Button to start runs (requires pull-up resistor)