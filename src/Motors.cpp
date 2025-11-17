/*
 * Motors.cpp
 * Implementation for the motor control.
 */

#include "Motors.h"
#include <Arduino.h>

Motors::Motors() {
    // Constructor
}

void Motors::setup() {
    // 1. Setup L298N direction pins as outputs
    pinMode(MOTOR_L_IN1, OUTPUT);
    pinMode(MOTOR_L_IN2, OUTPUT);
    pinMode(MOTOR_R_IN3, OUTPUT);
    pinMode(MOTOR_R_IN4, OUTPUT);

    // 2. Setup ledc PWM for speed control
    ledcSetup(pwm_channel_left, pwm_frequency, pwm_resolution);
    ledcSetup(pwm_channel_right, pwm_frequency, pwm_resolution);

    // 3. Attach PWM channels to ENA/ENB pins
    ledcAttachPin(MOTOR_L_ENA, pwm_channel_left);
    ledcAttachPin(MOTOR_R_ENB, pwm_channel_right);

    // 4. Setup Encoders
    // Use ESP32 hardware pulse counters
    puType pu_type = puType::up;
    ESP32Encoder::useInternalWeakPullResistors = pu_type; //to be called before attaching encoder pins
    leftEncoder.attachHalfQuad(ENCODER_L_A, ENCODER_L_B);
    rightEncoder.attachHalfQuad(ENCODER_R_A, ENCODER_R_B);
    
    // Clear initial counts
    leftEncoder.clearCount();
    rightEncoder.clearCount();

    // Start with motors stopped
    stopBrake();
}

void Motors::setSpeeds(int leftSpeed, int rightSpeed) {
    // Clamp speeds to -255 to +255
    leftSpeed = constrain(leftSpeed, -255, 255);
    rightSpeed = constrain(rightSpeed, -255, 255);

    // === Left Motor ===
    if (leftSpeed >= 0) {
        digitalWrite(MOTOR_L_IN1, HIGH);
        digitalWrite(MOTOR_L_IN2, LOW);
    } else {
        digitalWrite(MOTOR_L_IN1, LOW);
        digitalWrite(MOTOR_L_IN2, HIGH);
    }
    ledcWrite(pwm_channel_left, abs(leftSpeed));

    // === Right Motor ===
    // Note: IN3/IN4 may need to be swapped if motor is wired "backwards"
    if (rightSpeed >= 0) {
        digitalWrite(MOTOR_R_IN3, HIGH);
        digitalWrite(MOTOR_R_IN4, LOW);
    } else {
        digitalWrite(MOTOR_R_IN3, LOW);
        digitalWrite(MOTOR_R_IN4, HIGH);
    }
    ledcWrite(pwm_channel_right, abs(rightSpeed));
}

void Motors::stopBrake() {
    // L298N brake mode (both inputs HIGH)
    digitalWrite(MOTOR_L_IN1, HIGH);
    digitalWrite(MOTOR_L_IN2, HIGH);
    digitalWrite(MOTOR_R_IN3, HIGH);
    digitalWrite(MOTOR_R_IN4, HIGH);
    ledcWrite(pwm_channel_left, 0);
    ledcWrite(pwm_channel_right, 0);
}

// === Atomic Turn Functions ===

void Motors::turn_90_left() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    
    setSpeeds(180, -180); // Pivot turn
    
    // Wait until the right wheel has moved the required distance
    while (rightEncoder.getCount() < TICKS_FOR_90_DEG) {
        delay(1);
    }
    stopBrake();
}

void Motors::turn_90_right() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    
    setSpeeds(-180, 180); // Pivot turn
    
    // Wait until the left wheel has moved the required distance
    while (leftEncoder.getCount() < TICKS_FOR_90_DEG) {
        delay(1);
    }
    stopBrake();
}

void Motors::turn_180_back() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    
    setSpeeds(180, -180); // Pivot turn
    
    // Wait for 180 degrees
    while (rightEncoder.getCount() < TICKS_FOR_180_DEG) {
        delay(1);
    }
    stopBrake();
}

void Motors::rotate(){
    leftEncoder.clearCount();
    rightEncoder.clearCount();
    setSpeeds(-180,180);
}

void Motors::moveForward(int ticks) {
    leftEncoder.clearCount();
    rightEncoder.clearCount();

    setSpeeds(150, 150);

    // Wait until average distance is met
    while ((leftEncoder.getCount() + rightEncoder.getCount()) / 2 < ticks) {
        delay(1);
    }
    stopBrake();
}

long Motors::getLeftCount() {
    return leftEncoder.getCount();
}

long Motors::getRightCount() {
    return rightEncoder.getCount();
}

// === NEW Encoder Helper Functions ===

long Motors::getAverageCount() {
    // Return average of left and right. Cast to long to prevent overflow.
    return (long)(leftEncoder.getCount() + rightEncoder.getCount()) / 2;
}

void Motors::clearEncoders() {
    leftEncoder.clearCount();
    rightEncoder.clearCount();
}