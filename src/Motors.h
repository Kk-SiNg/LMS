/*
 * Motors.h
 * Interface for the L298N driver and N-20 encoders.
 * Uses hardware ledc for PWM and hardware PCNT (via ESP32Encoder) for encoders.
 */

#pragma once

#include "Pins.h"
#include <ESP32Encoder.h>

// Tuning constant: Ticks for a 90-degree pivot turn.
// MUST BE TUNED by user.
#define TICKS_FOR_90_DEG 450 

// Tuning constant: Ticks for a 180-degree pivot turn.
#define TICKS_FOR_180_DEG (TICKS_FOR_90_DEG * 2)

// Tuning constant: Ticks to move forward to center on an intersection.
#define TICKS_TO_CENTER 150

class Motors {
public:
    Motors();

    // Initialize pins, PWM channels, and encoders
    void setup();

    // Set motor speeds. Positive = forward, Negative = reverse.
    // Range: -255 to +255
    void setSpeeds(int leftSpeed, int rightSpeed);

    // Hard brake motors
    void stopBrake();

    // === Atomic, Encoder-Based Turn Functions ===
    // These are blocking and execute a precise turn.

    void turn_90_left();
    void turn_90_right();
    void turn_180_back();
    
    // Helper function to move a precise distance: used for centering on intersection
    void moveForward(int ticks);

    // Get encoder counts for debugging or advanced logic
    long getLeftCount();
    long getRightCount();

    // === NEW Encoder Helper Functions ===
    
    // Get average encoder count from both wheels
    long getAverageCount();

    // Clear counts on both encoders
    void clearEncoders();

private:
    // ESP32 ledc PWM channels
    const int pwm_channel_left = 0;
    const int pwm_channel_right = 1;
    const int pwm_frequency = 5000; // 5 kHz
    const int pwm_resolution = 8; // 0-255

    // Encoder objects
    ESP32Encoder leftEncoder;
    ESP32Encoder rightEncoder;
};