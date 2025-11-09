/*
 * Sensors.cpp
 * Implementation for the QTR-8RC sensor array.
 */

#include "Sensors.h"
#include <Arduino.h>

Sensors::Sensors() {
    // Initialize the QTRSensors object
    qtr.setTypeRC();
    qtr.setSensorPins((const uint8_t){
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    }, SensorCount);
    qtr.setEmitterPin(QTR_EMITTER_PIN);
    qtr.setTimeout(2500); // 2.5ms timeout
}

void Sensors::setup() {
    Serial.println("--- Sensor Calibration START ---");
    Serial.println("Sweep robot over line and background for 10s...");
    pinMode(ONBOARD_LED, OUTPUT);
    digitalWrite(ONBOARD_LED, HIGH);

    // Calibration: 10 seconds (20ms * 500)
    for (uint16_t i = 0; i < 500; i++) {
        qtr.calibrate();
        delay(20);
    }

    digitalWrite(ONBOARD_LED, LOW);
    Serial.println("--- Sensor Calibration DONE ---");

    // Print calibration results for debugging
    Serial.println("Calibrated Minimums:");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.minimum[i]);
        Serial.print(" ");
    }
    Serial.println();

    Serial.println("Calibrated Maximums:");
    for (uint8_t i = 0; i < SensorCount; i++) {
        Serial.print(qtr.calibrationOn.maximum[i]);
        Serial.print(" ");
    }
    Serial.println();
}

int16_t Sensors::getLineError() {
    // Get calibrated line position (0-7000)
    uint16_t position = qtr.readLineBlack(sensorValues);
    
    // Return normalized error (-3500 to +3500)
    return (int16_t)position - setpoint;
}

bool Sensors::isIntersection() {
    // Read raw values first
    qtr.read(sensorValues);

    // Simple intersection detection:
    // If both the far-left and far-right sensors see black,
    // it's a T, X, or 90-degree turn.
    // Assumes calibrated "black" is > 800.
    bool left_sees_line = sensorValues > 800;
    bool right_sees_line = sensorValues > 800;

    return left_sees_line && right_sees_line;
}

bool Sensors::isLineEnd() {
    // Read raw values
    qtr.read(sensorValues);

    // Check if all sensors are on a white surface
    // Assumes calibrated "white" is < 200.
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 200) {
            return false; // Found a non-white reading
        }
    }
    return true; // All sensors read white
}

void Sensors::readRaw(uint16_t* values) {
    qtr.read(values);
}