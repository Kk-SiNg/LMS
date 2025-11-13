/*
 * Sensors.cpp
 * Implementation for the QTR-8RC sensor array.
 */

#include "Sensors.h"
#include <Arduino.h>

Sensors::Sensors() {
    // Initialize the QTRSensors object
    qtr.setTypeRC();
    // BUGFIX: Added to define this as an array [1, 2, 3, 4]
    qtr.setSensorPins((const uint8_t[]){
        QTR_PIN_1, QTR_PIN_2, QTR_PIN_3, QTR_PIN_4,
        QTR_PIN_5, QTR_PIN_6, QTR_PIN_7, QTR_PIN_8
    }, SensorCount);
    qtr.setEmitterPin(QTR_EMITTER_PIN);
    qtr.setTimeout(2500); // 2.5ms timeout(i.e. if a sensor do not read anything for 2.5 sec then this means it is in background region and not on line)
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
    // This function reads the calibrated values into our sensorValues array
    uint16_t position = qtr.readLineBlack(sensorValues);

    // Return normalized error (-3500 to +3500)
    return (int16_t)position - setpoint;
}

bool Sensors::isIntersection() {
    // Read raw values first
    qtr.read(sensorValues);

    // Simple intersection detection:
    // it's a T, X, or 90-degree turn.
    // Assumes calibrated "black" is > 800.

    // Count how many sensors see the line
    uint8_t sensorsOnLine = 0;
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] > 800) {
            sensorsOnLine++;
        }
    }
    
    // A real junction (T, X, or 90-degree turn)
    // will have 3 or more sensors on the line.
    return sensorsOnLine >= 3;
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