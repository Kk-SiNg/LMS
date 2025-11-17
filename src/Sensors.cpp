/*
 * Sensors.cpp
 * Implementation for the QTR-8RC sensor array.
 */

#include "Sensors.h"
#include "Motors.h"
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
    Serial.println("Sweeping robot over line and background for 10s...");
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
    uint16_t position = qtr.readLineWhite(sensorValues);

    // Return normalized error (-3500 to +3500)
    return (int16_t)position - setpoint;
}

bool Sensors::isIntersection() {
    // Read raw values first
    qtr.read(sensorValues);

    // Simple intersection detection:
    // If both the far-left and far-right sensors see black,
    // it's a T, X, or 90-degree turn.
    // Assumes calibrated "white" is < 300.
    bool left_sees_line = sensorValues[0] < lineThreshold; // Index 0 (QTR_PIN_1)
    bool right_sees_line = sensorValues[7] < lineThreshold; // Last sensor (QTR_PIN_8)

    return left_sees_line || right_sees_line;
}

// NEW: Implementation of the path-checking function
PathOptions Sensors::getOpenPaths() {
    // This function assumes the robot has moved forward to center
    // itself over the junction. It reads all 8 sensors to see
    // which paths (L, S, R) are available.
    
    qtr.read(sensorValues);
    
    PathOptions paths;

    // Check for a left path (using the left-most sensors)
    // Sensor 7 is QTR_PIN_8, Sensor 6 is QTR_PIN_7
    paths.right = (sensorValues[0] < lineThreshold || sensorValues[1] < lineThreshold);
    
    // Check for a right path (using the right-most sensors)
    // Sensor 0 is QTR_PIN_1, Sensor 1 is QTR_PIN_2
    paths.left = (sensorValues[7] < lineThreshold || sensorValues[6] < lineThreshold);
    
    return paths;
}


bool Sensors::isFinished() {
    // Read raw values
    qtr.read(sensorValues);

    // Check if all sensors are on a white surface
    // Assumes calibrated "black" is > 200.
    for (uint8_t i = 0; i < SensorCount; i++) {
        if (sensorValues[i] < 200) {
            return false; // Found a black reading
        }
    }
    return true; // All sensors read white
}

bool Sensors::isLineEnd(){
    // Read raw values
    qtr.read(sensorValues);

    for (uint8_t i = 0; i < SensorCount; i++) {
            if (sensorValues[i] > black_surface) {
                return false; // Found a white reading
            }
        }
    return true; // All sensors read black
}


void Sensors::readRaw(uint16_t* values) {
    qtr.read(values);
}