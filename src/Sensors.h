/*
 * Sensors.h
 * Interface for the Pololu QTR-8RC Reflectance Array.
 * Handles calibration, line error calculation, and intersection detection.
 */

#pragma once

#include <QTRSensors.h>
#include "Pins.h"

// NEW: A struct to hold the results of checking paths
struct PathOptions {
    bool left = false;
    bool right = false;
};

class Sensors {
public:
    // Constructor
    Sensors();

    // Initialize and calibrate the sensors. Blocks for 10 seconds.
    void setup();

    // Get the line position error (-3500 to +3500).
    int16_t getLineError();

    // Check if the robot is at an intersection (T, X, or 90-deg turn).
    bool isIntersection();

    // Check if the line has ended (dead end or finish line).
    bool isLineEnd();

    //check for finishing.
    bool isFinished();

    // NEW: Function to check all available paths at an intersection
    PathOptions getOpenPaths();

    // Reads raw sensor values for debugging or advanced detection.
    void readRaw(uint16_t* sensorValues);

private:
    QTRSensors qtr;
    uint16_t sensorValues[8]; 

    // Setpoint is the "perfectly centered" value (SensorCount-1) * 1000 / 2
    const int16_t setpoint = (SensorCount - 1) * 1000 / 2; // = 3500
    
    // Threshold for detecting a black line
    const uint16_t lineThreshold = 300;
    const uint16_t black_surface = 800;
};