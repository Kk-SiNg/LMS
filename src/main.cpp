/*
 * main.cpp (.ino file)
 * Main application for the IIT Bombay Mesmerise Line Maze Solver.
 * Implements the core Finite State Machine (FSM) and PID control loop.
 */

#include "Pins.h"
#include "Sensors.h"
#include "Motors.h"
#include "PathOptimization.h"
#include <QuickPID.h>

// === Global Objects ===
Sensors sensors;
Motors motors;
PathOptimization optimizer;

// === Robot State Machine ===
enum RobotState {
    CALIBRATING,
    WAIT_FOR_RUN_1,
    MAPPING,
    OPTIMIZING,
    WAIT_FOR_RUN_2,
    SOLVING,
    FINISHED
};
RobotState currentState = CALIBRATING;

// === Path Storage (Using safe String class) ===
String rawPath = "";
String optimizedPath = "";
int pathIndex = 0;

// === PID Controller ===
// These constants MUST be tuned.
float Kp = 0.05, Ki = 0.0001, Kd = 0.02; 
float pidInput, pidOutput, pidSetpoint = 0;
// Base speed (0-255). Tune this for your robot.
int baseSpeed = 150;
QuickPID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// === FSM Helper ===
bool lineFoundAtIntersection = false;


void setup() {
    Serial.begin(115200);
    Serial.println("Booting Maze Solver...");
    pinMode(ONBOARD_LED, OUTPUT);
    
    // 1. Initialize subsystems
    motors.setup();
    
    // Setup button with internal pull-up
    // Assumes button is wired between USER_BUTTON pin (GPIO 0) and GND
    pinMode(USER_BUTTON, INPUT_PULLUP);

    // 2. Calibrate Sensors
    currentState = CALIBRATING;
    sensors.setup(); // This blocks for 10s

    // 3. Setup PID
    pid.SetTunings(Kp, Ki, Kd);
    pid.SetSampleTimeUs(1000); // 1ms sample time
    pid.SetMode(QuickPID::Control(1)); //(1)--->Automatically calculates and allots value to pidOutput

    // 4. Wait for Run 1
    currentState = WAIT_FOR_RUN_1;
    Serial.println("Calibration complete. Place at START.");
    Serial.println("Press button to begin Run 1 (Mapping)...");
}

void runPID();

void loop() {
    
    switch (currentState) {

        case WAIT_FOR_RUN_1:
            // Replaced userButton.pressed() with digitalRead()
            if (digitalRead(USER_BUTTON) == LOW) { // Button is pressed
                delay(50); // Simple software debounce
                if (digitalRead(USER_BUTTON) == LOW) { // Check again
                    Serial.println("Run 1 Starting!");
                    currentState = MAPPING;
                    while(digitalRead(USER_BUTTON) == LOW); // Wait for button release
                }
            }
            break;

        case MAPPING: // Run 1: Follow line and map using LSRB
            runPID(); // Follow the line

            if (sensors.isIntersection()) {
                motors.stopBrake();
                motors.moveForward(TICKS_TO_CENTER); // Center on junction
                
                // LSRB Logic: Always try Left, then Straight, then Right. [2, 3, 4]
                
                // 1. Try Left
                motors.turn_90_left();
                delay(100); // Settle
                pidInput = sensors.getLineError();
                lineFoundAtIntersection = abs(pidInput) < 2000;

                if (lineFoundAtIntersection) {
                    Serial.println("Intersection: Turned L");
                    rawPath += 'L'; // Append 'L' to the String
                } else {
                    // 2. Try Straight (from original heading)
                    motors.turn_90_right(); // Return to center
                    delay(100);
                    pidInput = sensors.getLineError();
                    lineFoundAtIntersection = abs(pidInput) < 2000;

                    if (lineFoundAtIntersection) {
                        Serial.println("Intersection: Went S");
                        rawPath += 'S'; // Append 'S'
                    } else {
                        // 3. Try Right
                        motors.turn_90_right();
                        delay(100);
                        pidInput = sensors.getLineError();
                        lineFoundAtIntersection = abs(pidInput) < 2000;
                        
                        if (lineFoundAtIntersection) {
                            Serial.println("Intersection: Turned R");
                            rawPath += 'R'; // Append 'R'
                        } else {
                            // 4. Dead End (must be)
                            motors.turn_180_back(); // Already facing right, just turn 180
                            Serial.println("Intersection: Dead End, Turned B");
                            rawPath += 'B'; // Append 'B'
                        }
                    }
                }
            } 
            else if (sensors.isLineEnd()) {
                motors.stopBrake();
                optimizedPath = rawPath; // Copy the string
                
                Serial.println("--- Run 1 Finished ---");
                Serial.print("Raw Path: ");
                Serial.println(rawPath);
                
                currentState = OPTIMIZING;
            }
            break;

        case OPTIMIZING:
            Serial.println("Optimizing path...");
            optimizer.optimize(optimizedPath); // Optimize the String
            
            Serial.print("Optimized Path: ");
            Serial.println(optimizedPath);
            
            Serial.println("Place at START.");
            Serial.println("Press button to begin Run 2 (Solving)...");
            currentState = WAIT_FOR_RUN_2;
            pathIndex = 0; // Reset index for optimized path
            break;

        case WAIT_FOR_RUN_2:
            // Replaced userButton.pressed() with digitalRead()
            if (digitalRead(USER_BUTTON) == LOW) { // Button is pressed
                delay(50); // Simple software debounce
                if (digitalRead(USER_BUTTON) == LOW) { // Check again
                    Serial.println("Run 2 Starting!");
                    currentState = SOLVING;
                    while(digitalRead(USER_BUTTON) == LOW); // Wait for button release
                }
            }
            break;

        case SOLVING: // Run 2: Follow optimized path
            runPID(); // Follow the line

            if (sensors.isIntersection()) {
                motors.stopBrake();
                motors.moveForward(TICKS_TO_CENTER); // Center on junction

                // Read turn from String. This is safe.
                char turn = optimizedPath[pathIndex++];
                
                Serial.print("Intersection: Executing ");
                Serial.println(turn);

                if (turn == 'L') {
                    motors.turn_90_left();
                } 
                else if (turn == 'S') {
                    // Do nothing, just continue straight
                } 
                else if (turn == 'R') {
                    motors.turn_90_right();
                }
                // 'B' should not be in the optimized path.    
            }
            else if (sensors.isLineEnd()) {
                motors.stopBrake();
                Serial.println("--- MAZE SOLVED ---");
                currentState = FINISHED;
            }
            break;

        case FINISHED:
            // Blink LED to signify success
            digitalWrite(ONBOARD_LED,!digitalRead(ONBOARD_LED));
            delay(500);
            break;
            
        case CALIBRATING:
            // This state is handled in setup()
            break;
    }
}

// PID Control Loop Function
void runPID() {
    pidInput = sensors.getLineError();
    pid.Compute();
    
    int correction = (int)pidOutput;
    
    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;

    motors.setSpeeds(leftSpeed, rightSpeed);
}