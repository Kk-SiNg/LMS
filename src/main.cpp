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

// === Tuning Constants ===
#define MAX_PATH_LENGTH 100    // Max number of segments
#define HIGH_SPEED 255         // Full throttle speed for Run 2
#define BASE_SPEED 150         // Normal mapping and turning speed
#define SLOWDOWN_TICKS 500     // How many ticks before intersection to slow down


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
// === Path & Distance Storage ===
String rawPath = "";
long pathSegments[10000];   //taking large size of array for storing path lengths to avoid overflow.
int pathIndex = 0;

String optimizedPath = "";
long optimizedSegments[10000];
int optimizedPathLength = 0;
int solvePathIndex = 0; // Index used during the solving run

// === Solving Sub-State Machine ===
enum SolvingSubState {
    SOLVE_TURN,
    SOLVE_FAST_RUN,
    SOLVE_SLOW_RUN,
    SOLVE_FINAL_RUN
};
SolvingSubState solveState = SOLVE_TURN;

// === PID Controller ===
// These constants MUST be tuned.
float Kp = 0.05, Ki = 0.0001, Kd = 0.02; 
float pidInput, pidOutput, pidSetpoint = 0;
// Base speed (0-255). Tune this for your robot.
int baseSpeed = 150;
QuickPID pid(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// // === FSM Helper ===
// bool lineFoundAtIntersection = false;


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

void runPID(int currentBaseSpeed);

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
            runPID(BASE_SPEED); // Follow the line

            if (sensors.isIntersection()) {
                motors.stopBrake();

                // Get distance of the segment just traveled
                long segmentTicks = motors.getAverageCount();

                // NEW "SEEING" LSRB LOGIC [16, 13, 14]
                // 1. Check all available paths at once(left and right)
                PathOptions paths = sensors.getOpenPaths();

                motors.moveForward(TICKS_TO_CENTER); // Center on junction

                uint16_t values[8];         //get the sensors values after moving forward ---> for availability of straight path
                sensors.readRaw(values);

                // 2. Apply LSRB (Left-Hand-on-the-Wall) priority
                if (paths.left) {
                    Serial.println("Intersection: Path open Left. Turning L.");
                    motors.turn_90_left();
                    rawPath += 'L'; // Append 'L' to the String
                }
                else if (values[3] > 800 || values[4] > 800) {
                    Serial.println("Intersection: Path open Straight. Going S.");
                    // No turn needed, just continue
                    rawPath += 'S'; // Append 'S'
                }
                else if (paths.right) {
                    Serial.println("Intersection: Path open Right. Turning R.");
                    motors.turn_90_right();
                    rawPath += 'R'; // Append 'R'
                }
                else {
                    // 4. Dead End (no paths open)
                    Serial.println("Intersection: Dead End. Turning B.");
                    motors.turn_180_back();
                    rawPath += 'B'; // Append 'B'
                }


                //Store the distance for the turn
                pathSegments[pathIndex] = segmentTicks;
                pathIndex++;
                
                //Clear encoders to start counting the *next* segment
                motors.clearEncoders();
                delay(100); // Settle

            }
            else if (sensors.isLineEnd()) {
                motors.stopBrake();
                optimizedPath = rawPath; // Copy the string
                optimizedPathLength = optimizedPath.length();

                Serial.println("--- Run 1 Finished ---");
                Serial.print("Raw Path: ");
                Serial.println(rawPath);
                
                currentState = OPTIMIZING;
            }
            break;

        case OPTIMIZING:
        {
            Serial.println("Optimizing path...");
            bool changesMade = true;
            
            while(changesMade) {
                int oldLength = optimizedPathLength;
                optimizer.optimize(optimizedPath, optimizedSegments, optimizedPathLength);
                changesMade = (oldLength != optimizedPathLength);
            }
            
            Serial.print("Optimized Path: "); Serial.println(optimizedPath);
            Serial.println("Optimized Segments:");
            for(int i = 0; i < optimizedPathLength; i++) {
                Serial.print(optimizedPath[i]);
                Serial.print(": ");
                Serial.println(optimizedSegments[i]);
            }
            
            Serial.println("Place at START.");
            Serial.println("Press button to begin Run 2 (Solving)...");
            currentState = WAIT_FOR_RUN_2;
            break;
        }


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

        case SOLVING: // Run 2: High-speed, distance-based execution
            
            if (solveState == SOLVE_TURN) {
                // We are at an intersection, ready to execute a turn.
                char turn = optimizedPath[solvePathIndex];
                
                Serial.print("At intersection. Executing: "); Serial.println(turn);

                if (turn == 'L') motors.turn_90_left();
                else if (turn == 'R') motors.turn_90_right();
                // For 'S' (Straight), we do nothing.

                // Check if this was the last segment
                if (solvePathIndex >= optimizedPathLength - 1) {
                    solveState = SOLVE_FINAL_RUN;
                    Serial.println("Last segment. Running to finish.");
                }
                else {
                    // Not the last segment. Clear encoders and start fast run.
                    motors.clearEncoders();
                    solveState = SOLVE_FAST_RUN;
                    Serial.println("Entering FAST RUN.");
                }
                delay(100); // Settle after turn

            }
            else if (solveState == SOLVE_FAST_RUN) {
                long currentTicks = motors.getAverageCount();
                long targetTicks = optimizedSegments[solvePathIndex + 1]; // Get *next* segment's length

                if (currentTicks < (targetTicks - SLOWDOWN_TICKS)) {
                    runPID(HIGH_SPEED);
                }
                else {
                    // We're close. Transition to slow run.
                    solveState = SOLVE_SLOW_RUN;
                    Serial.println("Entering SLOW RUN.");
                }
            
            }
            else if (solveState == SOLVE_SLOW_RUN) {
                long currentTicks = motors.getAverageCount();
                long targetTicks = optimizedSegments[solvePathIndex + 1];

                if (currentTicks < targetTicks) {
                    runPID(BASE_SPEED);
                }
                else {
                    // We have arrived at the next intersection.
                    motors.stopBrake();
                    motors.moveForward(TICKS_TO_CENTER); // Re-center
                    solvePathIndex++; // Move to the next path command
                    solveState = SOLVE_TURN; // Go back to turning logic
                    Serial.println("Arrived at intersection.");
                }
            
            }
            else if (solveState == SOLVE_FINAL_RUN) {
                // Just follow the line until the end is detected
                runPID(BASE_SPEED);
                if (sensors.isLineEnd()) {
                    motors.stopBrake();
                    Serial.println("--- MAZE SOLVED ---");
                    currentState = FINISHED;
                }
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
void runPID(int currentBaseSpeed) {
    pidInput = sensors.getLineError();
    pid.Compute();
    
    int correction = (int)pidOutput;
    
    int leftSpeed = currentBaseSpeed + correction;
    int rightSpeed = currentBaseSpeed - correction;

    motors.setSpeeds(leftSpeed, rightSpeed);
}