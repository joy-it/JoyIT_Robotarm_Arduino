#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Robotarm.h>

Adafruit_PWMServoDriver motorino(0x40);

// Motorino channels connected to the six servos.
// Adjust these numbers to match your wiring.
// values are sort from bottom to top
const uint8_t servoChannels[Robotarm::SERVO_COUNT] = {
    0, 1, 2, 3, 4, 5
};

// set up robot arm with Motorino and used channels
Robotarm robotarm(motorino, servoChannels);

// constant for duration of movement and duration of pauses in between
constexpr uint32_t MOVEMENT_DURATION_MS = 2500;
constexpr uint32_t PAUSE_DURATION_MS = 2000;

// Angle offsets relative to the configured home positions.
// These are example movements; adjust them to your robot arm.
const int8_t poseOffsets[][Robotarm::SERVO_COUNT] = {
    { 30, -30,  30, -30,  30, -30 },
    {-30,  20, -20,  30, -30,  20 },
    {  0,   0,   0,   0,   0,   0 }
};
// amount of poses
constexpr uint8_t POSE_COUNT = sizeof(poseOffsets) / sizeof(poseOffsets[0]);

// angles of motors at the start
uint8_t homeAngles[Robotarm::SERVO_COUNT];

// counter for poses
uint8_t nextPose = 0;

// variables to controll duration of pauses and movement 
bool waiting = true;
uint32_t pauseStartMs = 0;


void setup() {
    // setup serial monitor
    Serial.begin(115200);
    Wire.begin();

    // begin communication with Motorino
    if (!robotarm.begin()) {
        Serial.println(F("Motorino initialization failed."));
        while (true) {
            delay(100);
        }
    }

    // set all motors to home position
    robotarm.resetAllPositions();

    // save positions of robot arm into desired array
    robotarm.getPositionOfAllServo(homeAngles);
    // set variable to current time
    pauseStartMs = millis();
}

void loop() {
    // Keep timed movements running
    robotarm.update();

    // check if movement of robot is finished
    if (waiting) {
        // check if time of pause is passed
        if (millis() - pauseStartMs < PAUSE_DURATION_MS) return;

        // array to save target angles into
        uint8_t targetAngles[Robotarm::SERVO_COUNT];

        // set target angles to the offset values with the current position 
        for (uint8_t i = 0; i < Robotarm::SERVO_COUNT; i++) {
            const int16_t angle = static_cast<int16_t>(homeAngles[i]) + poseOffsets[nextPose][i];
            targetAngles[i] = constrain(angle, 0, 180);
        }
        // set movement of all motors and select next pose
        if (robotarm.moveAllTo(targetAngles, MOVEMENT_DURATION_MS)) {
            nextPose = (nextPose + 1) % POSE_COUNT;
            waiting = false;
        }
        return;
    }
    // if movement is finished reset values
    if (!robotarm.isMoving()) {
        pauseStartMs = millis();
        waiting = true;
    }
}