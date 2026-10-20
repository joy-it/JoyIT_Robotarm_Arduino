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
}

void loop(){
  robotarm.setAngleOfServo(0, 20);
  delay(1000);

  robotarm.setAngleOfServo(0, 160);
  delay(1000);
}