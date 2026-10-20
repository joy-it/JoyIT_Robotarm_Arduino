#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Robotarm.h>

// Motorino with I2C address 0x40
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
    // set pulse width of all used servo motors
    robotarm.setPulseWidthOfAllServos(500, 2500);

    // set all motors (except the claw) to neutral position
    for (uint8_t i = 0; i < Robotarm::SERVO_COUNT - 1; i++){
        motorino.writeMicroseconds(servoChannels[i], 1500);
    }

    // set claw to a slighty different position
    motorino.writeMicroseconds(servoChannels[Robotarm::SERVO_COUNT - 2], 1600);
}

void loop() {
    // Nothing happens here
}
