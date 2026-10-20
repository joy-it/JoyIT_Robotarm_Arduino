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

    // set all motors to home position
    robotarm.resetAllPositions();

    // activate use with controller
    robotarm.useExternalController(true);


}

// variable t slow output down
uint32_t lastPrint = 0;

void loop() {
    // Poll the gamepad and update servo movements
    robotarm.update();
    // output position of motors every second
    if (millis() - lastPrint >= 1000){
        lastPrint = millis();
        // print position of motors
        for (uint8_t i = 0; i < Robotarm::SERVO_COUNT; i++){
            uint8_t angle;
            Serial.print("Servo ");
            Serial.print(i);
            Serial.print(": ");
            if (robotarm.getPositionOfServo(i, angle)){
                Serial.print(angle);
                Serial.print(" deg");
            }
            // if motor position is still unkown
            else Serial.print("unkown");
            if (i < Robotarm::SERVO_COUNT - 1) Serial.print(" | ");
        }
        // next line for next positions
        Serial.println();
    }
}