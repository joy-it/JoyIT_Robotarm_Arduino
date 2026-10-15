#ifndef JOYIT_ROBOTARM_H
#define JOYIT_ROBOTARM_H

#include <DigitalIO.h>
#include <PsxControllerHwSpi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

class Robotarm{
    public:
        static constexpr uint8_t SERVO_COUNT = 6;
        
        Robotarm(Adafruit_PWMServoDriver& motorino, const uint8_t* channel);
        void begin();
        void useExternalController(bool external=true);
        void setAllServosCalibration(uint8_t servo, uint16_t minPulseUs, uint16_t maxPulseUs);
        void setPulseWidthOfAllServos(uint16_t minPulseUs, uint16_t maxPulseUs);
        void setAngleOfServo(uint8_t servo, uint8_t angle);
        void setAngleOfAllServos(uint8_t angle);
        bool getPositionOfServo(uint8_t servo, uint8_t& angle) const;
        void getPositionOfAllServo(uint8_t& angle) const;
        bool setHomePosition(uint8_t servo, uint8_t angle);
        bool resetPosition(uint8_t servo);
        bool resetAllPositions();
        bool moveTo(uint8_t servo, uint8_t angle, uint32_t durationMs);
        bool moveAllTo(const uint8_t* angles, uint32_t durationMs);
        void update();
        bool isMoving() const;
    
    private:
        Adafruit_PWMServoDriver& _motorino;

        static constexpr uint16_t DEFAULT_SERVO_MIN_US = 750;
        static constexpr uint16_t DEFAULT_SERVO_MAX_US = 2250;

        static constexpr uint32_t EXTERNAL_OSCILLATOR_HZ = 25000000UL;
        static constexpr uint16_t SERVO_FREQUENCY_HZ = 50;
        static constexpr uint8_t PRESCALE = (EXTERNAL_OSCILLATOR_HZ + 2048UL * SERVO_FREQUENCY_HZ) / (4096UL * SERVO_FREQUENCY_HZ) - 1;

        uint8_t _channels[SERVO_COUNT] = {};
        
        uint16_t _servoMinUs[SERVO_COUNT] = {};
        uint16_t _servoMaxUs[SERVO_COUNT] = {};

        uint8_t _positions[SERVO_COUNT] = {};
        bool _positionKnown[SERVO_COUNT] = {};

        uint8_t _homePositions[SERVO_COUNT] = {};
        bool _homeConfigured[SERVO_COUNT] = {};
        
        uint8_t _startAngles[SERVO_COUNT] = {};
        uint8_t _targetAngles[SERVO_COUNT] = {};

        uint32_t _movementStartMs[SERVO_COUNT] = {};
        uint32_t _movementDurationMs[SERVO_COUNT] = {};

        bool _moving[SERVO_COUNT] = {};

        bool _externalController = false;
        bool _controllerEnabled = false;
        bool _controllerReady = false;
        bool _initialized = false;
        uint32_t _lastControllerReadMs = 0;
        uint32_t _lastControlMs = 0;

        constexpr uint16_t _CONTROLLER_POLL_MS = 20;
        constexpr uint8_t _ANALOG_CENTER = 128;
        constexpr uint8_t _ANALOG_DEADZONE = 24;
        constexpr float _STICK_SPEED_DEG_PER_SEC = 45.0f;
        constexpr float _BUTTON_SPEED_DEG_PER_SEC = 60.0f;

        void _applyControllerAxis(uint8_t servo, float input, float dt);
        uint8_t _clampAngle(float angle);
        float _axisValue(uint8_t value);
};
#endif