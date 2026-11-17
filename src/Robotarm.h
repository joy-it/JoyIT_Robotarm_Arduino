/*
  Robotarm.h - Library for the Joy-IT robot arm and ARD-Moto3
  Created by Joy-IT, October 20, 2026
  Released into the public domain.
*/

#ifndef JOYIT_ROBOTARM_H
#define JOYIT_ROBOTARM_H

#include <Arduino.h> 
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#if defined(ARDUINO_ARCH_ZEPHYR)
#include "WLGamepadUnoQ.h"
#else
#include <PsxControllerHwSpi.h>
#endif


class Robotarm{
    public:
        static constexpr uint8_t SERVO_COUNT = 6;
        
        Robotarm(Adafruit_PWMServoDriver& motorino, const uint8_t* channel);
        bool begin();
        bool useController(bool external=true);
        bool isControllerConnected();
        bool setServoCalibration(uint8_t servo, uint16_t minPulseUs, uint16_t maxPulseUs);
        bool setPulseWidthOfAllServos(uint16_t minPulseUs, uint16_t maxPulseUs);
        void setAngleOfServo(uint8_t servo, uint8_t angle);
        void setAngleOfAllServos(uint8_t angle);
        bool getPositionOfServo(uint8_t servo, uint8_t& angle);
        void getPositionOfAllServo(uint8_t* angles);
        void setHomePosition(uint8_t servo, uint8_t angle);
        void resetPosition(uint8_t servo);
        void resetAllPositions();
        bool moveTo(uint8_t servo, uint8_t angle, uint32_t durationMs);
        bool moveAllTo(const uint8_t* angles, uint32_t durationMs);
        void update();
        bool isMoving();
    
    private:
        Adafruit_PWMServoDriver& _motorino;
        #if defined(ARDUINO_ARCH_ZEPHYR)
        WLGamepadUnoQ<10, 11, 12, 13> _controller;
        #else
        PsxControllerHwSpi<10> _controller;
        #endif

        static constexpr uint16_t _DEFAULT_SERVO_MIN_US = 500;
        static constexpr uint16_t _DEFAULT_SERVO_MAX_US = 2500;

        static constexpr uint32_t _EXTERNAL_OSCILLATOR_HZ = 25000000UL;
        static constexpr uint16_t _SERVO_FREQUENCY_HZ = 50;
        static constexpr uint8_t _PRESCALE = (_EXTERNAL_OSCILLATOR_HZ + 2048UL * _SERVO_FREQUENCY_HZ) / (4096UL * _SERVO_FREQUENCY_HZ) - 1;

        uint8_t _channel[SERVO_COUNT] = {};
        
        uint16_t _servoMinUs[SERVO_COUNT] = {};
        uint16_t _servoMaxUs[SERVO_COUNT] = {};

        uint8_t _position[SERVO_COUNT] = {};
        bool _positionKnown[SERVO_COUNT] = {};

        uint8_t _homePosition[SERVO_COUNT] = {90, 90, 20, 110, 30, 150};   
        
        uint8_t _startAngle[SERVO_COUNT] = {};
        uint8_t _targetAngle[SERVO_COUNT] = {};

        uint32_t _movementStartMs[SERVO_COUNT] = {};
        uint32_t _movementDurationMs[SERVO_COUNT] = {};

        bool _moving[SERVO_COUNT] = {};

        bool _externalController = false;
        bool _controllerEnabled = false;
        bool _controllerReady = false;
        bool _controllerFound = false;
        bool _initialized = false;
        uint32_t _lastControllerReadMs = 0;
        uint32_t _lastControlMs = 0;
        uint32_t _controllerFoundMs = 0;
        uint32_t _lastControllerRetryMs = 0;
        float _controllerRemainder[SERVO_COUNT] = {};
        static constexpr uint16_t _CONTROLLER_RETRY_MS = 250;
        static constexpr uint16_t _CONTROLLER_SETTLE_MS = 300;
        static constexpr float _CONTROLLER_MAX_DT = 0.1f;

        static constexpr uint16_t _CONTROLLER_POLL_MS = 20;
        static constexpr uint8_t _ANALOG_CENTER = 128;
        static constexpr uint8_t _ANALOG_DEADZONE = 24;
        static constexpr float _STICK_SPEED_DEG_PER_SEC = 45.0f;
        static constexpr float _BUTTON_SPEED_DEG_PER_SEC = 60.0f;

        void _applyControllerAxis(uint8_t servo, float input, float dt);
        uint8_t _clampAngle(float angle);
        float _axisValue(uint8_t value);
        bool _connectController(uint32_t now);
        void _configureController(uint32_t now);
};
#endif