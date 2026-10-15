#include "Robotarm.h"

Robotarm::Robotarm(Adafruit_PWMServoDriver& motorino, const uint8_t* channel)
    :_motorino(motorino) {
        for(uint8_t i = 0; i < SERVO_COUNT; i++){
            _channels[i] = channel[i];
            _servoMinUs[i] = DEFAULT_SERVO_MIN_US;
            _servoMaxUs[i] = DEFAULT_SERVO_MAX_US;
            _positions[i] = 90;
            _startAngles[i] = 90;
            _targetAngles[i] = 90;
            _positionKnown[i] = false;
            _moving[i] = false;
            _movementStartMs[i] = 0;
            _movementDurationMs[i] = 0;
        }
}



Robotarm::_clampAngle(float angle){
    if (angle < 0.0f) return 0;
    if (angle > 180.0f) return 180;
    return static_cast<uint8_t>(angle + 0.5f);
}

Robotarm::_axisValue(uint8_t value){
    int16_t offset = static_cast<int16_t>(value) - _ANALOG_CENTER;
    if (abs(offset) <= _ANALOG_DEADZONE) return 0.0f;
    return static_cast<float>(offset) / 127.0f;
}