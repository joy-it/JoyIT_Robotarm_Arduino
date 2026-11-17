/*
  Robotarm.cpp - Library for the Joy-IT robot arm and ARD-Moto3
  Created by Joy-IT, October 20, 2026
  Released into the public domain.
*/

#include "Robotarm.h"

Robotarm::Robotarm(Adafruit_PWMServoDriver& motorino, const uint8_t* channel)
    :_motorino(motorino){
        for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
            this->_channel[i] = channel[i];
            this->_servoMinUs[i] = this->_DEFAULT_SERVO_MIN_US;
            this->_servoMaxUs[i] = this->_DEFAULT_SERVO_MAX_US;
            this->_position[i] = 90;
            this->_startAngle[i] = 90;
            this->_targetAngle[i] = 90;
            this->_positionKnown[i] = false;
            this->_moving[i] = false;
            this->_movementStartMs[i] = 0;
            this->_movementDurationMs[i] = 0;
        }
}

bool Robotarm::begin(){
    if (!this->_motorino.begin(this->_PRESCALE)) return false;
    this->_motorino.setOscillatorFrequency(this->_EXTERNAL_OSCILLATOR_HZ);
    this->_initialized = true;
    if (this->_controllerEnabled) this->useController(this->_controllerEnabled);
    return true;
}

bool Robotarm::useController(bool external){
    this->_controllerEnabled = external;
    this->_controllerReady = false;
    this->_controllerFound = false;
    if (!external) return true;
    if (!this->_initialized) return true;
    return this->_connectController(millis());
}

bool Robotarm::isControllerConnected(){
    return this->_controllerEnabled && this->_controllerReady;
}

bool Robotarm::setServoCalibration(uint8_t servo, uint16_t minPulseUs, uint16_t maxPulseUs){
    if (servo < 0 || servo >= this->SERVO_COUNT || minPulseUs < 0 || maxPulseUs <= minPulseUs) return false;
    this->_servoMinUs[servo] = minPulseUs;
    this->_servoMaxUs[servo] = maxPulseUs;
    return true;
}

bool Robotarm::setPulseWidthOfAllServos(uint16_t minPulseUs, uint16_t maxPulseUs){
    bool state = false;
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
        state = this->setServoCalibration(i, minPulseUs, maxPulseUs);
        if (!state) return state;
    }
    return state;
}

void Robotarm::setAngleOfServo(uint8_t servo, uint8_t angle){
    if (servo < 0 || servo >= this->SERVO_COUNT) return;
    angle = constrain(angle, 0, 180);
    uint16_t pulseUs = map(angle, 0, 180, this->_servoMinUs[servo], this->_servoMaxUs[servo]);
    this->_motorino.writeMicroseconds(this->_channel[servo], pulseUs);
    this->_position[servo] = angle;
    this->_positionKnown[servo] = true;
    this->_moving[servo] = false;
}

void Robotarm::setAngleOfAllServos(uint8_t angle){
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
        this->setAngleOfServo(i, angle);
    }
}

bool Robotarm::getPositionOfServo(uint8_t servo, uint8_t& angle){
    if (servo >= this->SERVO_COUNT || servo < 0 || !_positionKnown[servo]) return false;
    else {
        angle = _position[servo];
        return true;
    }
}

void Robotarm::getPositionOfAllServo(uint8_t* angles){
    if (angles == nullptr) return;
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
        this->getPositionOfServo(i, angles[i]);
    }
}

void Robotarm::setHomePosition(uint8_t servo, uint8_t angle){
    if (servo < 0 || servo >= this->SERVO_COUNT) return;
    angle = constrain(angle, 0, 180);
    this->_homePosition[servo] = angle;
}

void Robotarm::resetPosition(uint8_t servo){
    if (servo < 0 || servo >= this->SERVO_COUNT) return;
    this->setAngleOfServo(servo, this->_homePosition[servo]);
}

void Robotarm::resetAllPositions(){
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
        this->resetPosition(i);
    }
}

bool Robotarm::moveTo(uint8_t servo, uint8_t angle, uint32_t durationMs){
    if (servo >= this->SERVO_COUNT || servo < 0 || !_positionKnown[servo]) return false;
    angle = constrain(angle, 0, 180);
    if (durationMs == 0){
        this->setAngleOfServo(servo, angle);
        return true;
    }
    this->_startAngle[servo] = this->_position[servo];
    this->_targetAngle[servo] = angle;
    this->_movementStartMs[servo] = millis();
    this->_movementDurationMs[servo] = durationMs;
    this->_moving[servo] = true;
    return true;
}

bool Robotarm::moveAllTo(const uint8_t* angles, uint32_t durationMs){
    if (angles == nullptr) return false;
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++) {
        if (!_positionKnown[i]) return false;
    }
    bool success = true;
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++) {
        if (!this->moveTo(i, angles[i], durationMs)) success = false;
    }
    return success;
}

void Robotarm::update(){
    if (!this->_initialized) return;
    const uint32_t now = millis();
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
        if (!this->_moving[i]) continue;
        const uint32_t elapsed = now - this->_movementStartMs[i];
        if (elapsed >= this->_movementDurationMs[i]){
            this->setAngleOfServo(i, this->_targetAngle[i]);
            continue;
        }
        const float progress = static_cast<float>(elapsed) / this->_movementDurationMs[i];
        const float angle = this->_startAngle[i] + (static_cast<int16_t>(this->_targetAngle[i]) - static_cast<int16_t>(this->_startAngle[i])) * progress;
        this->setAngleOfServo(i, this->_clampAngle(angle));
        this->_moving[i] = true;
    }
    if (!this->_controllerEnabled) return;
    if (!this->_controllerReady){
        if (this->_controllerFound){
            if (now - this->_controllerFoundMs >= this->_CONTROLLER_SETTLE_MS) this->_configureController(now);
        }
        else if (now - this->_lastControllerRetryMs >= this->_CONTROLLER_RETRY_MS){
            this->_connectController(now);
        }
        return;
    }
    if (now - this->_lastControllerReadMs < this->_CONTROLLER_POLL_MS) return;
    float dt = (now - this->_lastControlMs) / 1000.0f;
    if (dt > this->_CONTROLLER_MAX_DT) dt = this->_CONTROLLER_MAX_DT;
    this->_lastControllerReadMs = now;
    this->_lastControlMs = now;
    if (!this->_controller.read()){
        this->_controllerReady = false;
        this->_lastControllerRetryMs = now;
        return;
    }
    byte x = this->_ANALOG_CENTER;
    byte y = this->_ANALOG_CENTER;
    if (this->_controller.getLeftAnalog(x, y)){
        this->_applyControllerAxis(0, this->_axisValue(x), dt);
        this->_applyControllerAxis(1, -this->_axisValue(y), dt);
    }
    if (this->_controller.getRightAnalog(x, y)){
        this->_applyControllerAxis(2, -this->_axisValue(y), dt);
        this->_applyControllerAxis(3, this->_axisValue(x), dt);
    }
    const float buttonStep = this->_BUTTON_SPEED_DEG_PER_SEC * dt;
    if (this->_controller.buttonPressed(PSB_L1)) this->_applyControllerAxis(4, 1.0f, dt);
    else if (this->_controller.buttonPressed(PSB_R1)) this->_applyControllerAxis(4, -1.0f, dt);
    if (this->_controller.buttonPressed(PSB_L2)) this->_applyControllerAxis(5, 1.0f, dt);
    else if (this->_controller.buttonPressed(PSB_R2)) this->_applyControllerAxis(5, -1.0f, dt);
    (void) buttonStep;
}

bool Robotarm::isMoving(){
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++){
        if (this->_moving[i]) return true;
    }
    return false;
}

void Robotarm::_applyControllerAxis(uint8_t servo, float input, float dt){
    if (servo >= this->SERVO_COUNT || servo < 0 || input == 0.0f) return;
    const float nextAngle = this->_position[servo] + this->_controllerRemainder[servo]
                          + input * this->_STICK_SPEED_DEG_PER_SEC * dt;
    const uint8_t angle = this->_clampAngle(nextAngle);
    this->_controllerRemainder[servo] = (nextAngle <= 0.0f || nextAngle >= 180.0f) ? 0.0f : nextAngle - angle;
    this->setAngleOfServo(servo, angle);
}

uint8_t Robotarm::_clampAngle(float angle){
    if (angle < 0.0f) return 0;
    if (angle > 180.0f) return 180;
    return static_cast<uint8_t>(angle + 0.5f);
}

float Robotarm::_axisValue(uint8_t value){
    int16_t offset = static_cast<int16_t>(value) - this->_ANALOG_CENTER;
    if (abs(offset) <= this->_ANALOG_DEADZONE) return 0.0f;
    return static_cast<float>(offset) / 127.0f;
}

bool Robotarm::_connectController(uint32_t now){
    this->_lastControllerRetryMs = now;
    if (!this->_controller.begin()) return false;
    this->_controllerFound = true;
    this->_controllerFoundMs = now;
    return true;
}

void Robotarm::_configureController(uint32_t now){
    if (this->_controller.enterConfigMode()){
        this->_controller.enableAnalogSticks();
        this->_controller.exitConfigMode();
    }
    for (uint8_t i = 0; i < this->SERVO_COUNT; i++) this->_controllerRemainder[i] = 0.0f;
    this->_controllerFound = false;
    this->_controllerReady = true;
    this->_lastControllerReadMs = now;
    this->_lastControlMs = now; 
}