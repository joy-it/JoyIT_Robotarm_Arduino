/*
  WLGamepadUnoQ.h - Bit-banged driver for the Joy-IT SBC-WLGamepad receiver
  on the Arduino UNO Q, for the use with the Joy-IT Robot Arm library.

  Copyright (c) 2026 Joy-IT
  SPDX-License-Identifier: MIT
  See the LICENSE file in the root of this repository for the full license text.

  Based on PsxNewLib by SukkoPera (https://github.com/SukkoPera/PsxNewLib):
    - Derives from PsxController and follows the structure of PsxControllerBitBang.
    - Uses DigitalIO by William Greiman (https://github.com/greiman/DigitalIO) for pin access. 
  Both libraries are licensed under GPL-3.0-or-later.
  Programs that are compiled and distributed together with PsxNewLib are subject
  to the terms of the GPL.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.
*/

#ifndef WL_GAMEPAD_UNO_Q
#define WL_GAMEPAD_UNO_Q

#include <Arduino.h>
#include <PsxNewLib.h>
#include <DigitalIO.h>

template <uint8_t ATT, uint8_t CMD, uint8_t DATA, uint8_t CLK>
class WLGamepadUnoQ : public PsxController {
    public:
        virtual boolean begin() override {
            this->_attention.config(OUTPUT, HIGH);
            this->_command.config(OUTPUT, HIGH);
            this->_clock.config(OUTPUT, HIGH);
            this->_data.mode(INPUT_PULLUP);
            return PsxController::begin();
        }

    protected:
        virtual void attention() override {
            this->_attention.low();
            delayMicroseconds(this->_ATTN_DELAY_US);
        }

        virtual void noAttention() override {
            this->_command.high();
            this->_clock.high();
            this->_attention.high();
            delayMicroseconds(this->_ATTN_DELAY_US);
        }

        virtual byte shiftInOut(const byte out) override {
            byte in = 0;
            for (byte i = 0; i < 8; i++) {
                this->_command = (out >> i) & 0x01;
                this->_clock.low();
                delayMicroseconds(this->_HALF_PERIOD_US);
                if (this->_data) in |= (1 << i);
                this->_clock.high();
                delayMicroseconds(this->_HALF_PERIOD_US);
            }
            return in;
        }

    private:
        static constexpr uint8_t _HALF_PERIOD_US = 5; 
        static constexpr uint8_t _ATTN_DELAY_US = 50; 

        DigitalPin<ATT> _attention;
        DigitalPin<CLK> _clock;
        DigitalPin<CMD> _command;
        DigitalPin<DATA> _data;

};

#endif