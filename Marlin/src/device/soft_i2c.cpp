/*
 * Snapmaker2-Modules Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Modules
 * (see https://github.com/Snapmaker/Snapmaker2-Modules)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdexcept>
#include <board/board.h>
#include <io.h>
#include <src/HAL/hal_tim.h>
#include "src/device/soft_i2c.h"

void SoftI2C::Init(uint8_t sda, uint8_t scl) {
  i2c_sda_ = sda;
  i2c_scl_ = scl;
  pinMode(i2c_sda_, OUTPUT);
  digitalWrite(i2c_sda_, 1);
  pinMode(i2c_scl_, OUTPUT);
  digitalWrite(i2c_scl_, 1);
}

void SoftI2C::IICStart() {
  pinMode(i2c_sda_, OUTPUT);
  digitalWrite(i2c_sda_, 1);
  digitalWrite(i2c_scl_, 1);
  delay_us(5);
  digitalWrite(i2c_sda_, 0);
  delay_us(5);
  digitalWrite(i2c_scl_, 0);
}

void SoftI2C::IICStop() {
  pinMode(i2c_sda_, OUTPUT);
  digitalWrite(i2c_sda_, 0);
  digitalWrite(i2c_scl_, 0);
  delay_us(5);
  digitalWrite(i2c_scl_, 1);
  // delay_us(5);
  digitalWrite(i2c_sda_, 1);
  delay_us(5);
}

bool SoftI2C::IICWaitAck() {
  uint8_t wait_time = 0;
  pinMode(i2c_sda_, INPUT);
  digitalWrite(i2c_sda_, 1);
  delay_us(5);
  digitalWrite(i2c_scl_, 1);
  delay_us(5);
  while(digitalRead(i2c_sda_)) {
      wait_time++;
      if (wait_time > 250) {
          IICStop();
          return false;
      }
  }
  digitalWrite(i2c_scl_, 0);
  return true;
}

void SoftI2C::IICAck() {
  digitalWrite(i2c_scl_, 0);
  pinMode(i2c_sda_, OUTPUT);
  digitalWrite(i2c_sda_, 0);
  delay_us(5);
  digitalWrite(i2c_scl_, 1);
  delay_us(5);
  digitalWrite(i2c_scl_, 0);
}

void SoftI2C::IICNAck() {
  digitalWrite(i2c_scl_, 0);
  pinMode(i2c_sda_, OUTPUT);
  digitalWrite(i2c_sda_, 1);
  delay_us(5);
  digitalWrite(i2c_scl_, 1);
  delay_us(5);
  digitalWrite(i2c_scl_, 0);
}

void SoftI2C::IICSendByte(uint8_t data) {
  pinMode(i2c_sda_, OUTPUT);
  digitalWrite(i2c_scl_, 0);
  uint8_t i;
  for (i = 0; i < 8; i++) {
      digitalWrite(i2c_sda_, (data&0x80)>>7);
      data <<= 1;
      delay_us(5);
      digitalWrite(i2c_scl_, 1);
      delay_us(5);
      digitalWrite(i2c_scl_, 0);
      delay_us(5);
  }
}

// send_ack == true, send ack else send Nack
uint8_t SoftI2C::IICReadByte(bool send_ack) {
  uint8_t i;
  uint8_t receive = 0;
  pinMode(i2c_sda_, INPUT);
  for (i = 0; i < 8; i++) {
      digitalWrite(i2c_scl_, 0);
      delay_us(5);
      digitalWrite(i2c_scl_, 1);
      receive <<= 1;
      if (digitalRead(i2c_sda_)) {
          receive++;
      }
      delay_us(5);
  }

  if (send_ack) {
      IICAck();
  } else {
      IICNAck();
  }

  return receive;
}
