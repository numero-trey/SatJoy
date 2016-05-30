/*
 * SatJoy - DSMX Satellite -> USB Joystick
 * v0.1 "Alpha AF"
 * Copyright 2016 C Chandler
 *
 * This program is free software: you can redistribute it and/or modify it under 
 * the terms of the GNU General Public License as published by the Free Software 
 * Foundation, either version 3 of the License, or (at your option) any later 
 * version. 
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT 
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS 
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more 
 * details. 
 *
 * You should have received a copy of the GNU General Public License along with 
 * this program.  If not, see
 * <http://www.gnu.org/licenses/>.
 */

#include "HID.h"

#define JOYSTICK_CHANNELS 8
#define JOYSTICK_STATE_SIZE JOYSTICK_CHANNELS * 2
#define JOYSTICK_REPORT_ID 0x03

#define SPEKTRUM_BAUDRATE 115200
#define SPEKTRUM_PACKET_SIZE 16
#define SPEKTRUM_PACKET_CHANNELS 7
#define SPEKTRUM_PACKET_TIMEOUT_MS 4

#if !defined(USBCON)
#error Native USB not found. Arduino Micro or ATmega32U4 required.
#endif

// HID descriptor for our Joystick
static const uint8_t _hidReportDescriptor[] PROGMEM = {
  // Joystick
  0x05, 0x01,           // USAGE_PAGE (Generic Desktop)
  0x09, 0x04,           // USAGE (Joystick)
  0xa1, 0x01,           // COLLECTION (Application)
  0x85, JOYSTICK_REPORT_ID,  //   REPORT_ID (3)
  0x05, 0x01,           //   USAGE_PAGE (Generic Desktop)
  0x15, 0x00,           //   LOGICAL_MINIMUM (0)
  0x26, 0xff, 0x07,     //   LOGICAL_MAXIMUM (2047)
  0x75, 0x10,           //   REPORT_SIZE (16)
  0x09, 0x01,           //   USAGE (Pointer)
  0xA1, 0x00,           //   COLLECTION (Physical)
  0x09, 0x30,           //     USAGE (x)
  0x09, 0x31,           //     USAGE (y)
  0x09, 0x32,           //     USAGE (z)
  0x09, 0x33,           //     USAGE (rx)
  0x09, 0x34,           //     USAGE (ry)
  0x09, 0x35,           //     USAGE (rz)
  0x09, 0x36,           //     USAGE (slider)
  0x09, 0x37,           //     USAGE (dial)
  0x95, JOYSTICK_CHANNELS,  //     REPORT_COUNT (8)
  0x81, 0x02,           //     INPUT (Data,Var,Abs)
  0xc0,                 //   END_COLLECTION
  0xc0                  // END_COLLECTION
};

// Map of receiver channels to joystick channels
// Index is DSMX channel, value is HID report index
const uint8_t channelMapping[] = {
  3, // 0 -> Throttle
  0, // 1 -> Aileron
  1, // 2 -> Elevator
  2, // 3 -> Rudder
  4, // 4 -> Gear
  5, // 5 -> Aux 1
  6, // 6 -> Aux 2
  7, // 7 -> Aux 3
};

int16_t outputChannels[JOYSTICK_CHANNELS];
uint8_t radioPacket[SPEKTRUM_PACKET_SIZE];

void setup() {
  // Init HID Joystick
  static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);

  // Debugging Serial
  Serial.begin(9600);

  // Spektrum satellite
  Serial1.begin(SPEKTRUM_BAUDRATE);
  // Timeout is how we sync to the datastream (packet every 11ms)
  Serial1.setTimeout(SPEKTRUM_PACKET_TIMEOUT_MS);

  // Init output channels
  for (size_t i = 0; i < JOYSTICK_CHANNELS; i++) {
    outputChannels[i] = 0;
  }

  // Send report
  sendJoyReport();
}

void loop() {
   uint8_t ch;
   uint16_t val;
   
  // Wait to receive a whole 16 byte packet within our window
  if (Serial1.readBytes(radioPacket, SPEKTRUM_PACKET_SIZE) == SPEKTRUM_PACKET_SIZE) {
    // Check for correct format ID
    if (radioPacket[1] != 0xb2) {
      return;
    }
    // Start processing byte pairs, skipping the preamble
    for (uint8_t i = 1; i <= SPEKTRUM_PACKET_CHANNELS; i++) {
      ch = dsmxChannelID(radioPacket[i * 2]);
      val = dsmxChannelValue(radioPacket[i * 2], radioPacket[i * 2 + 1]);

      // Make sure we are not receiving a channel we don't support
      if (ch < JOYSTICK_CHANNELS) {
        outputChannels[channelMapping[ch]] = val;
      }
      sendJoyReport();
    }
  }
}

// Shift & mask bits 3-6 for the channel #
uint8_t dsmxChannelID(uint8_t highByte) {
  return (highByte >> 3) & 0b1111;
}

// Calculate channel value from low byte and low 3 bits of the high byte
uint16_t dsmxChannelValue(uint8_t highByte, uint8_t lowByte) {
  return ((highByte & 0b111) << 8) + lowByte;
}

void sendJoyReport() {
  uint8_t data[JOYSTICK_STATE_SIZE];
  // Split 16 bit values into 2 bytes
  for (size_t i = 0; i < JOYSTICK_CHANNELS; i++) {
    data[2 * i] = outputChannels[i] & 0xFF;
    data[2 * i + 1] = outputChannels[i] >> 8;
  }
  HID().SendReport(JOYSTICK_REPORT_ID, data, JOYSTICK_STATE_SIZE);
}
