/**
 * PRDC_TMAESC.h - T-Motor ALPHA ESC Telemetry Protocol Arduino library
 * Author: Veljko Petrovic <vpetrovic@pr-dc.com>
 * PR-DC, Republic of Serbia
 * info@pr-dc.com
 * 
 * --------------------
 * Copyright (C) 2021 PR-DC <info@pr-dc.com>
 *
 * This file is part of PRDC_TMAESC.
 *
 * PRDC_TMAESC is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as 
 * published by the Free Software Foundation, either version 3 of the 
 * License, or (at your option) any later version.
 * 
 * PRDC_TMAESC is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with PRDC_TMAESC.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef _PRDC_TMAESC_H_

#define _PRDC_TMAESC_H_

#include "Arduino.h"

#define PRDC_TMAESC_B1 0x9B // byte 1 - header
#define PRDC_TMAESC_B2 0x16 // byte 2 - packet size
#define PRDC_TMAESC_PACKET_SIZE 24 // 22 byte packet + 2 byte checksum
#define PRDC_TMAESC_BAUD 19200 // fixed baudrate

class PRDC_TMAESC {
  public:
    void init(Stream& stream); // initalizes communication
    void init(HardwareSerial& serial);
    bool update(); // reads and parses a packet if available
    void set_poles(uint8_t p); // sets BLDC motor pole number

    // readable telemetry data
    struct data_t {
      uint32_t time; // packet time (since MCU boot)
      uint16_t counter; // packet counter (on the ESC side)
      float throttle_rx; // received throttle in %
      float throttle_out; // actual output throttle in %
      uint16_t rpm; // motor RPM
      float voltage_bus; // bus bar voltage in V
      float current_bus; // bus bar current in A
      float current_phase; // phase current in A
      uint8_t temperature_mos; // MOSFET temperature in degC
      uint8_t temperature_cap; // capactitor temperature in degC
      uint16_t status; // status bits
      bool fault; // fault flag (true if any status bit set)
    } data;

  private:
    uint16_t calc_checksum(uint8_t* data, uint16_t n); // calculates checksum by summing message bytes
    uint8_t temperature_decode(uint8_t temp_raw); // decodes raw temperature reading
    bool parse_packet(void); // parses a raw data packet
    Stream* _serial;
    uint8_t data_buffer[PRDC_TMAESC_PACKET_SIZE];  // data buffer
    uint8_t len; // data_buffer counter
    uint8_t poles = 12; // number of BLDC motor poles

    // raw data
    struct data_raw_t {
      uint16_t counter;
      uint16_t throttle_rx;
      uint16_t throttle_out;
      uint16_t rpm;
      uint16_t voltage_bus;
      int16_t current_bus;
      int16_t current_phase;
      uint8_t temperature_mos;
      uint8_t temperature_cap;
      uint16_t status;
    } data_raw;

};
#endif // _PRDC_TMAESC_H_
