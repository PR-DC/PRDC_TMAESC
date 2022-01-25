/**
 * PRDC_TMAESC.cpp - T-Motor ALPHA ESC Telemetry Protocol Arduino library
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

#include "PRDC_TMAESC.h"

// init()
// initalizes communication
// --------------------
void PRDC_TMAESC::init(Stream& stream) {
	_serial = &stream;
}

void PRDC_TMAESC::init(HardwareSerial& serial){
  serial.begin(PRDC_TMAESC_BAUD);
  init((Stream&)serial);
}

// update()
// reads and parses a packet if available
// --------------------
bool PRDC_TMAESC::update() {
	uint32_t n = _serial->available();
	if(n == 0) {
  	return false;
  }
	while(n--) {
		uint8_t b = _serial->read();
		if(len == 0 && b != PRDC_TMAESC_B1) {
			continue;
		}
		if(len == 1 && b != PRDC_TMAESC_B2) {
			continue;
		}
		data_buffer[len++] = b;
		if(len == PRDC_TMAESC_PACKET_SIZE) {
			len = 0;
			if(parse_packet()){
				return true;
			}
		}
	}
	return false;
}

// parse_packet()
// parses a raw data packet
// --------------------
bool PRDC_TMAESC::parse_packet(void) {
	// check packet integrity
	uint16_t checksum_received = (data_buffer[23] << 8) | data_buffer[22];
	uint16_t checksum_calculated = calc_checksum(data_buffer, PRDC_TMAESC_PACKET_SIZE-2);
	if(checksum_received == checksum_calculated) {
		data.time = millis();
		// pack raw data
		data_raw.counter = (data_buffer[4] << 8) | data_buffer[5];
		data_raw.throttle_rx = (data_buffer[6] << 8) | data_buffer[7];
		data_raw.throttle_out = (data_buffer[8] << 8) | data_buffer[9];
		data_raw.rpm = (data_buffer[10] << 8) | data_buffer[11];
		data_raw.voltage_bus = (data_buffer[12] << 8) | data_buffer[13];
		data_raw.current_bus = (data_buffer[14] << 8) | data_buffer[15];
		data_raw.current_phase = (data_buffer[16] << 8) | data_buffer[17];
		data_raw.temperature_mos = data_buffer[18];
		data_raw.temperature_cap = data_buffer[19];
		data_raw.status = (data_buffer[20] << 8) | data_buffer[21];
		// process raw data
		data.counter = data_raw.counter;
		data.throttle_rx = (float)data_raw.throttle_rx*100.0/1024.0;
		data.throttle_out = (float)data_raw.throttle_out*100.0/1024.0;
		data.rpm = data_raw.rpm*10.0/(3.0*poles);
		data.voltage_bus = (float)data_raw.voltage_bus/10.0;
		data.current_bus = (float)data_raw.current_bus/64.0;
		data.current_phase = (float)data_raw.current_phase/64.0;
		data.temperature_mos = temperature_decode(data_raw.temperature_mos);
		data.temperature_cap = temperature_decode(data_raw.temperature_cap);
		data.status = data_raw.status;
		data.fault = (data.status != 0x00);
		return true;
	}
	else {
		return false;
	}
}

// set_poles()
// sets BLDC motor pole number
// --------------------
void PRDC_TMAESC::set_poles(uint8_t p) {
  poles = p;
}

// calc_checksum()
// calculates checksum by summing message bytes
// --------------------
uint16_t PRDC_TMAESC::calc_checksum(uint8_t* data, uint16_t n) {
  uint16_t checksum = 0;
  while(n--) {
    checksum += data[n];
  }
	return checksum;
}

// temperature decoding table
static const struct {
	uint8_t temp_reading;
	uint8_t temp_value;
} temp_table[] = {
	{241, 	0}, 	{240, 	1}, 	{239, 	2}, 	{238, 	3}, 	{237, 	4}, 	{236, 	5}, 	{235, 	6}, 	{234, 	7}, 	{233, 	8}, 	{232, 	9},
	{231, 	10}, 	{230, 	11}, 	{229, 	12}, 	{228, 	13}, 	{227, 	14}, 	{226, 	15}, 	{224, 	16}, 	{223, 	17}, 	{222, 	18}, 	{220, 	19},
	{219, 	20}, 	{217, 	21}, 	{216, 	22}, 	{214, 	23}, 	{213, 	24}, 	{211, 	25}, 	{209, 	26}, 	{208, 	27}, 	{206, 	28}, 	{204, 	29},
	{202, 	30}, 	{201, 	31}, 	{199, 	32}, 	{197, 	33}, 	{195, 	34}, 	{193, 	35}, 	{191, 	36}, 	{189, 	37}, 	{187, 	38}, 	{185, 	39},
	{183, 	40}, 	{181, 	41}, 	{179, 	42}, 	{177, 	43}, 	{174, 	44}, 	{172, 	45}, 	{170, 	46}, 	{168, 	47}, 	{166, 	48}, 	{164, 	49},
	{161, 	50}, 	{159, 	51}, 	{157, 	52}, 	{154, 	53}, 	{152, 	54}, 	{150, 	55}, 	{148, 	56}, 	{146, 	57}, 	{143, 	58}, 	{141, 	59},
	{139, 	60}, 	{136, 	61}, 	{134, 	62}, 	{132, 	63}, 	{130, 	64}, 	{128, 	65}, 	{125, 	66}, 	{123, 	67}, 	{121, 	68}, 	{119, 	69},
	{117, 	70}, 	{115, 	71}, 	{113, 	72}, 	{111, 	73}, 	{109, 	74}, 	{106, 	75}, 	{105, 	76}, 	{103, 	77}, 	{101, 	78}, 	{99, 	79},
	{97, 	80}, 	{95, 	81}, 	{93, 	82}, 	{91, 	83}, 	{90, 	84}, 	{88, 	85}, 	{85, 	86}, 	{84, 	87}, 	{82, 	88}, 	{81, 	89},
	{79, 	90}, 	{77, 	91}, 	{76, 	92}, 	{74, 	93}, 	{73, 	94}, 	{72, 	95}, 	{69, 	96}, 	{68, 	97}, 	{66, 	98}, 	{65, 	99},
	{64, 	100}, 	{62, 	101}, 	{62, 	102}, 	{61, 	103}, 	{59, 	104}, 	{58, 	105}, 	{56, 	106}, 	{54, 	107}, 	{54, 	108}, 	{53, 	109},
	{51, 	110}, 	{51, 	111}, 	{50, 	112}, 	{48, 	113}, 	{48, 	114}, 	{46, 	115}, 	{46, 	116}, 	{44, 	117}, 	{43, 	118}, 	{43, 	119},
	{41, 	120}, 	{41, 	121}, 	{39, 	122}, 	{39, 	123}, 	{39, 	124}, 	{37, 	125}, 	{37, 	126}, 	{35, 	127}, 	{35, 	128}, 	{33, 	129},
};

// temperature_decode()
// decodes raw temperature reading
// --------------------
uint8_t PRDC_TMAESC::temperature_decode(uint8_t temp_raw) {
  for (uint8_t i=0; i<130; i++) {
    if (temp_table[i].temp_reading <= temp_raw) {
      return temp_table[i].temp_value;
    }
  }
  return 130U;
}
