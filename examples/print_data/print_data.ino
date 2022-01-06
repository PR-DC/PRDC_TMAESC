/**
 * print_data.ino - T-Motor ALPHA ESC Telemetry Protocol Arduino library example
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
 
// Library
// --------------------
// PRDC PRDC_TMAESC
// Author: PR-DC
#include <PRDC_TMAESC.h>

#define DEBUG_BAUD  1000000
#define LED_PIN     PB12

HardwareSerial esc_serial(USART2);
PRDC_TMAESC esc;

uint32_t last_time;

uint32_t dt_LED = 1000; // [ms] time difference for LED state change
uint32_t t_LED; // [ms] LED time

// setup function
// --------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  esc.set_poles(12);
  esc.init(esc_serial);
  Serial.begin(DEBUG_BAUD);
}

// loop function
// --------------------
void loop() {
  if (esc.update()) {
    print_data();
    last_time = esc.data.time;
  }
  blinkLED();
}

void print_data() {
  Serial.print("Time: ");
  Serial.print(esc.data.time);
  Serial.println(" ms");
  if (esc.data.fault) {
    Serial.print("Fault(s) detected! Status code: ");
    Serial.println(esc.data.status, BIN);
  }
  else {
    Serial.println("No faults detected.");
  }
  Serial.print("Packet number: ");
  Serial.println(esc.data.counter);
  Serial.print("Input throttle: ");
  Serial.print(esc.data.throttle_rx);
  Serial.println("%");
  Serial.print("Actual throttle: ");
  Serial.print(esc.data.throttle_out);
  Serial.println("%");
  Serial.print("RPM: ");
  Serial.println(esc.data.rpm);
  Serial.print("Bus voltage: ");
  Serial.print(esc.data.voltage_bus);
  Serial.println(" V");
  Serial.print("Bus current: ");
  Serial.print(esc.data.current_bus);
  Serial.println(" A");
  Serial.print("Phase current: ");
  Serial.print(esc.data.current_phase);
  Serial.println(" A");
  Serial.print("MOSFET temperature: ");
  Serial.print(esc.data.temperature_mos);
  Serial.println(" degC");
  Serial.print("Capacitor temperature: ");
  Serial.print(esc.data.temperature_cap);
  Serial.println(" degC");
  Serial.print("Packet frequency: ");
  Serial.print(1000.0 / (esc.data.time - last_time));
  Serial.println(" Hz");
  Serial.println();
}

// blinkLED() function
// Blink LED at right time
// --------------------
void blinkLED() {
  static bool s = 0;
  if((millis()-t_LED) > dt_LED) {
    t_LED = millis();
    s = !s;
    digitalWrite(LED_BUILTIN, s);
  }
}