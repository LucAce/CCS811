//*****************************************************************************
// Copyright (c) 2018 LucAce
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
// CCS811 Library Sensor Example
//
// Board Compatibility:
// - ESP8266 NodeMCU LUA CP2102 ESP-12E Internet WIFI Development Board
//     https://www.amazon.com/dp/B010O1G1ES/
// - Adafruit CCS811 Air Quality Sensor Breakout
//     https://www.adafruit.com/product/3566
//
// Arduino ESP8266 Boards Package:
// - http://arduino.esp8266.com/stable/package_esp8266com_index.json
//
// Arduino Library Dependencies:
// - Wire
//
// Notes:
// - ESP8266 Arduino Library uses a default I2C timout value that is
//   too short for the CCS811.  A timeout value of 500us is sufficient in most
//   use cases.  Increase the timeout value by calling:
//     Wire.setClockStretchLimit(500);
// - CCS811 library defaults to a 1 sec sample rate and the time between
//   reading the sensor values must exceed that value plus some margin of
//   error.  If the delay is not sufficient the sensor may continuously
//   report data as not available.
//*****************************************************************************

#include <Wire.h>
#include "CCS811.h"

// Startup delay (20 Minutes)
#define CCS811STARTUPDELAY 1200000

// Sensor sample delay (5 Seconds)
#define CCS811SAMPLEDELAY 5000

// CCS811 I2C Interface
CCS811 ccs;

// CCS811 Startup Time
volatile bool          ccs811Enabled = 0;
volatile unsigned long ccs811StartupTime;


//*****************************************************************************
// Function: setup
// Environment setup.
//*****************************************************************************
void setup() {
    // Set Serial BAUD Rate
    Serial.begin(115200);
    delay(100);

    // Wait for serial monitor to open
    while(!Serial);

    Serial.println(F("\nCCS811 Library Example\n"));

    // Configure the CCS sensor to use I2C interface
    if (!ccs.begin()) {
        Serial.println(F("ERROR: Could not find a valid CCS811 sensor"));
        Serial.println(F("WARN:  Forcing Watch Dog Timeout (WDT)"));
        while(1);
    }

    // Let sensor boot up
    delay(1000);

    // Extend I2C clock stretch timeout (See Notes)
    Wire.setClockStretchLimit(500);

    // Calibrate CS811 sensor
    do {
        delay(CCS811SAMPLEDELAY);
        ccs.readStatusRegister();
    }
    while (!ccs.isDATA_READY());
    float temp = ccs.calculateTemperature();
    ccs.setTempOffset(temp - 25.0);

    // Set the sensore startup delay time
    ccs811StartupTime = millis() + CCS811STARTUPDELAY;

    // Print CCS811 sensor information
    Serial.println(F("CCS811 Sensor Enabled:"));
    Serial.print(F("Hardware ID:           0x"));
    Serial.println(ccs.getHWID(), HEX);
    Serial.print(F("Hardware Version:      0x"));
    Serial.println(ccs.getHWVersion(), HEX);
    Serial.print(F("Firmware Boot Version: 0x"));
    Serial.println(ccs.getFWBootVersion(), HEX);
    Serial.print(F("Firmware App Version:  0x"));
    Serial.println(ccs.getFWAppVersion(), HEX);
    Serial.println();

    // Print startup delay message
    Serial.print(F("CCS811 Startup In "));
    Serial.print(CCS811STARTUPDELAY/1000);
    Serial.println(F(" sec\n"));
}


//*****************************************************************************
// Function: loop
// Main Processing Loop.
//*****************************************************************************
void loop() {
  // If the sensor has not yet been enabled and the startup delay time has
  // not transpired return.  This implements the startup delay.
  if ((!ccs811Enabled) && (millis() < ccs811StartupTime)) {
    return;
  }

  // Startup time has transpired, set the enable so the delay code is
  // no longer used
  ccs811Enabled = 1;

  // Sample CCS811 Data
  sampleCCS811();

  // Wait for next sample
  delay(CCS811SAMPLEDELAY);
}


//*****************************************************************************
// Function: sampleCCS811
// Sample and update CCS811 data from sensor.
//*****************************************************************************
void sampleCCS811() {
  uint16_t    ccs_eco2;   // CCS811 eCO2
  uint16_t    ccs_tvoc;   // CCS811 TVOC
  uint8_t     ccs_error;  // CCS811 error register

  Serial.println(F("Reading CCS811 Sensor"));

  // Read the sensor data, this updates multiple fields
  // in the CCS811 library
  ccs.readAlgResultDataRegister();

  // Read error register if an error was reported
  if (ccs.hasERROR()) {
    Serial.println(F("ERROR: CCS811 Error Flag Set"));

    ccs_error = ccs.getERROR_ID();
    Serial.print(F("CCS811 Error Register = "));
    Serial.println(ccs_error);
    Serial.println();
    return;
  }

  // Data is ready
  if (ccs.isDATA_READY()) {
    ccs_eco2 = ccs.geteCO2();
    ccs_tvoc = ccs.getTVOC();

    // Verify eCO2 is valid
    if (ccs_eco2 > CCS811_ECO2_MAX) {
      Serial.println(F("ERROR: CCS811 eCO2 Exceeded Limit"));
    }

    // Print eCO2 to serial monitor
    Serial.print(F("eCO2 = "));
    Serial.print(ccs_eco2);
    Serial.println(F(" ppm"));

    // Verify TVOC is valid
    if (ccs_tvoc > CCS811_TVOC_MAX) {
      Serial.println(F("ERROR: CCS811 TVOC Exceeded Limit"));
    }

    // Print TVOC to serial monitor
    Serial.print(F("TVOC = "));
    Serial.print(ccs_tvoc);
    Serial.println(F(" ppb"));
  }
  else {
    Serial.println(F("ERROR: CCS811 Data Not Ready"));
  }

  Serial.println();
}
