//*****************************************************************************
// Copyright (c) 2017-2018 LucAce
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
//*****************************************************************************
//
// Arduino Library for AMS CCS811 Ultra-low power Digital VOC Sensor
//
// Library Dependencies:
// - Wire
//
// Notes:
// - ESP8266 Arduino Library v2.3.0 uses a default I2C timeout value that is
//   too short for the CCS811.  A timeout value of 500us is sufficient in most
//   use cases.  Increase the timeout value by calling:
//     Wire.setClockStretchLimit(500);
// - ESP8266 Arduino Library v2.3.0 Wire library resets the I2C timeout every
//   time a call to .begin is made.  setClockStretchLimit(TIMEOUT) must be
//   called following every subsequent call to .begin as a workaround.  Later
//   versions of the ESP8266 for Arduino libraries are expected to correct
//   this.  See:
//     https://github.com/esp8266/Arduino/issues/2162
// - Source code uses Natural Docs for Document Generation.
//
//*****************************************************************************

#include "CCS811.h"

// Group: Public Functions

//*****************************************************************************
// Function: begin
// Initialize the CCS811 Sensor.
//
// Parameters:
// addr - I2C address of the sensor
//
// Returns:
// bool - true if successful; false otherwise
//*****************************************************************************
bool CCS811::begin(uint8_t addr) {
    // Set the I2C Address used by this library
    _i2c_addr = addr;

    // Initialize I2C interface
    this->_i2c_init();

    // Issue reset to sensor
    this->issueSwReset();

    // Verify the Hardware ID field matches the expected value
    // if not do not continue intializing the sensor
    this->readHWIDRegister();
    this->readHWVersionRegister();
    this->readFWBootVersionRegister();
    this->readFWAppVersionRegister();

    if (_HW_ID != CCS811_HW_ID_CODE)
        return false;

    // Issue App Start command to sensor
    this->startApplication();

    // Read the state of the sensor
    this->readStatusRegister();

    // Return if an error is reported
    if (_status.ERROR)
        return false;

    // Return if not in application mode
    if (!_status.FW_MODE)
        return false;

    // Set mode
    // - 1 sample/sec
    // - Disable interrupt
    // - Normal interrupt mode
    this->writeMeasModeRegister(
        CCS811_DRIVE_MODE_1SEC,
        0, 0
    );

    return true;
}


//*****************************************************************************
// Function: reset
// Re-initialize the CCS811 Sensor using the same I2C address already
// configured.
//
// Returns:
// bool - true if successful; false otherwise
//*****************************************************************************
bool CCS811::reset() {
    return this->begin(_i2c_addr);
}


//*****************************************************************************
// Function: readStatusRegister
// Read the Status Register (0x00, 1 byte)
//*****************************************************************************
void CCS811::readStatusRegister() {
    _status.set(read8(CCS811_STATUS));
}


//*****************************************************************************
// Function: readMeasModeRegister
// Read the Measurement and Conditions Register (0x01, 1 byte)
//*****************************************************************************
void CCS811::readMeasModeRegister() {
    _meas_mode.set(read8(CCS811_MEAS_MODE));
}


//*****************************************************************************
// Function: writeMeasModeRegister
// Write the Measurement and Conditions Register (0x01, 1 byte)
//
// Notes:
// - Invalid mode selection will result in a reported error
//
// Parameters:
// drive_mode  - Measurement mode
// int_datardy - Interrupt generation
// int_thresh  - Interrupt threshold mode
//*****************************************************************************
void CCS811::writeMeasModeRegister(
    uint8_t drive_mode,
    uint8_t int_datardy,
    uint8_t int_thresh
) {
    _meas_mode.DRIVE_MODE  = drive_mode  & 0x07;
    _meas_mode.INT_DATARDY = int_datardy & 0x01;
    _meas_mode.INT_THRESH  = int_thresh  & 0x01;
    this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}


//*****************************************************************************
// Function: readAlgResultDataRegister
// Read the Algorithm Results Data Register (0x02, 8 bytes)
//
// Notes:
// - All 8 bytes of this register, including the raw data, are read
//*****************************************************************************
void CCS811::readAlgResultDataRegister() {
    uint8_t buf[8];

    this->read(CCS811_ALG_RESULT_DATA, buf, 8);

    _eCO2 = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
    _TVOC = ((uint16_t)buf[2] << 8) | ((uint16_t)buf[3]);

    _status.set(buf[4]);
    _error_id.set(buf[5]);
    _raw_data = ((uint16_t)buf[6] << 8) | ((uint16_t)buf[7]);
}


//*****************************************************************************
// Function: readRawDataRegister
// Read the Algorithm Results Data Register (0x03, 2 bytes)
//*****************************************************************************
void CCS811::readRawDataRegister() {
    uint8_t buf[2];

    this->read(CCS811_RAW_DATA, buf, 2);
    _raw_data = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
}


//*****************************************************************************
// Function: writeEnvDataRegister
// Write the Environment Data Register (0x05, 2 bytes)
//
// Notes:
// - Humidity is stored as an unsigned 16 bits in 1/512%RH. The default
//   value is 50% = 0x64, 0x00. As an example 48.5% humidity would be
//   0x61, 0x00.
// - Temperature is stored as an unsigned 16 bits integer in 1/512 degrees;
//   there is an offset: 0 maps to -25C. The default value is
//   25C = 0x64, 0x00. As an example 23.5% temperature would be 0x61, 0x00.
//   The internal algorithm uses these values (or default values if not set
//   by the application) to compensate for changes in relative humidity and
//   ambient temperature.
//
// Parameters:
// humidity    - Sensor representation of the humidity value
// temperature - Sensor representation of the temperature value
//*****************************************************************************
void CCS811::writeEnvDataRegister(
    uint16_t humdity,
    uint16_t temperature
) {
    uint8_t buf[] = {
        (uint8_t)((humdity     >> 8) & 0xFF),
        (uint8_t)((humdity     >> 0) & 0xFF),
        (uint8_t)((temperature >> 8) & 0xFF),
        (uint8_t)((temperature >> 0) & 0xFF)
    };

    this->write(CCS811_ENV_DATA, buf, 4);
}


//*****************************************************************************
// Function: readNTCRegister
// Read the NTC Register (0x06, 4 bytes)
//*****************************************************************************
void CCS811::readNTCRegister() {
    uint8_t buf[4];

    this->read(CCS811_NTC, buf, 4);
    _vref = ((uint32_t)buf[0] << 8) | (uint32_t)buf[1];
    _vntc = ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}


//*****************************************************************************
// Function: writeThresholdsRegister
// Write the Thresholds Register (0x10, 5 bytes)
//
// Notes:
// - An interrupt is asserted (if enabled) if the eCO2 value moved from the
//   current range (Low, Medium, or High) into another range by more than the
//   Hysteresis value (used to prevent multiple interrupts close to a
//   threshold).
//
// Parameters:
// low_med    - Low to Medium Threshold (default = 1500ppm = 0x05D)
// med_high   - Medium to High Threshold (default = 2500ppm = 0x09C4)
// hysteresis - Hysteresis value (default = 50 = 0x32)
//*****************************************************************************
void CCS811::writeThresholdsRegister(
    uint16_t low_med,
    uint16_t med_high,
    uint8_t  hysteresis
) {
    uint8_t buf[] = {
        (uint8_t)((low_med  >> 8) & 0xFF),
        (uint8_t)((low_med  >> 0) & 0xFF),
        (uint8_t)((med_high >> 8) & 0xFF),
        (uint8_t)((med_high >> 0) & 0xFF),
        hysteresis
    };

    this->write(CCS811_THRESHOLDS, buf, 5);
}


//*****************************************************************************
// Function: readBaselineRegister
// Read the Baseline Register (0x11, 2 bytes)
//
// Notes:
// - The format of the baseline value is not specified.
//*****************************************************************************
void CCS811::readBaselineRegister() {
    uint8_t buf[2];

    this->read(CCS811_BASELINE, buf, 2);
    _baseline = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
}


//*****************************************************************************
// Function: writeBaselineRegister
// Write the Baseline Register (0x11, 2 bytes)
//
// Notes:
// - The format of the baseline value is not specified.
//
// Parameters:
// baseline - Sensor baseline value
//*****************************************************************************
void CCS811::writeBaselineRegister(uint16_t baseline) {
    uint8_t buf[] = {
        (uint8_t)((baseline >> 8) & 0xFF),
        (uint8_t)((baseline >> 0) & 0xFF)
    };

    _baseline = baseline;
    this->write(CCS811_BASELINE, buf, 2);
}


//*****************************************************************************
// Function: readHWIDRegister
// Read the Hardware ID Register (0x20, 1 byte)
//*****************************************************************************
void CCS811::readHWIDRegister() {
    _HW_ID = this->read8(CCS811_HW_ID);
}


//*****************************************************************************
// Function: readHWVersionRegister
// Read the Hardware Version Register (0x21, 1 byte)
//*****************************************************************************
void CCS811::readHWVersionRegister() {
    _HW_Version = this->read8(CCS811_HW_VERSION);
}


//*****************************************************************************
// Function: readFWBootVersionRegister
// Read the Firmware Bootloader Version Register (0x23, 2 bytes)
//*****************************************************************************
void CCS811::readFWBootVersionRegister() {
    uint8_t buf[2];

    this->read(CCS811_FW_BOOT_VERSION, buf, 2);
    _FW_Boot_Version = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
}


//*****************************************************************************
// Function: readFWAppVersionRegister
// Read the Firmware Application Version Register (0x24, 2 bytes)
//*****************************************************************************
void CCS811::readFWAppVersionRegister() {
    uint8_t buf[2];

    this->read(CCS811_FW_APP_VERSION, buf, 2);
    _FW_App_Version = ((uint16_t)buf[0] << 8) | ((uint16_t)buf[1]);
}


//*****************************************************************************
// Function: readErrorIDRegister
// Read the Error ID Register (0xE0, 2 bytes)
//*****************************************************************************
void CCS811::readErrorIDRegister() {
    _error_id.set(read8(CCS811_ERROR_ID));
}


//*****************************************************************************
// Function: writeAppEraseRegister
// Write the Application Erase Register (0xF1, 4 bytes)
//
// Notes:
// - Application is erased only when the proper sequence is provided
//   0xE7, 0xA7, 0xE6, 0x09
// - To protect the application code from accidental erases the sequence must
//   be provided with the function call
//
// Parameters:
// buf - Erase sequence
//*****************************************************************************
void CCS811::writeAppEraseRegister(uint8_t buf[4]) {
    this->write(CCS811_BOOTLOADER_APP_ERASE, buf, 4);
}


//*****************************************************************************
// Function: writeAppDataRegister
// Write the Application Data Register (0xF2, 9 bytes)
//
// Parameters:
// buf - 9 bytes of application data
//*****************************************************************************
void CCS811::writeAppDataRegister(uint8_t buf[9]) {
    this->write(CCS811_BOOTLOADER_APP_DATA, buf, 9);
}


//*****************************************************************************
// Function: writeAppVerifyRegister
// Write the Application Verify Register (0xF3, 0 bytes)
//
// Notes:
// - No data is sent with this write
//*****************************************************************************
void CCS811::writeAppVerifyRegister() {
    this->write(CCS811_BOOTLOADER_APP_VERIFY, NULL, 0);
}


//*****************************************************************************
// Function: writeAppStartRegister
// Write the Application Start Register (0xF4, 0 bytes)
//
// Notes:
// - No data is sent with this write
//*****************************************************************************
void CCS811::writeAppStartRegister() {
    this->write(CCS811_BOOTLOADER_APP_START, NULL, 0);
}


//*****************************************************************************
// Function: writeSwResetRegister()
// Write the Software Reset Register (0xFF, 4 bytes)
//
// Notes:
// - Application is reset only when the proper sequence is provided
//   0x11, 0xE5, 0x72, 0x8A
// - To protect the application code from accidental resets the sequence must
//   be provided with the function call
//
// Parameters:
// buf - 4 bytes reset sequence
//*****************************************************************************
void CCS811::writeSwResetRegister(uint8_t buf[4]) {
    this->write(CCS811_SW_RESET, buf, 4);
}


//*****************************************************************************
// Function: setDriveMode
// Set the sensor sample/drive mode.
//
// Notes:
// Invalid mode selection will result in a reported error
//
// Parameters:
// drive_mode - Drive mode to configure
//*****************************************************************************
void CCS811::setDriveMode(uint8_t drive_mode) {
    _meas_mode.DRIVE_MODE = drive_mode;
    this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}


//*****************************************************************************
// Function: enableInterrupt
// Enable the interrupt output signal.
//*****************************************************************************
void CCS811::enableInterrupt() {
    _meas_mode.INT_DATARDY = 1;
    this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}


//*****************************************************************************
// Function: disableInterrupt
// Disable the interrupt output signal.
//*****************************************************************************
void CCS811::disableInterrupt() {
    _meas_mode.INT_DATARDY = 0;
    this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}


//*****************************************************************************
// Function: enableInterruptThreshold
// Enable the interrupt threshold mode.
//*****************************************************************************
void CCS811::enableInterruptThreshold() {
    _meas_mode.INT_THRESH = 1;
    this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}


//*****************************************************************************
// Function: disableInterruptThreshold
// Disable the interrupt threshold mode.
//*****************************************************************************
void CCS811::disableInterruptThreshold() {
    _meas_mode.INT_THRESH = 0;
    this->write8(CCS811_MEAS_MODE, _meas_mode.get());
}


//*****************************************************************************
// Function: calculateTemperature
// Calculate the Temperature using the onboard NTC register.
//
// Notes:
// - Calculation based on vendor App note
//
// Returns:
// double - Calculated temperature value
//*****************************************************************************
double CCS811::calculateTemperature() {
    double   ntc_temp;
    uint32_t rntc;

    this->readNTCRegister();

    rntc     =  _vntc * CCS811_REF_RESISTOR / _vref;
    ntc_temp =  log((double)rntc / CCS811_REF_RESISTOR);
    ntc_temp /= 3380;
    ntc_temp += 1.0 / (25 + 273.15);
    ntc_temp =  1.0 / ntc_temp;
    ntc_temp -= 273.15;

    return ntc_temp - _temp_offset;
}


//*****************************************************************************
// Function: setTempOffset
// Set the temperature office attribute.
//
// Parameters:
// offset - Temperature offset to set
//*****************************************************************************
void CCS811::setTempOffset(float offset) {
    _temp_offset = offset;
}


//*****************************************************************************
// Function: setEnvData
// Set the Environment Data
//
// Parameters:
// humidity    - Relative Humidity
// temperature - Temperature
//*****************************************************************************
void CCS811::setEnvData(float humidity, float temperature) {
    uint16_t humd_buf;
    uint16_t temp_buf;
    uint8_t  envData[4];

    // Check for invalid temperature
    if ((temperature < -25.0F) || (temperature > 50.0F))
        return;

    // Check for invalid humidity
    if ((humidity < 0.0F) || (humidity > 100.0F))
        return;

    uint32_t rH   = humidity    * 1000;
    uint32_t temp = temperature * 1000;

    // Split value into 7-bit integer and 9-bit fractional
    envData[0] = ((rH % 1000) / 100) > 7 ? (rH / 1000 + 1) << 1 : (rH / 1000) << 1;

    // CCS811 only supports increments of 0.5 so bits 7-0 will always be zero
    envData[1] = 0;
    if (((rH % 1000) / 100) > 2 && (((rH % 1000) / 100) < 8)) {
        // Set 9th bit of fractional to indicate 0.5%
        envData[0] |= 1;
    }
    temp += 25000; // Add the 25C offset

    // Split value into 7-bit integer and 9-bit fractional
    envData[2] = ((temp % 1000) / 100) > 7 ? (temp / 1000 + 1) << 1 : (temp / 1000) << 1;
    envData[3] = 0;
    if (((temp % 1000) / 100) > 2 && (((temp % 1000) / 100) < 8)) {
        // Set 9th bit of fractional to indicate 0.5C
        envData[2] |= 1;
    }

    humd_buf = ((uint16_t)envData[0] << 8) | ((uint16_t)envData[1] << 0);
    temp_buf = ((uint16_t)envData[2] << 8) | ((uint16_t)envData[3] << 0);

    this->writeEnvDataRegister(humd_buf, temp_buf);
}


//*****************************************************************************
// Function: issueSwReset()
// Issue software reset.
//*****************************************************************************
void CCS811::issueSwReset() {
    uint8_t buf[4] = {0x11, 0xE5, 0x72, 0x8A};
    this->writeSwResetRegister(buf);
    delay(100);
}


//*****************************************************************************
// Function: startApplication()
// Issue application start.
//*****************************************************************************
void CCS811::startApplication() {
    this->writeAppStartRegister();
    delay(100);
}


// Group: Private Functions

//*****************************************************************************
// Function: _i2c_init
// Initialize the I2C interface.
//*****************************************************************************
void CCS811::_i2c_init() {
    Wire.begin();
}


//*****************************************************************************
// Function: read8
// Read one byte of data from the I2C interface.
//
// Parameters:
// reg - Register/Mailbox to read
//
// Returns:
// uint8_t - Value read
//*****************************************************************************
uint8_t CCS811::read8(byte reg) {
    uint8_t ret;
    this->read(reg, &ret, 1);
    return ret;
}


//*****************************************************************************
// Function: write8
// Write one byte of data over the I2C interface.
//
// Parameters:
// reg   - Register/Mailbox to write
// value - Value to write
//*****************************************************************************
void CCS811::write8(byte reg, byte value) {
    this->write(reg, &value, 1);
}


//*****************************************************************************
// Function: read
// Read multi-byte data from the I2C interface.
//
// Parameters:
// reg  - Register/Mailbox to read
// *buf - Pointer to data read on interface
// num  - Number of bytes to read
//*****************************************************************************
void CCS811::read(uint8_t reg, uint8_t *buf, uint8_t num) {
    uint8_t value;
    uint8_t pos = 0;

    // On arduino we need to read in 32 byte chunks
    while (pos < num){
        uint8_t read_now = min(32, num - pos);
        Wire.beginTransmission((uint8_t)_i2c_addr);
        Wire.write((uint8_t)reg + pos);
        Wire.endTransmission();
        Wire.requestFrom((uint8_t)_i2c_addr, read_now);

        for (int i=0; i<read_now; i++){
            buf[pos] = Wire.read();
            pos++;
        }
    }
}


//*****************************************************************************
// Function: write
// Write multi-byte data to the I2C interface.
//
// Parameters:
// reg  - Register/Mailbox to write
// *buf - Pointer to data to write on interface
// num  - Number of bytes to write
//*****************************************************************************
void CCS811::write(uint8_t reg, uint8_t *buf, uint8_t num) {
    Wire.beginTransmission((uint8_t)_i2c_addr);
    Wire.write((uint8_t)reg);
    Wire.write((uint8_t *)buf, num);
    Wire.endTransmission();
}
