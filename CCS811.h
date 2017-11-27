//*****************************************************************************
// Copyright (c) 2017 LucAce
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

#ifndef __CCS811_H__
#define __CCS811_H__

#include "Arduino.h"
#include <Wire.h>

#define CCS811_ADDRESS      (0x5A)
#define CCS811_HW_ID_CODE   (0x81)
#define CCS811_REF_RESISTOR 100000
#define CCS811_ECO2_MAX     8192
#define CCS811_TVOC_MAX     1187

// Sensor Registers/Mailboxes
enum {
    CCS811_STATUS          = 0x00,
    CCS811_MEAS_MODE       = 0x01,
    CCS811_ALG_RESULT_DATA = 0x02,
    CCS811_RAW_DATA        = 0x03,
    CCS811_ENV_DATA        = 0x05,
    CCS811_NTC             = 0x06,
    CCS811_THRESHOLDS      = 0x10,
    CCS811_BASELINE        = 0x11,
    CCS811_HW_ID           = 0x20,
    CCS811_HW_VERSION      = 0x21,
    CCS811_FW_BOOT_VERSION = 0x23,
    CCS811_FW_APP_VERSION  = 0x24,
    CCS811_ERROR_ID        = 0xE0,
    CCS811_SW_RESET        = 0xFF
};

// Bootloader Registers/Mailboxes
enum {
    CCS811_BOOTLOADER_APP_ERASE  = 0xF1,
    CCS811_BOOTLOADER_APP_DATA   = 0xF2,
    CCS811_BOOTLOADER_APP_VERIFY = 0xF3,
    CCS811_BOOTLOADER_APP_START  = 0xF4
};

// Drive Modes
enum {
    CCS811_DRIVE_MODE_IDLE  = 0x00,
    CCS811_DRIVE_MODE_1SEC  = 0x01,
    CCS811_DRIVE_MODE_10SEC = 0x02,
    CCS811_DRIVE_MODE_60SEC = 0x03,
    CCS811_DRIVE_MODE_250MS = 0x04
};

// CCS811 Class Definition
class CCS811 {

    public:
        CCS811(void)  {};
        ~CCS811(void) {};

        // Initialization
        bool     begin(uint8_t addr = CCS811_ADDRESS);
        bool     reset();

        // Direct Register Operations
        void     readStatusRegister();
        void     readMeasModeRegister();
        void     writeMeasModeRegister(
                    uint8_t drive_mode  = CCS811_DRIVE_MODE_1SEC,
                    uint8_t int_datardy = 0,
                    uint8_t int_thresh  = 0
                 );
        void     readAlgResultDataRegister();
        void     readRawDataRegister();
        void     writeEnvDataRegister(
                    uint16_t humdity     = 0x6400,
                    uint16_t temperature = 0x6400
                 );
        void     readNTCRegister();
        void     writeThresholdsRegister(
                    uint16_t low_med    = 0x005D,
                    uint16_t med_high   = 0x09C4,
                    uint8_t  hysteresis = 0x32
                 );
        void     readBaselineRegister();
        void     writeBaselineRegister(uint16_t baseline);
        void     readHWIDRegister();
        void     readHWVersionRegister();
        void     readFWBootVersionRegister();
        void     readFWAppVersionRegister();
        void     readErrorIDRegister();
        void     writeAppEraseRegister(uint8_t buf[4]);
        void     writeAppDataRegister(uint8_t buf[9]);
        void     writeAppVerifyRegister();
        void     writeAppStartRegister();
        void     writeSwResetRegister(uint8_t buf[4]);

        // Configuration
        void     setDriveMode(uint8_t mode);
        void     enableInterrupt();
        void     disableInterrupt();
        void     enableInterruptThreshold();
        void     disableInterruptThreshold();

        // Set environment data
        double   calculateTemperature();
        void     setTempOffset(float offset);
        void     setEnvData(
                    float humidity    = 50.0F,
                    float temperature = 25.0F
                 );

        // Commands
        void     issueSwReset();
        void     startApplication();

        // Get value functions
        uint8_t  isDATA_READY()     { return _status.DATA_READY; }
        uint16_t geteCO2()          { return _eCO2;              }
        uint16_t getTVOC()          { return _TVOC;              }

        uint8_t  hasERROR()         { return _status.ERROR;      }
        uint8_t  getERROR_ID()      { return _error_id.get();    }

        uint8_t  getHWID()          { return _HW_ID;             }
        uint8_t  getHWVersion()     { return _HW_Version;        }
        uint16_t getFWBootVersion() { return _FW_Boot_Version;   }
        uint16_t getFWAppVersion()  { return _FW_App_Version;    }

    private:
        // Sensor attributes
        uint8_t  _i2c_addr;
        float    _temp_offset;
        uint16_t _eCO2;
        uint16_t _TVOC;
        uint16_t _raw_data;
        uint32_t _vref;
        uint32_t _vntc;
        uint16_t _baseline;

        uint8_t  _HW_ID;
        uint8_t  _HW_Version;
        uint16_t _FW_Boot_Version;
        uint16_t _FW_App_Version;

        // I2C operators
        void     _i2c_init();

        uint8_t  read8(byte reg);
        void     write8(byte reg, byte value);
        void     read(uint8_t reg, uint8_t *buf, uint8_t num);
        void     write(uint8_t reg, uint8_t *buf, uint8_t num);


        // STATUS Register (0x00)
        struct status {
            // Bit 0: Error
            // 0: No error has occurred
            // 1: There is an error on the I²C or sensor
            uint8_t ERROR: 1;

            // Bit 2:1: Reserved

            // Bit 3: Data Ready
            // 0: No new data samples are ready
            // 1: A new data sample is ready in ALG_RESULT_DATA
            uint8_t DATA_READY: 1;

            // Bit 4: Application Loaded
            // 0: No application firmware loaded
            // 1: Valid application firmware loaded
            uint8_t APP_VALID: 1;

            // Bit 6:5: Reserved

            // Bit 7: Firmware Mode
            // 0: Firmware is in boot mode
            // 1: Firmware is in application mode
            uint8_t FW_MODE: 1;

            uint8_t get() {
                return (
                    (ERROR      << 0) |
                    (DATA_READY << 3) |
                    (APP_VALID  << 4) |
                    (FW_MODE    << 7)
                );
            }

            void set(uint8_t data) {
                ERROR      = (data >> 0) & 0x01;
                DATA_READY = (data >> 3) & 0x01;
                APP_VALID  = (data >> 4) & 0x01;
                FW_MODE    = (data >> 7) & 0x01;
            }
        };
        status _status;


        // MEAS_MODE (Measurement and Conditions) Register (0x01)
        struct meas_mode {
            // Bit: 1:0: Reserved

            // Bit: 2: Interrupt Threshold
            // 0: Interrupt mode (if enabled) operates normally
            // 1: Interrupt mode (if enabled) only asserts the
            //    nINT signal (driven low) if the new ALG_RESULT_DATA
            //    crosses one of the thresholds.
            uint8_t INT_THRESH: 1;

            // Bit: 3: Interrupt Data Ready
            // 0: Interrupt generation is disabled
            // 1: The nINT signal is asserted (driven low) when a
            //    new sample is ready in ALG_RESULT_DATA.
            uint8_t INT_DATARDY: 1;

            // Bit: 6:4: Drive Mode
            // 000: Mode 0 – Idle (Measurements are disabled in this mode)
            // 001: Mode 1 – Constant power mode, IAQ measurement every second
            // 010: Mode 2 – Pulse heating mode IAQ measurement every 10 seconds
            // 011: Mode 3 – Low power pulse heating mode IAQ measurement every 60 seconds
            // 100: Mode 4 – Constant power mode, sensor measurement every 250ms
            // 1xx: Reserved modes
            uint8_t DRIVE_MODE: 3;

            // Bit: 7: Reserved

            uint8_t get(){
                return (
                    (INT_THRESH  << 2) |
                    (INT_DATARDY << 3) |
                    (DRIVE_MODE  << 4)
                );
            }

            void set(uint8_t data) {
                INT_THRESH  = (data >> 2) & 0x01;
                INT_DATARDY = (data >> 3) & 0x01;
                DRIVE_MODE  = (data >> 4) & 0x07;
            }

        };
        meas_mode _meas_mode;


        // ERROR_ID (Error Identifier) Register (0xE0)
        struct error_id {
            // Bit 0: Write Register Invalid
            // 0: No error
            // 1: The CCS811 received an I2C write request addressed to this
            // station but with invalid register address ID
            uint8_t WRITE_REG_INVALID: 1;

            // Bit 1: Read Register Invalid
            // 0: No error
            // 1: The CCS811 received an I2C read request to a mailbox ID that
            //    is invalid
            uint8_t READ_REG_INVALID: 1;

            // Bit 2: Write of Unsupported Measurement Mode
            // 0: No error
            // 1: The CCS811 received an I2C request to write an unsupported
            //    mode to MEAS_MODE
            uint8_t MEASMODE_INVALID: 1;

            // Bit 3: Resistance Out of Range
            // 0: No error
            // 1: The sensor resistance measurement has reached or exceeded
            //    the maximum range
            uint8_t MAX_RESISTANCE: 1;

            // Bit 4: Heater Out of Range
            // 0: No error
            // 1: The Heater current in the CCS811 is not in range
            uint8_t HEATER_FAULT: 1;

            // Bit 5: Hearter Voltage Error
            // 0: No error
            // 1: The Heater voltage is not being applied correctly
            uint8_t HEATER_SUPPLY: 1;

            // Bit 7:6: Reserved

            uint8_t get(){
                return (
                    (WRITE_REG_INVALID << 0) |
                    (READ_REG_INVALID  << 1) |
                    (MEASMODE_INVALID  << 2) |
                    (MAX_RESISTANCE    << 3) |
                    (HEATER_FAULT      << 4) |
                    (HEATER_SUPPLY     << 5)
                );
            }

            void set(uint8_t data){
                WRITE_REG_INVALID = (data >> 0) & 0x01;
                READ_REG_INVALID  = (data >> 1) & 0x01;
                MEASMODE_INVALID  = (data >> 2) & 0x01;
                MAX_RESISTANCE    = (data >> 3) & 0x01;
                HEATER_FAULT      = (data >> 4) & 0x01;
                HEATER_SUPPLY     = (data >> 5) & 0x01;
            }
        };
        error_id _error_id;
};

#endif
