/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include "AP_EFI.h"
#include "AP_EFI_Backend.h"

// Engine Status Bitmask
#define ENG_STATUS_READY             (1<<0)
#define ENG_STATUS_CRANKING          (1<<1)
#define ENG_STATUS_WARMUP_ENRICHMENT (1<<2)
#define ENG_STATUS_WARMUP            (1<<3)
#define ENG_STATUS_TPS_ACCEL         (1<<4)
#define ENG_STATUS_DECCEL_MODE       (1<<5)
#define ENG_STATUS_MAP_ACCEL         (1<<6)
#define ENG_STATUS_IDLE_ON           (1<<7)

// Fuel Injector Status Bitmask
#define SQUIRT_FIRING_1              (1<<0)
#define SQUIRT_FIRING_2              (1<<1)
#define SQUIRT_SCHED_1               (1<<2)
#define SQUIRT_NOW_1                 (1<<3)
#define SQUIRT_SCHED_2               (1<<4)
#define SQUIRT_NOW_2                 (1<<5)



class AP_EFI_Serial_MS: public AP_EFI_Backend {
    
public:
    // Constructor with initialization
    AP_EFI_Serial_MS(EFI_State& _efi_state);

    // Update the state structure
    void update();



private:
    AP_HAL::UARTDriver *_port = nullptr;
    void parse_realtime_data();
    bool read_incoming_realtime_data();
    void send_request(uint8_t table, uint16_t first_offset, uint16_t last_offset);
    uint8_t read_byte_CRC32();
    uint32_t CRC32_compute_byte(uint32_t inCrc32, uint8_t data);
    float f_to_k(float temp_f) { return (temp_f + 459.67f) * 0.55556f; };
    
    // Serial Protocol Variables
    uint32_t _checksum;
    uint8_t _step;
    uint8_t _response_flag;
    uint16_t _message_counter;
    
    
    // confirmed that last command was ok
    bool _last_command_confirmed : 1;
    

    
    // Command Response Codes
    enum response_codes {
        RESPONSE_WRITE_OK =0x00,
        RESPONSE_REALTIME_DATA,
        RESPONSE_PAGE_DATA,
        RESPONSE_CONFIG_ERROR,
        RESPONSE_PAGE10_OK,
        RESPONSE_CAN_DATA,
        // Error Responses
        ERR_UNDERRUN = 0X80,
        ERR_OVERRUN,
        ERR_CRC_FAILURE,
        ERR_UNRECOGNIZED_COMMAND,
        ERR_OUT_OF_RANGE,
        ERR_SERIAL_BUSY,
        ERR_FLASH_LOCKED,
        ERR_SEQ_FAIL_1,
        ERR_SEQ_FAIL_2,
        ERR_CAN_QUEUE_FULL,
        ERR_CAN_TIMEOUT,
        ERR_CAN_FAILURE,
        ERR_PARITY,
        ERR_FRAMING,
        ERR_SERIAL_NOISE,
        ERR_TXMODE_RANGE,
        ERR_UNKNOWN
    };
    
    // Realtime Data Table Locations
    enum realtime_data {
        PW1_MSB = 2,
        PW1_LSB,
        RPM_MSB = 6,
        RPM_LSB,
        ADVANCE_MSB,
        ADVANCE_LSB,
        ENGINE_BM = 11,
        BAROMETER_MSB = 16,
        BAROMETER_LSB,
        MAP_MSB,
        MAP_LSB,
        MAT_MSB,
        MAT_LSB,
        CHT_MSB,
        CHT_LSB,
        TPS_MSB,
        TPS_LSB,
        AFR1_MSB = 28,
        AFR1_LSB,
        AFR2_MSB,
        AFR2_LSB,
        DWELL_MSB = 62,
        DWELL_LSB,
        LOAD = 66,
        FUEL_PRESSURE_MSB = 128,
        FUEL_PRESSURE_LSB, 
        // Helpers used when sending request
        RT_FIRST_OFFSET = PW1_MSB,
        RT_LAST_OFFSET = FUEL_PRESSURE_LSB
    };


    
};
