/**
 * @file    RM3100.cpp
 * @brief   Magnetometer data acquisition module
 * @author	Dr. Klaus Schaefer
 * @copyright 	Copyright 2026 Dr. Klaus Schaefer. All rights reserved.
 * @license 	This project is released under the GNU Public License GPL-3.0

    <Larus Flight Sensor Firmware>

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

 **************************************************************************/

#include "FreeRTOS_wrapper.h"
#include "RM3100.hpp"
#include "stm32l4xx_hal.h"

#define MEASURE_DAQ_TIME		0

#define RM3100_POLL_REG        0x00

#define RM3100_CMM_REG         0x01

#define RM3100_CCX1_REG        0x04
#define RM3100_CCX0_REG        0x05
#define RM3100_CCY1_REG        0x06
#define RM3100_CCY0_REG        0x07
#define RM3100_CCZ1_REG        0x08
#define RM3100_CCZ0_REG        0x09

#define RM3100_TMRC_REG        0x0B

#define RM3100_MX2_REG      0x24
#define RM3100_MX1_REG      0x25
#define RM3100_MX0_REG      0x26
#define RM3100_MY2_REG      0x27
#define RM3100_MY1_REG      0x28
#define RM3100_MY0_REG      0x29
#define RM3100_MZ2_REG      0x2A
#define RM3100_MZ1_REG      0x2B
#define RM3100_MZ0_REG      0x2C

#define RM3100_BIST_REG       0x33
#define RM3100_STATUS_REG     0x34
#define RM3100_HSHAKE_REG     0x35
#define RM3100_REVID_REG      0x36

#if 1
#define CCP0    0x90      // Cycle Count values
#define CCP1    0x01
#else
#define CCP0    0xC8      // Cycle Count values
#define CCP1    0x00
#endif

#define CCP0_DEFAULT 0xC8 // Default Cycle Count values (used as a whoami check)
#define CCP1_DEFAULT 0x00

#define GAIN_CC50 20.0f   // LSB/uT
#define GAIN_CC100 38.0f
#define GAIN_CC200 75.0f

//#define TMRC    0x92    // Update rate 600Hz
//#define TMRC    0x93    // Update rate 300Hz
#define TMRC    0x94    // Update rate 150Hz
//#define TMRC    0x95    // Update rate 75Hz
#define CMM     0x71    // read 3 axes and set data ready if 3 axes are ready

extern SPI_HandleTypeDef hspi1;
extern CAN_HandleTypeDef hcan1;

class hw_data
{
public:
	uint8_t dummy; // first byte invalid
	uint8_t magx_2;
	uint8_t magx_1;
	uint8_t magx_0;
	uint8_t magy_2;
	uint8_t magy_1;
	uint8_t magy_0;
	uint8_t magz_2;
	uint8_t magz_1;
	uint8_t magz_0;
};

void SPI1_select( bool enable)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, enable ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

bool read_register_set( uint8_t _register, unsigned count, uint8_t * target)
{
	uint8_t TX_data[count] = {0};
	TX_data[0] = 0x80 | _register;
	HAL_StatusTypeDef result;
	SPI1_select( true);
	result = HAL_SPI_TransmitReceive( &hspi1, (const uint8_t *)TX_data, target, count, 1000);
//	result = HAL_SPI_TransmitReceive_IT( &hspi1, (const uint8_t *)TX_data, target, count);
	SPI1_select( false);
	return result == HAL_OK;
}

ROM uint8_t INIT_DATA[] = { RM3100_CMM_REG, CMM, RM3100_MX2_REG, CCP1, CCP0, CCP1, CCP0, CCP1, CCP0, RM3100_TMRC_REG, TMRC};
ROM uint8_t TRIGGER_TRIPLE_MEASUREMENT[] = { RM3100_POLL_REG, 0x70};
ROM uint8_t TRIGGER_SINGLE_Z_MEASUREMENT[] = { RM3100_POLL_REG, 0x40};

bool configure_RM3100(void)
{
	uint8_t regbuf[20];
	memset( regbuf, 0x55, 20);
	bool ok = read_register_set( RM3100_CCX1_REG, 7, regbuf);
	if( not ok)
		return false;

	if( not
		(
		(regbuf[1] == CCP1_DEFAULT) && (regbuf[2] == CCP0_DEFAULT) &&
		(regbuf[3] == CCP1_DEFAULT) && (regbuf[4] == CCP0_DEFAULT) &&
		(regbuf[5] == CCP1_DEFAULT) && (regbuf[6] == CCP0_DEFAULT)
		)
		  )
		return false;

	HAL_StatusTypeDef result;
	SPI1_select( true);
	result = HAL_SPI_Transmit( &hspi1, INIT_DATA+9, 2, 1000);
	SPI1_select( false);
	delay(1);
	SPI1_select( true);
	result = HAL_SPI_Transmit( &hspi1, INIT_DATA+2, 7, 1000);
	SPI1_select( false);
	delay(1);

	return result == HAL_OK;
}

bool read_RM3100( hw_data * target)
{
    return read_register_set( RM3100_MX2_REG, sizeof(hw_data), (uint8_t *)target);
}

bool read_RM3100_single( uint8_t  * target)
{
    return read_register_set( RM3100_MZ2_REG, 4, (uint8_t *)target);
}

uint8_t handshake[2];
mag_data measurement_result;
unsigned fail_count;
hw_data target;
uint64_t packed_result;

#if MEASURE_DAQ_TIME
uint64_t time_consumed;
#endif

extern "C" void RM3100_runnable( void *)
{
restart:

	bool result;
	uint64_t packed_result;
	uint64_t start_time;
	uint8_t data_read[3][4];

	volatile HAL_StatusTypeDef stat = HAL_CAN_Start(&hcan1);
	CAN_TxHeaderTypeDef Header = { 0x070, 0, 0, 0, 6, DISABLE};
	uint32_t mbx;

#ifdef TEST_HANDSHAKE
	result = read_register_set( RM3100_HSHAKE_REG, 2, handshake);
    if( not  result)
    {
    	++fail_count;
    	goto restart;
    }
#endif

	while( true)
	{
		result = configure_RM3100();
		if( result)
			break;
		delay(100);
	}

	delay( 10);

	for( synchronous_timer t( 10); true; t.sync())
	{
#if MEASURE_DAQ_TIME
		start_time = getTime_usec_privileged();
#endif

		uint8_t status_register[2];

		for( unsigned sample = 0; sample < 3; ++sample)
		{
			SPI1_select( true);
			result = HAL_SPI_Transmit( &hspi1, TRIGGER_SINGLE_Z_MEASUREMENT, 2, 10);
			SPI1_select( false);

			if( result != HAL_OK)
			{
				++fail_count;
				goto restart;
			}

			delay(3); // measurement takes 3ms

			while( true)
			{
				result = read_register_set( RM3100_STATUS_REG, 2, status_register);
				if( not  result)
				{
					++fail_count;
					goto restart;
				}
				if( (status_register[1] & 0x80) != 0)
					break;
				delay(1); // wait one more tick and try again
			}

			result = read_RM3100_single( &(data_read[ sample][0]));
			if( not  result)
			{
				++fail_count;
				goto restart;
			}
		}

	    measurement_result.magx = (( data_read[0][1] << 24) | ( data_read[0][2] << 16) | ( data_read[0][3] << 8)) >> 8;
	    measurement_result.magy = (( data_read[1][1] << 24) | ( data_read[1][2] << 16) | ( data_read[1][3] << 8)) >> 8;
	    measurement_result.magz = (( data_read[2][1] << 24) | ( data_read[2][2] << 16) | ( data_read[2][3] << 8)) >> 8;

		// pack result into single 64 bit datum: 16 + 16 + 16 bits -> 6 bytes telegram length
		packed_result =
				(((uint64_t)(measurement_result.magx) & 0xffff) |
				(((uint64_t)(measurement_result.magy) & 0xffff) << 16) |
				(((uint64_t)(measurement_result.magz) & 0xffff) << 32) );

		result = HAL_CAN_AddTxMessage( &hcan1, &Header, (uint8_t *)&packed_result, &mbx);
	    if( result != HAL_OK)
	    {
	    	++fail_count;
	    	goto restart;
	    }

#if MEASURE_DAQ_TIME
	    time_consumed = getTime_usec_privileged() - start_time;
#endif
	}
}

Task RM3100( RM3100_runnable, "MAG", 256);
