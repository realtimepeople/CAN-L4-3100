#include "FreeRTOS_wrapper.h"
#include "RM3100.hpp"
#include "stm32l4xx_hal.h"

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

#define CCP0    0xC8      // Cycle Count values
#define CCP1    0x00
#define CCP0_DEFAULT 0xC8 // Default Cycle Count values (used as a whoami check)
#define CCP1_DEFAULT 0x00
#define GAIN_CC50 20.0f   // LSB/uT
#define GAIN_CC100 38.0f
#define GAIN_CC200 75.0f

#define TMRC    0x94    // Update rate 150Hz
#define CMM     0x79    // read 3 axes and set data ready if 3 axes are ready

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
	uint8_t TX_data[count];
	TX_data[0] = 0x80 | _register;
	HAL_StatusTypeDef result;
	SPI1_select( true);
	result = HAL_SPI_TransmitReceive( &hspi1, (const uint8_t *)TX_data, target, count, 1000);
//	result = HAL_SPI_TransmitReceive_IT( &hspi1, (const uint8_t *)TX_data, target, count);
	SPI1_select( false);
	return result == HAL_OK;
}

ROM uint8_t INIT_DATA[] = { RM3100_CMM_REG, CMM, RM3100_MX2_REG, CCP1, CCP0, CCP1, CCP0, CCP1, CCP0, RM3100_TMRC_REG, TMRC};

bool configure_RM3100(void)
{
	uint8_t regbuf[20];
	memset( regbuf, 0x55, 20);
	bool ok = read_register_set( RM3100_CCX1_REG, 7, regbuf);
	if( not ok)
		return false;
#if 1 // do the test
	if( not
		(
		(regbuf[1] == CCP1_DEFAULT) && (regbuf[2] == CCP0_DEFAULT) &&
		(regbuf[3] == CCP1_DEFAULT) && (regbuf[4] == CCP0_DEFAULT) &&
		(regbuf[5] == CCP1_DEFAULT) && (regbuf[6] == CCP0_DEFAULT)
		)
		  )
		return false;
#endif

	HAL_StatusTypeDef result;
	SPI1_select( true);
	result = HAL_SPI_Transmit( &hspi1, INIT_DATA+9, 2, 1000);
	SPI1_select( false);
	delay(1);
	SPI1_select( true);
	result = HAL_SPI_Transmit( &hspi1, INIT_DATA, 2, 1000);
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
    return read_register_set( RM3100_MX2_REG, sizeof(hw_data) + 1, (uint8_t *)target);
}

uint8_t handshake[2];
mag_data measurement_result;
unsigned fail_count;
hw_data target;
uint64_t packed_result;
uint64_t time_consumed;

extern "C" void RM3100_runnable( void *)
{
restart:

	bool result;
	uint64_t packed_result;
	uint64_t start_time;

	volatile HAL_StatusTypeDef stat = HAL_CAN_Start(&hcan1);
	CAN_TxHeaderTypeDef Header = { 0x160, 0, 0, 0, 6, DISABLE};
	uint32_t mbx;

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
		uint8_t status_register[2];

		start_time = getTime_usec_privileged();

		result = read_register_set( RM3100_STATUS_REG, 2, status_register);
	    if( not  result)
	    {
	    	++fail_count;
	    	goto restart;
	    }

	    if( (status_register[1] & 0x80) == 0)
	    {
	    	++fail_count;
	    	goto restart;
	    }

	    result = read_register_set( RM3100_HSHAKE_REG, 2, handshake);
	    if( not  result)
	    {
	    	++fail_count;
	    	goto restart;
	    }

	    result = read_RM3100( &target);
	    if( not  result)
	    {
	    	++fail_count;
	    	goto restart;
	    }

	    measurement_result.magx = ((target.magx_2 << 24) | (target.magx_1 << 16) | (target.magx_0 << 8)) & 0xffff00;
		measurement_result.magy = ((target.magy_2 << 24) | (target.magy_1 << 16) | (target.magy_0 << 8)) & 0xffff00;
		measurement_result.magz = ((target.magz_2 << 24) | (target.magz_1 << 16) | (target.magz_0 << 8)) & 0xffff00;

		// pack result into single 64 bit datum: 16 + 16 + 16 bits -> 6 bytes telegram length
		packed_result = (measurement_result.magx >> 8) | ((uint64_t)(measurement_result.magy) << (16-8)) | ((uint64_t)(measurement_result.magz) << (2*16-8));
	    result = HAL_CAN_AddTxMessage( &hcan1, &Header, (uint8_t *)&packed_result, &mbx);
	    time_consumed = getTime_usec_privileged() - start_time;
	}
}

Task RM3100( RM3100_runnable, "MAG", 256);
