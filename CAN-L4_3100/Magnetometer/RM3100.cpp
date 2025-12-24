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
#define RM3100_HSHAKE_REG     0x34
#define RM3100_REVID_REG      0x36

#define CCP0    0xC8      // Cycle Count values
#define CCP1    0x00
#define CCP0_DEFAULT 0xC8 // Default Cycle Count values (used as a whoami check)
#define CCP1_DEFAULT 0x00
#define GAIN_CC50 20.0f   // LSB/uT
#define GAIN_CC100 38.0f
#define GAIN_CC200 75.0f

#define TMRC    0x94    // Update rate 150Hz
#define CMM     0x71    // read 3 axes and set data ready if 3 axes are ready

extern SPI_HandleTypeDef hspi1;

class hw_data
{
public:
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
//	result = HAL_SPI_TransmitReceive_IT( &hspi1, (const uint8_t *)TX_data, (uint8_t *)target, count);
	SPI1_select( false);
	return result == HAL_OK;
}

bool write_register_set( uint8_t register, unsigned count, const int8_t * target)
{

}

bool configure_RM3100(void)
{
	uint8_t regbuf[10];
	memset( regbuf, 0x55, 10);
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
	return true;
}

bool read_RM3100( mag_data & target)
{

}

volatile bool result;

extern "C" void RM3100_runnable( void *)
{
	result = configure_RM3100();

	while( true)
	{
		delay( 1000);
	}
}
