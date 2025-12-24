/*
 * RM3100.h
 *
 *  Created on: Dec 24, 2025
 *      Author: schaefer
 */

#ifndef RM3100_H_
#define RM3100_H_

#include "stdint.h"

class mag_data
{
public:
	int32_t magx;
	int32_t magy;
	int32_t magz;
};

extern "C" void RM3100_runnable( void *);
bool configure_RM3100(void);
bool read_RM3100( mag_data & target);

#endif /* RM3100_H_ */
