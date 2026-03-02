/*
 * param_set.h
 *
 *  Created on: Feb 24, 2026
 *      Author: Martin Stastny
 */

#ifndef INC_PARAM_SET_H_
#define INC_PARAM_SET_H_

//#include "meas.h"
#include "main.h"
#include "meas.h"

Status_t systemParamInit(void);

Status_t setParams(uint8_t servo_on,uint8_t gain);

#endif /* INC_PARAM_SET_H_ */
