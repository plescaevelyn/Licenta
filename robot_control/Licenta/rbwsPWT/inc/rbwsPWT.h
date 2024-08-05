/*
 * rbwsPWT.h
 *
 *  Created on: Apr 16, 2024
 *      Author: PLE1CLJ
 */

#ifndef INC_RBWSPWT_H_
#define INC_RBWSPWT_H_

#include "fsl_pwt.h"
#include "rbwsMotorControl.h"

/*********************/
/* Macro definitions */
/*********************/
#define PWT_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_BusClk) /*source clock for PWT driver */

/***********************/
/* Function prototypes */
/***********************/
void rbwsPWT_init(pwt_input_select_t pwt_port);
void rbwsPWT_readPulse(uint8_t mode);
int rbwsPWT_main(void);

#endif /* INC_RBWSPWT_H_ */
