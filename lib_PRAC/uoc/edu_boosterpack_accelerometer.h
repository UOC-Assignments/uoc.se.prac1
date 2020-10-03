/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MSP432_EDU_BOOSTERPACK_ACCELEROMETER_H_
#define MSP432_EDU_BOOSTERPACK_ACCELEROMETER_H_

/*--------------------------------includes------------------------------------*/

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "callback.h"

/*---------------------------------defines------------------------------------*/
#define ACCEL_AXIS      3
/*---------------------------------typedefs-----------------------------------*/
typedef void (*adc_callback_t)(uint16_t*);
typedef uint16_t adc_result[ACCEL_AXIS];

/*--------------------------------prototypes----------------------------------*/

void edu_boosterpack_accelerometer_init(void);
void edu_boosterpack_accelerometer_set_callback(adc_callback_t callback);
void edu_boosterpack_accelerometer_read(void);
void edu_boosterpack_accelerometer_clear_callback(void);
void edu_boosterpack_accelerometer_disable(void);

/*--------------------------------variables-----------------------------------*/
/*----------------------------------public------------------------------------*/
/*---------------------------------private------------------------------------*/
/*--------------------------------interrupts----------------------------------*/

#endif /* MSP432_EDU_BOOSTERPACK_ACCELEROMETER_H_ */
