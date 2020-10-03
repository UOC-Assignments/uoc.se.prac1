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

/*--------------------------------includes------------------------------------*/

#include <stddef.h>

#include "driverlib.h"

#include "timer_driver.h"

/*---------------------------------defines------------------------------------*/

#define TIMER_BASE            ( TIMER_A2_BASE )
#define TIMER_MODE            ( TIMER_A_UP_MODE )
#define TIMER_REGISTER        ( TIMER_A_CAPTURECOMPARE_REGISTER_0 )
#define TIMER_INTERRUPT       ( INT_TA2_0 )
#define TIMER_IRQ_HANDLER     ( TA2_0_IRQHandler )
#define TIMER_PERIOD          ( 32768 )

/*---------------------------------typedefs-----------------------------------*/
/*--------------------------------prototypes----------------------------------*/
/*--------------------------------variables-----------------------------------*/

// Timer_A UpMode Configuration
static Timer_A_UpModeConfig timer_config =
{
  TIMER_A_CLOCKSOURCE_ACLK,           // ACLK source = 32.768 kHz
  TIMER_A_CLOCKSOURCE_DIVIDER_64,     // ACLK/64 = 512 Hz = 1.95 ms
  TIMER_PERIOD,                       // 1 seconds
  TIMER_A_TAIE_INTERRUPT_DISABLE,     // Disable Timer interrupt
  TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE, // Enable CCR0 interrupt
  TIMER_A_DO_CLEAR                    // Clear value
};

static callback_t timer_callback;

/*----------------------------------public------------------------------------*/

void timer_init(uint32_t period_ms, callback_t callback)
{
  /* Store timer callback */
  timer_callback = callback;

  /* Set the timer period (maximum = 127 s) */
  timer_config.timerPeriod = period_ms / 1.95;

  /* Configure timer in up mode */
  MAP_Timer_A_configureUpMode(TIMER_BASE, &timer_config);

  /* Enable timer interrupt */
  MAP_Interrupt_enableInterrupt(TIMER_INTERRUPT);

  /* Enable master interrupt */
  MAP_Interrupt_enableMaster();

  /* Let timer start counting */
  MAP_Timer_A_startCounter(TIMER_BASE, TIMER_MODE);
}

/*---------------------------------private------------------------------------*/

/*--------------------------------interrupts----------------------------------*/

void TA2_0_IRQHandler(void)
{
  /* Clear timer interrupt */
  MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);

  /* Execute timer callback */
  if (timer_callback != NULL)
  {
    timer_callback();
  }
}
