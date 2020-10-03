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

#include "edu_boosterpack_buttons.h"

#include "driverlib.h"

/*---------------------------------defines------------------------------------*/

#define BUTTON_S1_PORT          ( GPIO_PORT_P3)
#define BUTTON_S1_PIN           ( GPIO_PIN5 )
#define BUTTON_S1_EDGE          ( GPIO_HIGH_TO_LOW_TRANSITION )
#define BUTTON_S1_IRQ           ( INT_PORT3 )
#define BUTTON_S1_IRQ_PRIO      ( 0xFF )

#define BUTTON_S2_PORT          ( GPIO_PORT_P5)
#define BUTTON_S2_PIN           ( GPIO_PIN1 )
#define BUTTON_S2_EDGE          ( GPIO_HIGH_TO_LOW_TRANSITION )
#define BUTTON_S2_IRQ           ( INT_PORT5 )
#define BUTTON_S2_IRQ_PRIO      ( 0xFF )

/*---------------------------------typedefs-----------------------------------*/

typedef struct
{
  uint16_t port;
  uint16_t pin;
  uint8_t edge;
  uint8_t irq;
  uint8_t irq_prio;
  callback_t callback;
} button_cfg_t;

/*--------------------------------prototypes----------------------------------*/

/*--------------------------------variables-----------------------------------*/

static button_cfg_t buttons[] =
{
  {BUTTON_S1_PORT, BUTTON_S1_PIN, BUTTON_S1_EDGE, BUTTON_S1_IRQ, BUTTON_S1_IRQ_PRIO, NULL},
  {BUTTON_S2_PORT, BUTTON_S2_PIN, BUTTON_S2_EDGE, BUTTON_S2_IRQ, BUTTON_S2_IRQ_PRIO, NULL},
};

/*----------------------------------public------------------------------------*/

void edu_boosterpack_buttons_init(void)
{
  uint16_t i;
  for (i = 0; i < MSP432_EDU_BOOSTERPACK_BUTTON_ELEMENTS; i++)
  {
    MAP_GPIO_setAsInputPin(buttons[i].port, buttons[i].pin);
    MAP_GPIO_clearInterruptFlag(buttons[i].port, buttons[i].pin);
    MAP_GPIO_interruptEdgeSelect(buttons[i].port, buttons[i].pin, buttons[i].edge);
    MAP_GPIO_enableInterrupt(buttons[i].port, buttons[i].pin);

    MAP_Interrupt_setPriority(buttons[i].irq, buttons[i].irq_prio);
    MAP_Interrupt_enableInterrupt(buttons[i].irq);
  }
}

void edu_boosterpack_buttons_set_callback(uint8_t button, callback_t callback)
{
  buttons[button].callback = callback;
}

void edu_boosterpack_buttons_clear_callback(uint8_t button)
{
  buttons[button].callback = NULL;
}

/*---------------------------------private------------------------------------*/

/*--------------------------------interrupts----------------------------------*/

void PORT3_IRQHandler(void)
{
  uint_fast16_t status;

  status = MAP_GPIO_getEnabledInterruptStatus(BUTTON_S1_PORT);
  MAP_GPIO_clearInterruptFlag(BUTTON_S1_PORT, status);

  if (((status & BUTTON_S1_PIN) == BUTTON_S1_PIN) &&
      (buttons[MSP432_EDU_BOOSTERPACK_BUTTON_S1].callback != NULL))
  {
    buttons[MSP432_EDU_BOOSTERPACK_BUTTON_S1].callback();
  }
}

void PORT5_IRQHandler(void)
{
  uint_fast16_t status;

  status = MAP_GPIO_getEnabledInterruptStatus(BUTTON_S2_PORT);
  MAP_GPIO_clearInterruptFlag(BUTTON_S2_PORT, status);

  if (((status & BUTTON_S2_PIN) == BUTTON_S2_PIN) &&
      (buttons[MSP432_EDU_BOOSTERPACK_BUTTON_S2].callback != NULL))
  {
    buttons[MSP432_EDU_BOOSTERPACK_BUTTON_S2].callback();
  }
}
