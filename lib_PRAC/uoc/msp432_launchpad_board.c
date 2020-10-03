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

#include "msp432_launchpad_board.h"

#include "driverlib.h"

/*---------------------------------defines------------------------------------*/

#define LED_RED_PORT            ( GPIO_PORT_P1 )
#define LED_RED_PIN             ( GPIO_PIN0 )
#define LED_RED1_PORT           ( GPIO_PORT_P2 )
#define LED_RED1_PIN            ( GPIO_PIN0 )
#define LED_GREEN_PORT          ( GPIO_PORT_P2 )
#define LED_GREEN_PIN           ( GPIO_PIN1 )
#define LED_BLUE_PORT           ( GPIO_PORT_P2 )
#define LED_BLUE_PIN            ( GPIO_PIN2 )

#define DEBUG1_PORT             ( GPIO_PORT_P4 )
#define DEBUG1_PIN              ( GPIO_PIN5 )
#define DEBUG2_PORT             ( GPIO_PORT_P4 )
#define DEBUG2_PIN              ( GPIO_PIN7 )
#define DEBUG3_PORT             ( GPIO_PORT_P5 )
#define DEBUG3_PIN              ( GPIO_PIN4 )
#define DEBUG4_PORT             ( GPIO_PORT_P5 )
#define DEBUG4_PIN              ( GPIO_PIN5 )

#define BUTTON_S1_PORT          ( GPIO_PORT_P1 )
#define BUTTON_S1_PIN           ( GPIO_PIN1 )
#define BUTTON_S1_EDGE          ( GPIO_HIGH_TO_LOW_TRANSITION )

#define BUTTON_IRQ              ( INT_PORT1 )
#define BUTTON_IRQ_PRIO         ( 0xFF )

#define BUTTON_S2_PORT          ( GPIO_PORT_P1)
#define BUTTON_S2_PIN           ( GPIO_PIN4 )
#define BUTTON_S2_EDGE          ( GPIO_HIGH_TO_LOW_TRANSITION )
#define BUTTON_S2_IRQ_PRIO      ( 0xFF )

#define LCD_CS_PORT             ( GPIO_PORT_P5 )
#define LCD_CS_PIN              ( GPIO_PIN0 )

/*---------------------------------typedefs-----------------------------------*/

typedef struct
{
  uint16_t port;
  uint16_t pin;
} gpio_cfg_t;

typedef struct
{
  uint16_t port;
  uint16_t pin;
  uint8_t edge;
  callback_t callback;
} button_cfg_t;

/*--------------------------------prototypes----------------------------------*/

static void board_init_leds(void);
static void board_init_debug(void);
static void board_init_buttons(void);
static void board_init_clk(void);

/*--------------------------------variables-----------------------------------*/

static gpio_cfg_t leds[] =
{
  {LED_RED_PORT,   LED_RED_PIN},
  {LED_RED1_PORT,  LED_RED1_PIN},
  {LED_GREEN_PORT, LED_GREEN_PIN},
  {LED_BLUE_PORT,  LED_BLUE_PIN},
};

static button_cfg_t buttons[] =
{
  {BUTTON_S1_PORT, BUTTON_S1_PIN, BUTTON_S1_EDGE, NULL},
  {BUTTON_S2_PORT, BUTTON_S2_PIN, BUTTON_S2_EDGE, NULL},
};

static gpio_cfg_t debug[] =
{
  {DEBUG1_PORT, DEBUG1_PIN},
  {DEBUG2_PORT, DEBUG2_PIN},
  {DEBUG3_PORT, DEBUG3_PIN},
  {DEBUG4_PORT, DEBUG4_PIN},
};

/*----------------------------------public------------------------------------*/

void board_init(void)
{
  board_init_leds();
  board_init_debug();
  board_init_buttons();
  board_init_clk();
}

void board_lock(void)
{
  led_on(MSP432_LAUNCHPAD_LED_RED);
  while(1)
    ;
}

void board_buttons_set_callback(uint8_t button, callback_t callback)
{
  if (button < MSP432_LAUNCHPAD_BUTTON_ELEMENTS)
  {
    buttons[button].callback = callback;
  }
}

void board_buttons_clear_callback(uint8_t button)
{
  if (button < MSP432_LAUNCHPAD_BUTTON_ELEMENTS)
  {
    buttons[button].callback = NULL;
  }
}

void led_on(uint8_t led)
{
  if (led < MSP432_LAUNCHPAD_LED_ELEMENTS)
  {
    MAP_GPIO_setOutputHighOnPin(leds[led].port, leds[led].pin);
  }
}

void led_off(uint8_t led)
{
  if (led < MSP432_LAUNCHPAD_LED_ELEMENTS)
  {
    MAP_GPIO_setOutputLowOnPin(leds[led].port, leds[led].pin);
  }
}

void led_toggle(uint8_t led)
{
  if (led < MSP432_LAUNCHPAD_LED_ELEMENTS)
  {
    MAP_GPIO_toggleOutputOnPin(leds[led].port, leds[led].pin);
  }
}

void debug_on(uint8_t gpio)
{
  if (gpio < MSP432_LAUNCHPAD_DEBUG_ELEMENTS)
  {
    MAP_GPIO_setOutputHighOnPin(debug[gpio].port, debug[gpio].pin);
  }
}

void debug_off(uint8_t gpio)
{
  if (gpio < MSP432_LAUNCHPAD_DEBUG_ELEMENTS)
  {
    MAP_GPIO_setOutputLowOnPin(debug[gpio].port, debug[gpio].pin);
  }
}

void debug_toggle(uint8_t gpio)
{
  if (gpio < MSP432_LAUNCHPAD_DEBUG_ELEMENTS)
  {
    MAP_GPIO_toggleOutputOnPin(debug[gpio].port, debug[gpio].pin);
  }
}

/*---------------------------------private------------------------------------*/

static void board_init_leds(void)
{
  uint16_t i;
  for (i = 0; i < MSP432_LAUNCHPAD_LED_ELEMENTS; i++)
  {
    MAP_GPIO_setOutputLowOnPin(leds[i].port, leds[i].pin);
    MAP_GPIO_setAsOutputPin(leds[i].port, leds[i].pin);
  }
}

static void board_init_debug(void)
{
  uint16_t i;
  for (i = 0; i < MSP432_LAUNCHPAD_DEBUG_ELEMENTS; i++)
  {
    MAP_GPIO_setOutputLowOnPin(debug[i].port, debug[i].pin);
    MAP_GPIO_setAsOutputPin(debug[i].port, debug[i].pin);
  }
}

static void board_init_buttons(void)
{
  uint16_t i;
  for (i = 0; i < MSP432_LAUNCHPAD_BUTTON_ELEMENTS; i++)
  {
    MAP_GPIO_setAsInputPinWithPullUpResistor(buttons[i].port, buttons[i].pin);
    MAP_GPIO_clearInterruptFlag(buttons[i].port, buttons[i].pin);
    MAP_GPIO_interruptEdgeSelect(buttons[i].port, buttons[i].pin, buttons[i].edge);
    MAP_GPIO_enableInterrupt(buttons[i].port, buttons[i].pin);
  }

  MAP_Interrupt_setPriority(BUTTON_IRQ, BUTTON_IRQ_PRIO);
  MAP_Interrupt_enableInterrupt(BUTTON_IRQ);
}

static void board_init_clk(void)
{
  /* Halt the Watchdog */
  MAP_WDT_A_holdTimer();

  /* Ensure the FPU is enabled */
  MAP_FPU_enableModule();

  /* Configuring pins for peripheral/crystal */
  // MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);
  MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);

  /* Set the clock frequency in the code */
  MAP_CS_setExternalClockSourceFrequency(32768, 40000000);

  /* Change VCORE to 1 to support the 48 MHz frequency */
  MAP_PCM_setCoreVoltageLevel(PCM_VCORE1);

  /* Set FLASH wait states */
  MAP_FlashCtl_setWaitState(FLASH_BANK0, 2);
  MAP_FlashCtl_setWaitState(FLASH_BANK1, 2);

  /* Starting HFXT/LFXT in non-bypass mode without a timeout */
  // MAP_CS_startHFXT(false);
  MAP_CS_startLFXT(false);

  /* Setting the DCO Frequency to 40 MHz */
  MAP_CS_setDCOFrequency(40000000);

  /* Initializing MCLK, HSMCLK, SMCLK and ACLK based on DCO and LFXT */
  MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1);
  MAP_CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

  /* Enable CPU after IRQ */
  MAP_Interrupt_disableSleepOnIsrExit();

  /* Globally enable interrupts */
  MAP_Interrupt_enableMaster();

  /* Ensure LCD CS pin is not floating to avoid problems */
  MAP_GPIO_setAsOutputPin(LCD_CS_PORT, LCD_CS_PIN);
  MAP_GPIO_setOutputHighOnPin(LCD_CS_PORT, LCD_CS_PIN);
}

/*--------------------------------interrupts----------------------------------*/

void PORT1_IRQHandler(void)
{
  uint_fast16_t status;

  status = MAP_GPIO_getEnabledInterruptStatus(BUTTON_S1_PORT);
  MAP_GPIO_clearInterruptFlag(BUTTON_S1_PORT, status);

  if (((status & BUTTON_S1_PIN) == BUTTON_S1_PIN) &&
      (buttons[MSP432_LAUNCHPAD_BUTTON_S1].callback != NULL))
  {
    buttons[MSP432_LAUNCHPAD_BUTTON_S1].callback();
  }

  if (((status & BUTTON_S2_PIN) == BUTTON_S2_PIN) &&
      (buttons[MSP432_LAUNCHPAD_BUTTON_S2].callback != NULL))
  {
    buttons[MSP432_LAUNCHPAD_BUTTON_S2].callback();
  }
}
