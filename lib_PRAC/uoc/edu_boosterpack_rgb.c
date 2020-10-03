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

#include "edu_boosterpack_rgb.h"

#include "msp432.h"
#include "driverlib.h"

/*---------------------------------defines------------------------------------*/

#define LED_RED_PORT            ( GPIO_PORT_P2 )
#define LED_RED_PIN             ( GPIO_PIN6 )
#define LED_RED_MODE            ( GPIO_PRIMARY_MODULE_FUNCTION )
#define LED_RED_TIMER           ( TIMER_A0_BASE )
#define LED_RED_TIMER_CCR       ( PM_TA0CCR3A )

#define LED_GREEN_PORT          ( GPIO_PORT_P2 )
#define LED_GREEN_PIN           ( GPIO_PIN4 )
#define LED_GREEN_MODE          ( GPIO_PRIMARY_MODULE_FUNCTION )
#define LED_GREEN_TIMER         ( TIMER_A0_BASE )
#define LED_GREEN_TIMER_CCR     ( PM_TA0CCR1A )

#define LED_BLUE_PORT           ( GPIO_PORT_P5 )
#define LED_BLUE_PIN            ( GPIO_PIN6 )
#define LED_BLUE_MODE           ( GPIO_PRIMARY_MODULE_FUNCTION )
#define LED_BLUE_TIMER          ( TIMER_A2_BASE )
#define LED_BLUE_TIMER_CCR      ( PM_TA1CCR1A )

/*---------------------------------typedefs-----------------------------------*/

typedef struct
{
  uint16_t port;
  uint16_t pin;
  uint8_t mode;
  uint32_t timer;
  Timer_A_PWMConfig* pwm_cfg;
} gpio_cfg_t;

/*--------------------------------prototypes----------------------------------*/

static void edu_boosterpack_rgb_gpio_init(void);
static void edu_boosterpack_rgb_set_pwm(uint8_t led, uint8_t duty_cycle);

/*--------------------------------variables-----------------------------------*/

static const uint8_t port2_remapping[] =
{
  PM_NONE, PM_NONE, PM_NONE, PM_NONE, LED_GREEN_TIMER_CCR, PM_NONE, LED_RED_TIMER_CCR, PM_NONE
};

static const uint8_t port5_remapping[] =
{
  PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, PM_NONE, LED_BLUE_TIMER_CCR, PM_NONE
};

static Timer_A_PWMConfig led_red_pwm_cfg =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  65535,
  TIMER_A_CAPTURECOMPARE_REGISTER_3,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

static Timer_A_PWMConfig led_green_pwm_cfg =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  65535,
  TIMER_A_CAPTURECOMPARE_REGISTER_1,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

static Timer_A_PWMConfig led_blue_pwm_cfg =
{
  TIMER_A_CLOCKSOURCE_SMCLK,
  TIMER_A_CLOCKSOURCE_DIVIDER_1,
  65535,
  TIMER_A_CAPTURECOMPARE_REGISTER_1,
  TIMER_A_OUTPUTMODE_RESET_SET,
  0
};

static const gpio_cfg_t leds[] =
{
  {LED_RED_PORT, LED_RED_PIN, LED_RED_MODE, LED_RED_TIMER, &led_red_pwm_cfg},
  {LED_GREEN_PORT, LED_GREEN_PIN, LED_GREEN_MODE, LED_GREEN_TIMER, &led_green_pwm_cfg},
  {LED_BLUE_PORT, LED_BLUE_PIN, LED_BLUE_MODE, LED_BLUE_TIMER, &led_blue_pwm_cfg}
};

/*----------------------------------public------------------------------------*/

void edu_boosterpack_rgb_init(void)
{
  edu_boosterpack_rgb_gpio_init();
}

void edu_boosterpack_rgb_on(uint8_t led)
{
  edu_boosterpack_rgb_set_pwm(led, 100);
}

void edu_boosterpack_rgb_off(uint8_t led)
{
  edu_boosterpack_rgb_set_pwm(led, 0);
}

void edu_boosterpack_rgb_pwm(uint8_t led, uint8_t duty_cycle)
{
  edu_boosterpack_rgb_set_pwm(led, duty_cycle);
}

void edu_boosterpack_rgb_pwm_all(uint8_t r, uint8_t g, uint8_t b)
{
  uint16_t r_ = 100 * r / 255;
  uint16_t g_ = 100 * g / 255;
  uint16_t b_ = 100 * b / 255;

  edu_boosterpack_rgb_set_pwm(EDU_BOOSTERPACK_LED_RED, r_);
  edu_boosterpack_rgb_set_pwm(EDU_BOOSTERPACK_LED_GREEN, g_);
  edu_boosterpack_rgb_set_pwm(EDU_BOOSTERPACK_LED_BLUE, b_);
}
/*---------------------------------private------------------------------------*/

static void edu_boosterpack_rgb_gpio_init(void)
{
  MAP_PMAP_configurePorts((const uint8_t *) port2_remapping, PMAP_P2MAP, 1, PMAP_DISABLE_RECONFIGURATION);
  MAP_PMAP_configurePorts((const uint8_t *) port5_remapping, PMAP_P5MAP, 1, PMAP_DISABLE_RECONFIGURATION);

  uint16_t i;
  for (i = 0; i < EDU_BOOSTERPACK_LED_ELEMENTS; i++)
  {
    MAP_GPIO_setAsPeripheralModuleFunctionOutputPin(leds[i].port, leds[i].pin, leds[i].mode);
  }
}

static void edu_boosterpack_rgb_set_pwm(uint8_t led, uint8_t duty_cycle)
{
  if (duty_cycle > 100)
  {
    duty_cycle = 100;
  }

  leds[led].pwm_cfg->dutyCycle = leds[led].pwm_cfg->timerPeriod * duty_cycle / 100;

  Timer_A_generatePWM(leds[led].timer, leds[led].pwm_cfg);
}

/*--------------------------------interrupts----------------------------------*/
