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

#include <math.h>

#include "driverlib.h"

#include "msp432_launchpad_temperature.h"

/*---------------------------------defines------------------------------------*/
/*---------------------------------typedefs-----------------------------------*/
/*--------------------------------prototypes----------------------------------*/
/*--------------------------------variables-----------------------------------*/

// Valores de calibracion de la temperatura de referencia del ADC
static uint32_t adcRefTempCal_1_2v_30;
static uint32_t adcRefTempCal_1_2v_85;

/*----------------------------------public------------------------------------*/

// Inicializacion del ADC14 para medir temperatura del sensor interno
void temperature_sensor_init(void)
{
  // Lee los valores de calibracion
  adcRefTempCal_1_2v_30 = TLV->ADC14_REF1P2V_TS30C;
  adcRefTempCal_1_2v_85 = TLV->ADC14_REF1P2V_TS85C;

  // Espera si el generador de tension de referencia este ocupado
  while (REF_A->CTL0 & REF_A_CTL0_GENBUSY);

  // Habilita la referencia interna de 1.2V
  REF_A->CTL0 |= REF_A_CTL0_VSEL_0 | REF_A_CTL0_ON;

  // Habilita el sensor de temperatura
  REF_A->CTL0 &= ~REF_A_CTL0_TCOFF;

  // Configura el ADC en pulse sample mode; ADC14_CTL0_SC trigger
  // ADC ON, temperature sample period > 5us
  ADC14->CTL0 |= ADC14_CTL0_SHT0_6 | ADC14_CTL0_ON | ADC14_CTL0_SHP;

  // Habilita el sensor de temperatura interno
  ADC14->CTL1 |= ADC14_CTL1_TCMAP;

  // ADC input channel A22 para medida de temperatura
  ADC14->MCTL[0] = ADC14_MCTLN_VRSEL_1 | ADC14_MCTLN_INCH_22;

  // Espera a que la tension de referencia sea estable
  while (!(REF_A->CTL0 & REF_A_CTL0_GENRDY));

  // ADC enable conversion
  ADC14->CTL0 |= ADC14_CTL0_ENC;
}

// Lee el valor de la medida de temperatura
float temperature_sensor_read(void)
{
  float temperature;

  // Inicia el muestreo y la conversion A/D
  ADC14->CTL0 |= ADC14_CTL0_SC;

  // Espera a que finalice la conversión A/D
  while (ADC14->CTL0 & ADC14_CTL0_BUSY);

  // Calcula el valor de temperatura en grados Celsius
  temperature = (((float) ADC14->MEM[0] - adcRefTempCal_1_2v_30) * (85 - 30)) /
          (adcRefTempCal_1_2v_85 - adcRefTempCal_1_2v_30) + 30.0f;

  return temperature;
}

/*---------------------------------private------------------------------------*/
/*--------------------------------interrupts----------------------------------*/
