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

#include <edu_boosterpack_accelerometer.h>
#include <string.h>
#include "driverlib.h"

/*---------------------------------defines------------------------------------*/

/*---------------------------------typedefs-----------------------------------*/
/*--------------------------------prototypes----------------------------------*/
/*--------------------------------variables-----------------------------------*/
adc_callback_t adc_callback_func;
adc_result results_buffer;
/*----------------------------------public------------------------------------*/

void edu_boosterpack_accelerometer_init(void){
    /* Zero-filling results buffer */
    memset(results_buffer, 0x00, ACCEL_AXIS);
    /* Setting reference voltage to 2.5  and enabling reference */
    MAP_REF_A_setReferenceVoltage(REF_A_VREF2_5V);
    MAP_REF_A_enableReferenceVoltage();
    /* Initialize ADC and configure clock to MCLK/1/1 */
    MAP_ADC14_enableModule();
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK,  ADC_PREDIVIDER_1,  ADC_DIVIDER_1, 0);
    /* Configuring GPIOs for Analog In */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
    /* Configuring ADC conversion mode and ADC Memory (ADC_MEM0 - ADC_MEM2 (A14, A13, A11) */
    MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A14, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A13, false);
    MAP_ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A11, false);
    /* Enabling the interrupt when a conversion on channel 2 (end of sequence) is complete and enabling conversions */
    MAP_ADC14_enableInterrupt(ADC_INT2);
    // Set up priority in ADC14 interrupt
    MAP_Interrupt_setPriority(INT_ADC14, 0xA0);
    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    /* Setting up the sample timer to automatically step through the sequence to convert.*/
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
}

void edu_boosterpack_accelerometer_read(void){
    /* Trigger the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();
}

void edu_boosterpack_accelerometer_disable(void){
    /* Disable conversion */
    ADC14_disableConversion ();
    /* Disable interrupts */
    MAP_ADC14_disableInterrupt(ADC_INT2);
    MAP_Interrupt_disableInterrupt(INT_ADC14);
    /* Disable module */
    MAP_ADC14_disableModule();
}

void edu_boosterpack_accelerometer_set_callback(adc_callback_t callback){
    adc_callback_func = callback;
}

void edu_boosterpack_accelerometer_clear_callback(void){
    adc_callback_func = NULL;
}

/*---------------------------------private------------------------------------*/

/*--------------------------------interrupts----------------------------------*/

void ADC_IRQHandler(void)
{
    uint64_t status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
    if (ADC_INT2 & status){
        MAP_ADC14_getMultiSequenceResult(results_buffer);
        /* call callback routine */
        adc_callback_func(results_buffer);
    }
}

