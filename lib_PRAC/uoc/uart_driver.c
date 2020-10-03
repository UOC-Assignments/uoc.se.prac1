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

#include "uart_driver.h"

#include "FreeRTOSConfig.h"


/*---------------------------------defines------------------------------------*/

#define UART_BASE                   ( EUSCI_A0_BASE )
#define UART_INTERRUPT              ( INT_EUSCIA0 )
#define UART_INTERRUPT_RX           ( EUSCI_A_UART_RECEIVE_INTERRUPT )
#define UART_INTERRUPT_TX           ( EUSCI_A_UART_TRANSMIT_INTERRUPT )
#define UART_IRQ_HANDLER            ( EUSCIA0_IRQHandler )

#define UART_BUFFER_TX_SIZE         ( 512 )
#define UART_BUFFER_RX_SIZE         ( 512 )

#define TX_BUFFER_RETRY_COUNT       ( 10 )

/*---------------------------------typedefs-----------------------------------*/
/*--------------------------------prototypes----------------------------------*/
/*--------------------------------variables-----------------------------------*/

/* Es configura l'UART en mode eUSCI per a una freqüència de rellotge de 40Mhz
 * i un baud rate de 115200 */

static const eUSCI_UART_Config uart_config =
{
  EUSCI_A_UART_CLOCKSOURCE_SMCLK,                   // SMCLK Clock Source
  21,                                               // BRDIV  = 21
  11,                                               // UCxBRF = 11
  34,                                               // UCxBRS = 34
  EUSCI_A_UART_NO_PARITY,                           // No Parity
  EUSCI_A_UART_LSB_FIRST,                           // LSB First
  EUSCI_A_UART_ONE_STOP_BIT,                        // One stop bit
  EUSCI_A_UART_MODE,                                // UART mode
  EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION     // Oversampling
};

static uint8_t uart_tx_buffer[UART_BUFFER_TX_SIZE];
static uint8_t uart_rx_buffer[UART_BUFFER_RX_SIZE];

static circ_buffer_t buffer_tx;
static circ_buffer_t buffer_rx;

static uart_rx_cb_t uart_rx_callback;

/*----------------------------------public------------------------------------*/

void uart_init(uart_rx_cb_t callback)
{
  /* Configure GPIO for UART peripheral */
  MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

  /* Initialize UART */
  MAP_UART_initModule(UART_BASE, &uart_config);

  /* Enable UART */
  MAP_UART_enableModule(UART_BASE);

  /* Enable UART interrupt in peripheral */
  MAP_UART_enableInterrupt(UART_BASE, UART_INTERRUPT_RX);

  /* Enable UART interrupt in NVIC */
  MAP_Interrupt_enableInterrupt(UART_INTERRUPT);
  MAP_Interrupt_setPriority(UART_INTERRUPT, configMAX_SYSCALL_INTERRUPT_PRIORITY);
  /* Enable master interrupt */
  MAP_Interrupt_enableMaster();

  /* Initialize transmit and receive circular buffers */
  circ_buffer_init(&buffer_tx, uart_tx_buffer, UART_BUFFER_TX_SIZE);
  circ_buffer_init(&buffer_rx, uart_rx_buffer, UART_BUFFER_TX_SIZE);

  uart_rx_callback = callback;
}

uint8_t uart_put_char(uint8_t data)
{
  /* Check if transmit circular buffer is full */
  if (circ_buffer_is_full(&buffer_tx) == false)
  {
    /* Add character to transmit circular buffer */
    circ_buffer_push(&buffer_tx, data);

    /* Enable UART transmit interrupt */
    MAP_UART_enableInterrupt(UART_BASE, UART_INTERRUPT_TX);
    MAP_Interrupt_enableInterrupt(UART_INTERRUPT);

    return 0;
  }

  return 1;
}

uint8_t uart_get_char(char *data)
{
  /* Check if receive circular buffer is empty */
  if (circ_buffer_is_empty(&buffer_rx) == false)
  {
    /* Read a character from receive circular buffer */
    *data = circ_buffer_pop(&buffer_rx);

    return 0;
  }

  return 1;
}

uint8_t uart_print(char* s)
{
  /* While character is different to NULL */
  while (*s)
  {
    uint8_t retryCount = 0;

    /* Try to put a character to the transmit buffer */
    while (uart_put_char((uint8_t)*s) != 0)
    {
      retryCount++;

      /* If we have gone above the retries limit */
      if (retryCount > TX_BUFFER_RETRY_COUNT)
      {
        return 1;
      }
    }

    /* Advance pointer to string */
    s++;
  }

  return 0;
}

/*---------------------------------private------------------------------------*/

/*--------------------------------interrupts----------------------------------*/

void EUSCIA0_IRQHandler(void)
{
  /* Read and clear UART interrupt status */
  uint32_t status = MAP_UART_getEnabledInterruptStatus(UART_BASE);
  MAP_UART_clearInterruptFlag(UART_BASE, status);

  /* If we have received a character */
  if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
  {
    uint8_t data;

    /* Read character from UART */
    data = MAP_UART_receiveData(UART_BASE);

    /* If receive circular buffer is not full */
    if (circ_buffer_is_full(&buffer_rx) == false)
    {

      // Si el caràcter rebut no és un backspace l'afegim al buffer,
      // si ho és, eliminem el darrer caràcter introduit al buffer.
      //
      // OBSERVACIÓ: El codi ASCII '\x7f' correspon al caràcter "backspace"
      // rebut d'un terminal sèrie específic (PuTTY) en un entorn amb WINDOWS 10.
      // És a dir, la funcionalitat de poder esborrar caràcters al terminal abans
      // de prèmer la tecla intro només funciona utilitzant aquest client / terminal
      // sèrie específic (al vídeo de la DEMO es pot observar el seu correcte funcionament).
      if (data!='\x7f'){
          /* Append character to receive circular buffer */
          circ_buffer_push(&buffer_rx, data);
      }
      else{
          // Si s'ha esborrat un caràcter (tecla  "backspace", es redueix
          // la mida del buffer circular per la cua (end) en una unitat
          buffer_rx.end--;
      }
      //Fem l'"echo" del caràcter rebut (data) de nou cap al terminal
      // per a poder veure el que estem escribint i/o esborrant.
      uart_put_char(data);
    }

    /* Execute UART receive callback */
    if ((uart_rx_callback != NULL) && data=='\r')
    {
      //circ_buffer_push(&buffer_rx, '\n'); //DEBUG
      uart_rx_callback(&buffer_rx);
      uart_print("\n\r");
    }
  }

  /* If we have transmitted a character */
  if (status & EUSCI_A_UART_TRANSMIT_INTERRUPT_FLAG)
  {
    /* If transmit circular buffer is not empty */
    if (circ_buffer_is_empty(&buffer_tx) == false)
    {
      uint8_t data;

      /* Get character from transmit circular buffer */
      data = circ_buffer_pop(&buffer_tx);

      /* Transmit character to UART */
      MAP_UART_transmitData(UART_BASE, data);
    }
    else
    {
      /* Transmit a new line character */
      MAP_UART_transmitData(UART_BASE, '\0');

      /* Disable UART transmit interrupt */
      MAP_UART_disableInterrupt(UART_BASE, UART_INTERRUPT_TX);
    }
  }
}
