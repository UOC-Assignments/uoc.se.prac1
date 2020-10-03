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

#ifndef CIRC_BUFFER_H_
#define CIRC_BUFFER_H_

/*--------------------------------includes------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

/*---------------------------------defines------------------------------------*/

/*---------------------------------typedefs-----------------------------------*/

typedef struct
{
  uint16_t begin;
  uint16_t end;
  uint8_t *buffer;
  uint16_t size;
} circ_buffer_t;

/*--------------------------------prototypes----------------------------------*/

void circ_buffer_init(circ_buffer_t* cb, uint8_t* buffer, uint16_t size);
void circ_buffer_push(circ_buffer_t* cb, uint8_t data);
uint8_t circ_buffer_pop(circ_buffer_t* cb);
bool circ_buffer_is_empty(circ_buffer_t* cb);
bool circ_buffer_is_full(circ_buffer_t* cb);

/*--------------------------------variables-----------------------------------*/
/*----------------------------------public------------------------------------*/
/*---------------------------------private------------------------------------*/
/*--------------------------------interrupts----------------------------------*/

#endif /* CIRC_BUFFER_H_ */
