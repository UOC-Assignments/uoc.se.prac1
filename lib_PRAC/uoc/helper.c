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

#include "helper.h"

/*---------------------------------defines------------------------------------*/
/*---------------------------------typedefs-----------------------------------*/
/*--------------------------------prototypes----------------------------------*/
/*--------------------------------variables-----------------------------------*/
/*----------------------------------public------------------------------------*/

void reverse_string(char *str, int len)
{
  int i=0, j=len-1, temp;

  while (i<j)
  {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++; j--;
  }
}

int int_to_string(int x, char str[], int d)
{
  int i = 0;

  while (x)
  {
    str[i++] = (x%10) + '0';
    x = x/10;
  }

  while (i < d)
  {
    str[i++] = '0';
  }

  reverse_string(str, i);

  str[i] = '\0';

  return i;
}

void ftoa(float n, char *res, int afterpoint)
{
  int ipart = (int)n;

  float fpart = n - (float)ipart;

  int i = int_to_string(ipart, res, 0);

  if (afterpoint != 0)
  {
    res[i] = '.';

    fpart = fpart * pow(10, afterpoint);

    int_to_string((int)fpart, res + i + 1, afterpoint);
  }
}

/*---------------------------------private------------------------------------*/
/*--------------------------------interrupts----------------------------------*/
