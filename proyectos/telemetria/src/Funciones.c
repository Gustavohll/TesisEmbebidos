/* Copyright 2014, Mariano Cerdeiro
 * Copyright 2014, Pablo Ridolfi
 * Copyright 2014, Juan Cecconi
 * Copyright 2014, Gustavo Muro
 *
 * This file is part of CIAA Firmware.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** \brief Blinking_echo example source file
 **
 ** This is a mini example of the CIAA Firmware to test the periodical
 ** task excecution and serial port funcionality.
 ** To run this sample in x86 plataform you must enable the funcionality of
 ** uart device setting a value of une or more of folowing macros defined
 ** in header file modules/plataforms/x86/inc/ciaaDriverUart_Internal.h
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking_echo example source file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 * MaCe         Mariano Cerdeiro
 * PR           Pablo Ridolfi
 * JuCe         Juan Cecconi
 * GMuro        Gustavo Muro
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20141019 v0.0.2   JuCe add printf in each task,
 *                        remove trailing spaces
 * 20140731 v0.0.1   PR   first functional version
 */

/*==================[inclusions]=============================================*/
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaak.h"            /* <= ciaa kernel header */
#include "telemetria.h"         /* <= own header */

/*==================[macros and definitions]=================================*/

void blinkled ()
{
	uint8_t outputs;
	//static int32_t fd_out;
	/* open CIAA digital outputs */
	//fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);
	/* read outputs */
    ciaaPOSIX_read(fd_out, &outputs, 1);

    /* update outputs with inputs */
    //outputs &= 0xF0;
    //outputs |= inputs & 0x0F;

    /* blink */
    outputs ^= 0x10;

    /* write */
    ciaaPOSIX_write(fd_out, &outputs, 1);
}
/*==================[end of file]============================================*/

char *itoa(int num, char *str, int radix) {
    char sign = 0;
    char temp[17];  //an int can only be 16 bits long
                    //at radix 2 (binary) the string
                    //is at most 16 + 1 null long.
    int temp_loc = 0;
    int digit;
    int str_loc = 0;

    //save sign for radix 10 conversion
    if (radix == 10 && num < 0) {
        sign = 1;
        num = -num;
    }

    //construct a backward string of the number.
    do {
        digit = (unsigned int)num % radix;
        if (digit < 10)
            temp[temp_loc++] = digit + '0';
        else
            temp[temp_loc++] = digit - 10 + 'A';
        num = (((unsigned int)num) / radix);
    } while ((unsigned int)num > 0);

    //now add the sign for radix 10
    if (radix == 10 && sign) {
        temp[temp_loc] = '-';
    } else {
        temp_loc--;
    }


    //now reverse the string.
    while ( temp_loc >=0 ) {// while there are still chars
        str[str_loc++] = temp[temp_loc--];
    }
    str[str_loc] = 0; // add null termination.

    return str;
}

