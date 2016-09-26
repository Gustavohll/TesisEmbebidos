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
#include "Funciones.h"         /* <= own header */

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

/*==================[Funcion STRTOK]============================================*/
char *strtok(char * str, const char * delim)
{
    static char* p=0;
    if(str)
        p=str;
    else if(!p)
        return 0;
    str=p+strspn(p,delim);
    p=str+strcspn(str,delim);
    if(p==str)
        return p=0;
    p = *p ? *p=0,p+1 : 0;
    return str;
}

/*==================[Funcion STRTOK]============================================*/
int8_t *intstrtok(int8_t * str, const int8_t * delim)
{
    static int8_t* p=0;
    if(str)
        p=str;
    else if(!p)
        return 0;
    str=p+strspn(p,delim);
    p=str+strcspn(str,delim);
    if(p==str)
        return p=0;
    p = *p ? *p=0,p+1 : 0;
    return str;
}
/*==================[Funcion ATOI]============================================*/
// A simple atoi() function
int atoi(char *str)
{
    int res = 0; // Initialize result

    // Iterate through all characters of input string and
    // update result
    int i;
    for (i=0; str[i] != '\0'; i++)
        res = res*10 + str[i] - '0';

    // return result.
    return res;
}

/*==================[Funcion ATOI]============================================*/

double Myatof(char *str){

int len=0, n=0,i=0;
float f=1.0,val=0.0;

//counting length of String
while(str[len])len++;
//cheking for valid string
if(!len)return 0;

//Extracting Integer  part
while(i<len && str[i]!='.')
    n=10*n +(str[i++]-'0');

//checking if only Integer
if(i==len) return n;
i++;
while(i<len)
{
    f*=0.1;
    val+=f*(str[i++]-'0');
    }
    return(val+n);
}
/*==================[Funciones Parseo datos GPS]============================================*/

void formato_respuesta(struct DATOS_POSICION * p)
{
	char CGPS[]="AT+CGPSINF=2 \r";
	char CGPS_2[]="AT+CGPSINF=128 \r";
	char *pch;
	double aux1,aux2;
//	int8_t respuesta1[]="AT+CGPSINF=2\r\r\n2,180231,3437.130250,S,5824.354484,W,1,7,1.348750,45.631660,M,14.588299,M,,0000\r\nOK\r";
//	int8_t respuesta1[]="AT+CGPSINF=128 \r\r\n128,180255.000,19,09,2016,00,00\r\nOK\r\n";

	pch = strtok(respuesta,",");       // Tokenizamos (troceamos) la respuesta que tenemos en el array respuesta por las comas
									   // y el primer intervalo lo guardamos en pch (puntero char)
    //Analizamos ese intervalo guardado en pch para ver si es la respuesta que necesitamos
	if (ciaaPOSIX_strncmp(pch,CGPS,12)==0)
	//if (strcmp(pch,"2")==0)           // Si es la correcta, seguimos adelante
    {
        pch = strtok (NULL, ",");     		 // Pasamos al siguiente intervalo cortado de la respuesta
        p->hora = atoi(pch);			 		 // Guardo la parte entera del TIEMPO
        pch = strtok (NULL, ".");
        p->Lat = atoi(pch);
        pch = strtok (NULL, ",");
        p->DecLat = atoi(pch);
        pch = strtok (NULL, ",");
        pch = strtok (NULL, ".");
        p->Long = atoi(pch);
        pch = strtok (NULL, ",");
        p->DecLong = atoi(pch);
        pch = strtok (NULL, ",");
        pch = strtok (NULL, ",");
        p->validity = atoi(pch);		 			 // Guardo si la posicion es valida (A) o no (V)
    }
	if (ciaaPOSIX_strncmp(pch,CGPS_2,14)==0)
	{
		pch = strtok (NULL, ",");     		 // Pasamos al siguiente intervalo cortado de la respuesta
		pch = strtok (NULL, ",");
		p->dia = atoi(pch);			 	 // Guardo el dia
		pch = strtok (NULL, ",");
		p->mes = atoi(pch);			 	 // Guardo el mes
		pch = strtok (NULL, ",");
		p->anio = atoi(pch);			 	 // Guardo el año
	}
    return;
}


