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
	uint8_t out;
	//static int32_t fd_out;
	/* open CIAA digital outputs */
	//fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);
	/* read outputs */
    ciaaPOSIX_read(fd_out, &out, 1);

    /* update outputs with inputs */
    //outputs &= 0xF0;
    //outputs |= inputs & 0x0F;

    /* blink */
    out ^= 0x10;

    /* write */
    ciaaPOSIX_write(fd_out, &out, 1);
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

/*==================[Funciones Parseo datos GPS]============================================*/
//Guardo datos posicion en pos_data
void Guardo_datos_posicion(struct DATOS_POSICION * p,uint8_t *statusgps)
{
	char GGA[]="GPGGA";
	char RMC[]="GPRMC";
	char *pch;
	double aux1,aux2;
	int fecha;
	int8_t respuesta1[]="AT+CGPSINF=2\r\r\n2,180231,3437.130250,S,5824.354484,W,1,7,1.348750,45.631660,M,14.588299,M,,0000\r\nOK\r";
	//	                          $GPGGA,150212.000,3443.011810,S,05818.595681,W,1,5,3.44,0.983,M,14.456,M,,*5B

	//$GPRMC,134907.991,V,,,,,,,031016,,,N*41
	//$GPRMC,135032.000,A,3443.005606,S,05818.572033,W,0.000,0.0,010517,,,A*64
	//	int8_t respuesta1[]="AT+CGPSINF=128 \r\r\n128,180255.000,19,09,2016,00,00\r\nOK\r\n";

	pch = strtok(respuesta_gps,",");       // Tokenizamos (troceamos) la respuesta que tenemos en el array respuesta por las comas
									   // y el primer intervalo lo guardamos en pch (puntero char)
    //Analizamos ese intervalo guardado en pch para ver si es la respuesta que necesitamos
	if (ciaaPOSIX_strncmp(pch,GGA,5)==0)
	//if (strcmp(pch,"2")==0)           // Si es la correcta, seguimos adelante
    {
        pch = strtok (NULL, ".");     		 // Pasamos al siguiente intervalo cortado de la respuesta
        p->hora = atoi(pch);			 	 // Guardo la parte entera del TIEMPO
        pch = strtok (NULL, ",");
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
        *statusgps = p->validity;
        //p->hora = 112233;
        //p->anio = 2016;
    }
	if (ciaaPOSIX_strncmp(pch,RMC,5)==0)
	{
		//$GPRMC,135032.000,A,3443.005606,S,05818.572033,W,0.000,0.0,010517,,,A*64
		pch = strtok (NULL, ".");     		 // Pasamos al siguiente intervalo cortado de la respuesta
		p->hora = atoi(pch);			 	 // Guardo la parte entera del TIEMPO
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		p->validity = atoi(pch);		 	 // Guardo si la posicion es valida (A) o no (V)
		*statusgps = p->validity;
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
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		p->fecha = atoi(pch);			 // Guardo la fecha e formato ddmmaa
/*		if(fecha < 2016)
		{
			p->dia = 10;
			p->mes = 10;
			p->anio = 1970;
		}*/
	}
    return;
}

/*==================[Funciones Parseo datos GPS]============================================*/

void genero_paquete(struct DATOS_POSICION p,char *paq1,char *paq2)
{
	//char paq1[200];
	char str[10]=">RUSCIAA,";
	char str2[25]=";ID=C001;#IP0:";
	char str3[25]="< \x1A";
	ciaaPOSIX_strcat(paq1, str);				// Copio encabezado
	itoa(p.hora,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio hora en paquete
	/*if (p.dia < 10) ciaaPOSIX_strcat(paq1, "0");
	itoa(p.dia,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio dia en paquete
	if (p.mes < 10) ciaaPOSIX_strcat(paq1, "0");
	itoa(p.mes,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio mes en paquete
	itoa(p.anio,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio año en paquete
	*/
	itoa(p.fecha,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio fecha en paquete
	ciaaPOSIX_strcat(paq1, ",-");
	itoa(p.Lat,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio Lat en paquete
	ciaaPOSIX_strcat(paq1, ".");
	itoa(p.DecLat,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLat en paquete
	ciaaPOSIX_strcat(paq1, ",-");
	itoa(p.Long,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio Long en paquete
	ciaaPOSIX_strcat(paq1, ".");
	itoa(p.DecLong,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLong en paquete
	ciaaPOSIX_strcat(paq1, ",");
	itoa(p.validity,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio validad en paquete
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.IN1,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Digital in en paquete
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.IN2,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.IN3,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.IN4,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.ADC1,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Adc1 in en paquete
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.ADC2,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio ADC2 in en paquete
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.Modbus,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Modbus en paquete
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.event,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio n° de paquete
	ciaaPOSIX_strcat(paq2, str2);				// Copio Fin de Paquete1
	if (p.log < 1000) ciaaPOSIX_strcat(paq2, "0");
	if (p.log < 100)  ciaaPOSIX_strcat(paq2, "0");
	if (p.log < 10)   ciaaPOSIX_strcat(paq2, "0");
	itoa(p.log,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio n� log para ack

	ciaaPOSIX_strcat(paq2, str3);				// Copio Fin de Paquete3
return;
}

void genero_paquete_RUS07(struct DATOS_POSICION p,char *paq1,char *paq2)
{
	//char paq1[200];
	char str[10]=">RUS07,T,";
	char str2[25]=";ID=C001;#IP0:";
	char str3[25]="< \x1A";
	ciaaPOSIX_strcat(paq1, str);				// Copio encabezado
	itoa(p.hora,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio hora en paquete
	itoa(p.fecha,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio fecha en paquete
	ciaaPOSIX_strcat(paq2, ",V,");
	itoa(p.IN1,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Digital in en paquete
	ciaaPOSIX_strcat(paq2, ",W,");
	itoa(p.IN2,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",X,");
	itoa(p.IN3,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",Y,");
	itoa(p.IN4,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",Z,");
	itoa(p.ADC1,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Adc1 in en paquete
	ciaaPOSIX_strcat(paq2, ",a,");
	itoa(p.ADC2,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio ADC2 in en paquete
	ciaaPOSIX_strcat(paq2, ",b,");
	itoa(p.Modbus,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Modbus en paquete
	ciaaPOSIX_strcat(paq2, ",");
	itoa(p.event,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio n° de paquete
	ciaaPOSIX_strcat(paq2, str2);				// Copio Fin de Paquete1
	if (p.log < 1000) ciaaPOSIX_strcat(paq2, "0");
	if (p.log < 100)  ciaaPOSIX_strcat(paq2, "0");
	if (p.log < 10)   ciaaPOSIX_strcat(paq2, "0");
	itoa(p.log,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio n� log para ack

	ciaaPOSIX_strcat(paq2, str3);				// Copio Fin de Paquete3
return;
}
void genero_paquete_PI(struct DATOS_POSICION p,char *paq1,char *paq2)
{
	//char paq1[200];
//	RPI241207150026-3460180-05847823156025106 0000101311210000000720F500

	char str[10]=">RPI,";
	char str2[25]=";ID=C001;#IP0:";
	char str3[25]="< \x1A";
	ciaaPOSIX_strcat(paq1, str);				// Copio encabezado
	itoa(p.fecha,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio fecha en paquete
	itoa(p.hora,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio hora en paquete
	ciaaPOSIX_strcat(paq1, ",-");
	itoa(p.Lat,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio Lat en paquete
	itoa(p.DecLat,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLat en paquete
	ciaaPOSIX_strcat(paq1, "-0");
	itoa(p.Long,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio Long en paquete
	itoa(p.DecLong,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLong en paquete
	ciaaPOSIX_strcat(paq1, "156025106000");
	itoa(p.validity,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio validad en paquete
	ciaaPOSIX_strcat(paq1, "101311210000000720F500");
	ciaaPOSIX_strcat(paq1, ",");
	ciaaPOSIX_strcat(paq1, str2);				// Copio Fin de Paquete1
	if (p.log < 1000) ciaaPOSIX_strcat(paq1, "0");
	if (p.log < 100)  ciaaPOSIX_strcat(paq1, "0");
	if (p.log < 10)   ciaaPOSIX_strcat(paq1, "0");
	itoa(p.log,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio n� log para ack

	ciaaPOSIX_strcat(paq1, str3);				// Copio Fin de Paquete3
return;
}

/*==================[Funciones COLA]============================================*/

// insertar elemento a la lista
void put(struct DATOS_POSICION d,int *cola,int *cabeza,int *items)
{
    if ( *items == MAX_SIZE) *items=MAX_SIZE-1;
    if ( *cola >= MAX_SIZE) { *cola = 0; }
    log_data[*cola] = d;
    *cola +=1;
    *items +=1;
    return;
}

// retirar elemento de la lista
void get(struct DATOS_POSICION *d,int *cola,int *cabeza,int *items)
{
    //if ( empty() ) return -1;
    if (*items == 0) return;
    if ( *cabeza >= MAX_SIZE ) { *cabeza = 0; }
    *d = log_data[*cabeza];
    *cabeza +=1;
    *items -=1;
    return;
}





















