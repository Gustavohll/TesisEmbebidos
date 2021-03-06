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
#include "telemetria.h"       /* <= own header */
#include "Funciones.h"        /* <= own header */
//#include "stdlib.h"        	  /* <= own header */
//#include "string.h"           /* <= own header */

//#define Test_GSM
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
/*==================[Funcion ATOF]============================================*/
double atof(char *num)
{
	if (!num || !*num)
	         return 0;
	     double integerPart = 0;
	     double fractionPart = 0;
	     int divisorForFraction = 1;
	     int sign = 1;
	     boolean inFraction = FALSE;
	     /*Take care of +/- sign*/
	     if (*num == '-')
	     {
	         ++num;
	         sign = -1;
	     }
	     else if (*num == '+')
	     {
	         ++num;
	     }
	     while (*num != '\0')
	     {
	         if (*num >= '0' && *num <= '9')
	         {
	             if (inFraction)
	             {
	                 /*See how are we converting a character to integer*/
	                 fractionPart = fractionPart*10 + (*num - '0');
	                 divisorForFraction *= 10;
	             }
	             else
	             {
	                 integerPart = integerPart*10 + (*num - '0');
	             }
	         }
	         else if (*num == '.')
	         {
	             if (inFraction)
	                 return sign * (integerPart + fractionPart/divisorForFraction);
	             else
	                 inFraction = TRUE;
	         }
	         else
	         {
	             return sign * (integerPart + fractionPart/divisorForFraction);
	         }
	         ++num;
	     }
	     return sign * (integerPart + fractionPart/divisorForFraction);
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
	char cadena_aux[10];
	double aux,aux2;
	int fecha;
	#ifdef Test_GSM
	int8_t respuesta_gps[]="GPGGA,0150212.000,0,0,0,0,0,0,0,0,0,0,0,,*5B";
	//
	int8_t respuesta_gps2[]="GPRMC,0000009.991,0A,0,0,0,0,0,0,0031016,,,N*41";
	#endif
	//GPGGA,0150212.000,03443.011810,0S,005818.595681,0W,01,05,03.44,00.983,0M,014.456,0M,,*5B
	//GPGGA,0150212.000,0,0,0,0,0,0,0,0,0,0,0,,*5B
	//$GPRMC,134907.991,V,,,,,,,031016,,,N*41
	//$GPRMC,135032.000,A,3443.005606,S,05818.572033,W,0.000,0.0,010517,,,A*64
	//$GPRMC,232410.000,V,           , ,            , ,     ,   ,010517,,,N*4

	//3731.9404  ----> 37 + 31.9404/60 = 37.53234 degrees
	//10601.6986 ---> 106+1.6986/60 = 106.02831 degrees

	//	int8_t respuesta1[]="AT+CGPSINF=128 \r\r\n128,180255.000,19,09,2016,00,00\r\nOK\r\n";

	pch = strtok(respuesta_gps,",");       // Tokenizamos (troceamos) la respuesta que tenemos en el array respuesta por las comas
									   // y el primer intervalo lo guardamos en pch (puntero char)
    //Analizamos ese intervalo guardado en pch para ver si es la respuesta que necesitamos
	if (ciaaPOSIX_strncmp(pch,GGA,5)==0)
	//if (strcmp(pch,"2")==0)           // Si es la correcta, seguimos adelante
    {
		pch = strtok (NULL, ",");     		 // Pasamos al siguiente intervalo cortado de la respuesta
		aux = atof (pch);
		p->hora = aux;					 	 // Guardo la parte entera del TIEMPO (hhmmss)
		pch = strtok (NULL, ",");
		aux = atof (pch);					 //Parseo Latitud
		p->Lat = aux/100;
		aux2= (aux - (p->Lat * 100))/60;
		p->DecLat = aux2*100000;
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");			 //Parseo Longitud
		aux = atof (pch);
		p->Long = aux/100;
		aux2= (aux - (p->Long * 100))/60;
		p->DecLong = aux2*100000;
	}

	#ifdef Test_GSM
	pch = strtok(respuesta_gps2,",");
	#endif

	if (ciaaPOSIX_strncmp(pch,RMC,5)==0)
	{
		//$GPRMC,135032.000,A,3443.005606,S,05818.572033,W,0.000,0.0,010517,,,A*64
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		if (ciaaPOSIX_strncmp(pch,"0A",2)==0)	p->validity = 1;
		if (ciaaPOSIX_strncmp(pch,"0V",2)==0)	p->validity = 0;// Guardo si la posicion es valida (A) o no (V)
		*statusgps = p->validity;
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		pch = strtok (NULL, ",");
		p->fecha = atoi(pch);			 // Guardo la fecha e formato ddmmaa

	}
    return;
}

/*==================[Funciones Parseo datos GPS]============================================*/

//>RUS07,T,1502120,,V,0,W,0,X,0,Y,0,Z,0,a,0,b,0,0;ID=C001;#IP0:9987<
void genero_paquete_RUS07(struct DATOS_POSICION p,char *paq1,char *paq2)
{
	char str[10]=">RUS07,T,";
	char str2[25]=";ID=C001;#IP0:";
	char str3[25]="< \x1A";
	ciaaPOSIX_strcat(paq1, str);				// Copio encabezado
	if (p.Lat != 0)								// Si hay posicion pongo fecha, sino pongo 1970
	{
		if (p.fecha < 100000) ciaaPOSIX_strcat(paq1, "0");
		itoa(p.fecha,str,10);
		ciaaPOSIX_strcat(paq1, str);				// Copio fecha en paquete
	}else
	{
		ciaaPOSIX_strcat(paq1, "101070");
	}
	if (p.hora < 100000) ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 10000)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 1000)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 100)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 10)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 1) 	 ciaaPOSIX_strcat(paq1, "0");
	itoa(p.hora,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio hora en paquete
	ciaaPOSIX_strcat(paq2, ",V,");
	itoa(p.IN1,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Digital in en paquete
	itoa(p.IN2,str,10);
	ciaaPOSIX_strcat(paq2, str);
	itoa(p.IN3,str,10);
	ciaaPOSIX_strcat(paq2, str);
	itoa(p.IN4,str,10);
	ciaaPOSIX_strcat(paq2, str);
	ciaaPOSIX_strcat(paq2, ",W,");
	itoa(p.ADC1,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Adc1 in en paquete
	ciaaPOSIX_strcat(paq2, ",X,");
	itoa(p.ADC2,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio ADC2 in en paquete
	ciaaPOSIX_strcat(paq2, ",Y,");
	itoa(p.Modbus,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio Modbus en paquete
	ciaaPOSIX_strcat(paq2, str2);				// Copio Fin de Paquete1
	if (p.log < 1000) ciaaPOSIX_strcat(paq2, "0");
	if (p.log < 100)  ciaaPOSIX_strcat(paq2, "0");
	if (p.log < 10)   ciaaPOSIX_strcat(paq2, "0");
	itoa(p.log,str,10);
	ciaaPOSIX_strcat(paq2, str);				// Copio n� log para ack

	ciaaPOSIX_strcat(paq2, str3);				// Copio Fin de Paquete3
	ciaaPOSIX_printf("Paquete1: %s%s\n",paq1,paq2);
	return;
}

void genero_paquete_GP(struct DATOS_POSICION p,char *paq1)
{
//  char paq1[200];
//	RGP210517150104-3493011-058374710000663FF4F00

	char str[10]=">RGP";
	char str2[25]=";ID=C001;#LOG:";
	char str3[25]="< \x1A";
	ciaaPOSIX_strcat(paq1, str);				// Copio encabezado
	if (p.Lat != 0)								// Si hay posicion pongo fecha, sino pongo 1970
	{
		if (p.fecha < 100000) ciaaPOSIX_strcat(paq1, "0");
		itoa(p.fecha,str,10);
		ciaaPOSIX_strcat(paq1, str);				// Copio fecha en paquete
	}else
	{
		ciaaPOSIX_strcat(paq1, "101070");
	}
	if (p.hora < 100000) ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 10000)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 1000)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 100)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 10)	 ciaaPOSIX_strcat(paq1, "0");
	if (p.hora < 1) 	 ciaaPOSIX_strcat(paq1, "0");
	itoa(p.hora,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio hora en paquete
	ciaaPOSIX_strcat(paq1, "-");
	if(p.Lat == 0)  ciaaPOSIX_strcat(paq1, "00");
	itoa(p.Lat,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio Lat en paquete
	if (p.DecLat < 1000) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLat < 100) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLat < 10) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLat < 1) ciaaPOSIX_strcat(paq1, "0");
	itoa(p.DecLat,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLat en paquete
	ciaaPOSIX_strcat(paq1, "-0");
	if(p.Long == 0)  ciaaPOSIX_strcat(paq1, "00");
	itoa(p.Long,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio Long en paquete
	if (p.DecLong < 10000) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLong < 1000) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLong < 100) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLong < 10) ciaaPOSIX_strcat(paq1, "0");
	if (p.DecLong < 1) ciaaPOSIX_strcat(paq1, "0");
	itoa(p.DecLong,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLong en paquete
	ciaaPOSIX_strcat(paq1, "0000663FF4F");
	itoa(p.event,str,10);
	if (p.event < 10) ciaaPOSIX_strcat(paq1, "0");
	ciaaPOSIX_strcat(paq1, str);				// Copio DecLong en paquete
	ciaaPOSIX_strcat(paq1, str2);				// Copio Fin de Paquete1
	if (p.log < 1000) ciaaPOSIX_strcat(paq1, "0");
	if (p.log < 100)  ciaaPOSIX_strcat(paq1, "0");
	if (p.log < 10)   ciaaPOSIX_strcat(paq1, "0");
	itoa(p.log,str,10);
	ciaaPOSIX_strcat(paq1, str);				// Copio n� log para ack

	ciaaPOSIX_strcat(paq1, str3);				// Copio Fin de Paquete3
	ciaaPOSIX_printf("PaqueteRGP: %s\n",paq1);
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





















