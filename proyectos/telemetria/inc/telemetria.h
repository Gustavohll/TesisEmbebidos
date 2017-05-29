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

#ifndef _BLINKING_H_
#define _BLINKING_H_
/** \brief Blinking example header file
 **
 ** This is a mini example of the CIAA Firmware
 **
 **/

/** \addtogroup CIAA_Firmware CIAA Firmware
 ** @{ */
/** \addtogroup Examples CIAA Firmware Examples
 ** @{ */
/** \addtogroup Blinking Blinking example header file
 ** @{ */

/*
 * Initials     Name
 * ---------------------------
 *
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * yyyymmdd v0.0.1 initials initial version
 */

/*==================[inclusions]=============================================*/

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/
int32_t fd_out;   // no es static , porque sino no funciona, ver????
int32_t fd_outleds;

int8_t respuesta[100];
int8_t respuesta_gps[100];
#define MAX_SIZE 10
#define TIME_POSITION 6
/*==================[macros and definitions]=================================*/
//#define Test_AnalogInTask
//#define Test_DigitalInTask
//#define Test_LedTask
#define Test_GSM
//#define Test_SerialGsmTask
#define Test_SerialGpsTask
/*==================[TABLA DE EVENTOS]=================================*/

#define CAMBIO_POS 8
#define CAMBIO_IN1 10
#define CAMBIO_IN2 11
#define CAMBIO_IN3 12
#define CAMBIO_IN4 13
#define CAMBIO_ADC1 14
#define CAMBIO_ADC2 15


//static int cola = 0;
//static int cabeza = 0;
//static int items=0;
static int Send_Event=0;
static char paquete_1[100];
static char paquete_2[100];
static char paquete_3[100];
static struct DATOS_POSICION
{
	int fecha;
	int dia;
	int mes;
	int anio;
	long hora;
	int Lat;
	long DecLat;
	long Long;
	long DecLong;
	int8_t validity;
	int IN1;
	int IN2;
	int IN3;
	int IN4;
	int ADC1;
	int ADC2;
	float Modbus;
	int event;
	int log;
} pos_data,send_data,log_data[MAX_SIZE];

/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/


#endif /* #ifndef _BLINKING_H_ */

