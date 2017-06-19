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
/*==================[inclusions]=============================================*/
#include "os.h"               /* <= operating system header */
#include "ciaaPOSIX_stdio.h"  /* <= device handler header */
#include "ciaaPOSIX_string.h" /* <= string header */
#include "ciaaPOSIX_stdlib.h"
#include "ciaak.h"            /* <= ciaa kernel header */
#include "telemetria.h"         /* <= own header */
#include "Funciones.h"         /* <= own header */
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
/** \brief File descriptor for ADC
 *
 * Device path /dev/serial/aio/in/0
 */
static int32_t fd_adc_0;
static int32_t fd_adc_1;

/** \brief File descriptor for digital input ports
 *
 * Device path /dev/dio/in/0
 */
static int32_t fd_in;

/** \brief File descriptor for digital output ports
 *
 * Device path /dev/dio/out/0
 */
//static int32_t fd_out;

/** \brief File descriptor of the gps uart
 *
 * Device path /dev/serial/uart/0
 */
static int32_t fd_uart_gps;
/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart_usb;
/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int32_t fd_uart_gsm;

/** \brief Periodic Task Counter
 *
 */
static uint32_t Periodic_Task_Counter;
static uint32_t Contador_In1;
static uint32_t Contador_In2;
static uint32_t Contador_In3;
static uint32_t Contador_In4;

/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int estado_in1=0;
static int estado_in2=0;
static int estado_in3=0;
static int estado_in4=0;
static int cambioestado=0;
static char valor_in[4];

uint8_t statusgps=0;
uint8_t statusgsm=0;

static int valor_ch1;
static int valor_ch3;
static int estado_ch1=0;
static int estado_ch3=0;

static int items=0;
static int cola = 0;
static int cabeza = 0;
/*==================[Mutex UART GSM]===============================*/

//static uint8_t MutexUartGsm=FALSE;

/*==================[Variables Task GSM]===============================*/

enum ESTADOS_GSM
			{
				 RED=0,SET,SEND,ACKNOLEGE,ERROR,DELAY,ultimo_estado_gsm
			};
int estado_gsm;
enum ESTADOS_SEND
			{
				 SEND_ESPERA=0,SEND1,ACK1,SEND2,ACK2,ultimo_estado_send
			};
int estado_send;
int FSM_inicializada=0,i=0,h=0,x=0,delay=0;

#define Movistar
//#define Claro

#ifdef Claro
	char APN[50]="AT+CSTT=\"internet.ctimovil.com.ar\",\"gprs\",\"gprs\" \r";
	char R_APN[]="AT+CSTT=\"internet.ctimovil.com.ar\",\"gprs\",\"gprs\" \r\r\nOK\r\n";
#endif

#ifdef Movistar
	char APN[50]="AT+CSTT=\"m2m.movistar\",\"movistar\",\"movistar\" \r";
	char R_APN[]="AT+CSTT=\"m2m.movistar\",\"movistar\",\"movistar\" \r\r\nOK\r\n";
#endif
//m2m.movistar,movistar,movistar
//INTERNET.GPRS.UNIFON.COM.AR,WAP,WAP

char IP[]="AT+CIPSTART=\"UDP\",\"131.255.4.29\",\"6097\" \r";
//char IP[]="AT+CIPSTART=\"UDP\",\"190.12.119.150\",\"6097\" \r";
//>RPF041207152350-3460160-05847853000000300FF01;ID=1020;#IP0:00E0<

//AT+CIPSTART="UDP","190.12.119.150","6097"
char last_position []=">RPF041207152350-3460160-05847853000000300FF01;ID=1020;#IP0:00E0< \x1A \r\n";
//char last_position []="< \x1A";

char CIIR[]="AT+CIICR \r";
char CIFSR[]="AT+CIFSR \r";
char CGATT[]="AT+CGATT? \r";
char CIPSEND[]="AT+CIPSEND \r";
char finrespuesta[]=" \r\n";
char CIPSHUT[]="AT+CIPSHUT \r";

//char GPS[]="AT+CGPSINF=2 \r";
//char GPS_2[]="AT+CGPSINF=128 \r";
char GPSPWR[]="AT+CGPSPWR=1 \r";
char GPSRST[]="AT+CGPSRST=1 \r";
char GPSOUT[]="AT+CGPSOUT=34 \r";
char GPSIPR[]="AT+CGPSIPR=9600 \r";
char GPGGA[]="GPGGA";

char R_GPSPWR[]="AT+CGPSPWR=1 \r\r\nOK\r\n";
char R_GPSRST[]="AT+CGPSRST=1 \r\r\nOK\r\n";
char R_GPSOUT[]="AT+CGPSOUT=34 \r\r\nOK\r\n";
char R_GPSIPR[]="AT+CGPSIPR=9600 \r\r\nOK\r\n";
char R_GPS[]="AT+CGPSINF=2 \r\r\n2,000000,0.000000,N,0.000000,E,0,0,0.000000,0.000000,M,0.000000,M,,0000\r\nOK\r\n";
char R_GPS_2[]="AT+CGPSINF=128 \r\r\n128,180255.000,19,09,2016,00,00\r\nOK\r\n";
char R_CGATT[]="AT+CGATT? \r\r\n+CGATT: 1\r\n\r\nOK\r\n";
// ack street = char R_SAK[]="  \006>SAK;";
char R_SAK[]=">SAK;";
char R_CIIR[]="AT+CIICR \r\r\nOK\r\n";
char R_CIFSR[]="AT+CIFSR \r\r\n";
char R_CIPSHUT[]="AT+CIPSHUT \r\r\nSHUT OK\r\n";
char R_IP[]="AT+CIPSTART=\"UDP\",\"131.255.4.29\",\"6097\" \r\r\nOK\r\n\r\nCONNECT OK\r\n";
char R_CIPSEND[]="AT+CIPSEND \r\r\n";
//char SINRED[]="SIN RED : ERROR \r";

/*==================[Variables Task Gps]===============================*/
enum ESTADOS_GPS
			{
				 ENCIENDO_GPS=0,FIN_GPS,ultimo_estado_gps
			};
int estado_gps=ENCIENDO_GPS;
int i_gps=0;
int time_event_position=0;
/*==================[external data definition]===============================*/


/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/
/** \brief Main function
 *
 * This is the main entry point of the software.
 *
 * \returns 0
 *
 * \remarks This function never returns. Return value is only to avoid compiler
 *          warnings or errors.
 */
int main(void)
{
   /* Starts the operating system in the Application Mode 1 */
   /* This example has only one Application Mode */
   StartOS(AppMode1);

   /* StartOs shall never returns, but to avoid compiler warnings or errors
    * 0 is returned */
   return 0;
}
/** \brief Error Hook function
 *
 * This fucntion is called from the os if an os interface (API) returns an
 * error. Is for debugging proposes. If called this function triggers a
 * ShutdownOs which ends in a while(1).
 *
 * The values:
 *    OSErrorGetServiceId
 *    OSErrorGetParam1
 *    OSErrorGetParam2
 *    OSErrorGetParam3
 *    OSErrorGetRet
 *
 * will provide you the interface, the input parameters and the returned value.
 * For more details see the OSEK specification:
 * http://portal.osek-vdx.org/files/pdf/specs/os223.pdf
 *
 */
void ErrorHook(void)
{
   ciaaPOSIX_printf("ErrorHook was called\n");
   ciaaPOSIX_printf("Service: %d, P1: %d, P2: %d, P3: %d, RET: %d\n", OSErrorGetServiceId(), OSErrorGetParam1(), OSErrorGetParam2(), OSErrorGetParam3(), OSErrorGetRet());
   ShutdownOS(0);
}

/** \brief Initial task
 *
 * This task is started automatically in the application mode 1.
 */
TASK(InitTask)
{
   /* init CIAA kernel and devices */
   ciaak_start();

   ciaaPOSIX_printf("Init Task...\n");
   /* open CIAA digital inputs */
   fd_in = ciaaPOSIX_open("/dev/dio/in/0", ciaaPOSIX_O_RDONLY);

   /* open CIAA digital outputs */
   fd_out = ciaaPOSIX_open("/dev/dio/out/0", ciaaPOSIX_O_RDWR);

   /* open UART connected to gps(gpio1 y gpio2) connector */
  // Chip_SCU_PinMux(6, 4, MD_PDN, FUNC2);              /* P6_4: UART0_TXD */
  // Chip_SCU_PinMux(6, 5, MD_PLN|MD_EZI|MD_ZI, FUNC2); /* P6_5: UART0_RXD */

   fd_uart_gps = ciaaPOSIX_open("/dev/serial/uart/0", ciaaPOSIX_O_RDWR);

   /* open UART connected to USB bridge (FT2232) */
   fd_uart_usb = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

   /* open UART connected to RS232 connector */
   fd_uart_gsm = ciaaPOSIX_open("/dev/serial/uart/2", ciaaPOSIX_O_RDWR);

   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fd_uart_usb, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change baud rate for RS232 */
   ciaaPOSIX_ioctl(fd_uart_gsm, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_9600);

   /* change baud rate for GPS */
   ciaaPOSIX_ioctl(fd_uart_gps, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_9600);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fd_uart_gsm, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* change FIFO TRIGGER LEVEL for GPS */
   ciaaPOSIX_ioctl(fd_uart_gps, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* change FIFO TRIGGER LEVEL for RS232 */
   ciaaPOSIX_ioctl(fd_uart_usb, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* open CIAA ADC */
//   fd_adc_2 = ciaaPOSIX_open("/dev/serial/aio/in/0", ciaaPOSIX_O_RDONLY);
   fd_adc_0 = ciaaPOSIX_open("/dev/serial/aio/in/0", ciaaPOSIX_O_RDONLY);

   fd_adc_1 = ciaaPOSIX_open("/dev/serial/aio/in/1", ciaaPOSIX_O_RDONLY);

   /* open CIAA CH3 */
   ciaaPOSIX_ioctl(fd_adc_0, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);
   ciaaPOSIX_ioctl(fd_adc_0, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_3);

   /* open CIAA CH1 */
   ciaaPOSIX_ioctl(fd_adc_1, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);
   ciaaPOSIX_ioctl(fd_adc_1, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_1);

   /* activate example tasks */
   Periodic_Task_Counter = 0;
   SetRelAlarm(ActivateDigitalInTask, 200, 500); 	// Cada 500 ms
   SetRelAlarm(ActivateLedsTask, 100, 250);  		// Cada 250 ms
   SetRelAlarm(ActivateGpsTask,15000, 0);  			// Se activa 1 sola ves a los 15 segundos
   SetRelAlarm(ActivateEventTask,14600, 10000);  	// Cada 10 s
   /*Contadores a cero*/
   Contador_In1=0;
   Contador_In2=0;
   Contador_In3=0;
   Contador_In4=0;
   pos_data.event = 8;
   pos_data.log = 9997;
   pos_data.dia = 10;
   pos_data.mes = 10;
   pos_data.anio = 1970;
   pos_data.hora= 101010;
   pos_data.fecha=101070;
   ciaaPOSIX_write(fd_uart_usb, "Inicio programa \n\r",18 );
   /* Activates the SerialEchoTask task */
   ActivateTask(SerialGsmTask);
   ActivateTask(SerialGpsTask);
   //ActivateTask(GsmTask);

   //TEST FUNCIONES//
   //Aqui chequeamos el armado de los paquetes:
   Guardo_datos_posicion(&pos_data,&statusgps);		 	// Parseo datos de posicion gps y los guardo pos_data
   put(pos_data,&cola,&cabeza,&items);				 	// Guardo pos_data en cola de envio
   get(&send_data,&cola,&cabeza,&items); 			 	// Saco de la cola y actuliso send_data
   genero_paquete_RUS07(send_data,paquete_1,paquete_2);	// Imprimo send_data con el formato del paquete a enviar
   ciaaPOSIX_memset(paquete_2, 0, sizeof(paquete_2));  	// Limpio cadena respuesta_gps
   ciaaPOSIX_memset(paquete_1, 0, sizeof(paquete_1));  	// Limpio cadena respuesta_gps
   ciaaPOSIX_memset(paquete_3, 0, sizeof(paquete_3));  	// Limpio cadena respuesta_gps
   genero_paquete_GP(send_data,paquete_3);				// Imprimo send_data con el formato del paquete a enviar
   /* end InitTask */
   TerminateTask();
}

/** \brief Serial Task
 *
 * Esta tarea espera por datos que ingresen por fd_uart_gsm, detecta si es una respuesta de comando y envia el evento correspondiente.
 *
 */
TASK(SerialGpsTask)
{
   int8_t buf[5];   /* buffer for uart operation              */
   int32_t ret;      /* return value variable for posix calls */
   int8_t buf_aux[1];
   int i=0;
   int tipo_comando=0;
   int fin_cadena=0;
   ciaaPOSIX_memset(respuesta_gps, 0, sizeof(respuesta_gps));  		// Limpio cadena respuesta_gps
   ciaaPOSIX_memset(buf, 0, sizeof(buf)); 							// Limpio el buffer

   char *pch;
   enum ESTADOS_SERIAL
   			{
   				 ESPERA=0,RESPUESTA,ultimo_estado_gsm
   			};
   int estado_serial=ESPERA;
   while(1)
   {
      /* wait for any character ... */
      ret = ciaaPOSIX_read(fd_uart_gps, buf, 5);

       if(ret > 0)
      {
         /* also write them to the other device */
#ifdef Test_SerialGpsTask
    	  ciaaPOSIX_write(fd_uart_usb, buf, ret);   ///////////////////////para debug comandos
#endif
    	  for (i=0;i<ret;i++)
		  {
    		  switch ( estado_serial )
			  {
    		  	  case ESPERA:
    		  	  {
    		  		  if (buf[i] == '$')
    		  		  {
    		  			  estado_serial=RESPUESTA;   // Si llega $ es una paquete de datos
    		  			  ciaaPOSIX_memset(respuesta_gps, 0, sizeof(respuesta_gps));  		// Limpio cadena respuesta_gps
    		  		  }
    		  		  break;
    		  	  }
    		  	  case RESPUESTA:
				  {
					  buf_aux[0]=buf[i];
					  ciaaPOSIX_strcat(respuesta_gps, buf_aux);				// Copio buffer en cadena de respuesta
					  if (buf[i] == '*')
					  fin_cadena++;											// Busco caracteres que indican fin parcial de respuesta
					  if (buf[i] == ',')
				      ciaaPOSIX_strcat(respuesta_gps, "0");					// Si es "," inserto un 0 despues de cada coma.
					  if (fin_cadena > 0)									// Si hay fin parcial de respuesta
					  {
						  Guardo_datos_posicion(&pos_data,&statusgps);		//Parseo datos de la respuesta y los guardo
						  //ciaaPOSIX_memset(respuesta_gps, 0, sizeof(respuesta_gps));  // Limpio cadena respuesta
						  #ifdef Test_SerialGpsTask
						  ciaaPOSIX_write(fd_uart_usb, "Llego posicion!! \n\r",18 );
						  char str[6];
						  itoa(pos_data.fecha,str,6);
						  ciaaPOSIX_write(fd_uart_usb, str,6);
						  #endif
						  fin_cadena=0;
						  estado_serial=ESPERA;
					  }
					  break;
				  }
			  }
		  }
      }
   }
}

/** \brief Serial Task
 *
 * Esta tarea espera por datos que ingresen por fd_uart_gsm, detecta si es una respuesta de comando y envia el evento correspondiente.
 *
 */
TASK(SerialGsmTask)
{
   int8_t buf[5];   /* buffer for uart operation              */
   int8_t buf_aux[1];
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */
   int i=0;
   ciaaPOSIX_memset(respuesta, 0, sizeof(respuesta));  		// Limpio cadena respuesta
   ciaaPOSIX_memset(buf, 0, sizeof(buf)); 					// Limpio el buffer
   int estadorespuesta=0;
   int tipo_comando=0;
   int x,fin_cadena,respuesta_ok;
   int fin_comando=0;
   int respuesta_recibida=0;
   int inicio_respuesta=0;
   int inicio_comando=0;
   char *pch;
   enum ESTADOS_SERIAL
   			{
   				 ESPERA=0,ESPERA_R,ESPERA_C,RESPUESTA,COMANDO,ultimo_estado_gsm
   			};
   int estado_serial=ESPERA;
   while(1)
   {
      /* wait for any character ... */
      ret = ciaaPOSIX_read(fd_uart_gsm, buf, 5);

      if(ret > 0)
      {
         /* also write them to the other device */
#ifdef Test_SerialGsmTask
    	  ciaaPOSIX_write(fd_uart_usb, buf, ret);   ///////////////////////para debug comandos
#endif
    	 //ciaaPOSIX_strcat(respuesta, buf);			// Copio buffer en cadena de respuesta
         for (i=0;i<ret;i++)
         {
			 switch ( estado_serial )
			 {
				 case ESPERA:
				 {
					 if (buf[i] == 'A') estado_serial=ESPERA_R;   // Si llega A es una posible respuesta AT
					 if (buf[i] == '>') estado_serial=ESPERA_C;   // Si llega > es un posible comando
					 break;
				 }
				 case ESPERA_R:
				 {
					 if (buf[i] == 'T')
					 {
						 estado_serial=RESPUESTA;   //Es una respuesta
						 respuesta[0]='A';
						 respuesta[1]='T';
					 }else
					 {
						estado_serial=ESPERA;      //Sino vuelvo a espera
					 }
					 break;
				 }
				 case ESPERA_C:
				 {
					 if (buf[i] == 'S')
					 {
						 estado_serial=COMANDO; 		//Es un comando
						 respuesta[0]='>';
						 respuesta[1]='S';
					 }else
					 {
						estado_serial=ESPERA;      //Sino vuelvo a espera
					 }
					 break;
				 }
				case RESPUESTA:
				{
					buf_aux[0]=buf[i];
					ciaaPOSIX_strcat(respuesta, buf_aux);				// Copio buffer en cadena de respuesta
					if (buf[i] == '\n')
						fin_cadena++;									// Busco caracteres que indican fin parcial de respuesta
					if ((fin_cadena > 0) && (tipo_comando==0))			// Si hay fin parcial de respuesta
					{
						if (ciaaPOSIX_strncmp(respuesta,CIPSEND,8)==0)	tipo_comando=1;
						if (ciaaPOSIX_strncmp(respuesta,GPSOUT,8)==0)	tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,GPSIPR,8)==0)	tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,GPSRST,8)==0)	tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,GPSPWR,8)==0)	tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,APN,8)==0)		tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,CIIR,8)==0)		tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,CIFSR,8)==0)	tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,CIPSHUT,10)==0)	tipo_comando=2;
						if (ciaaPOSIX_strncmp(respuesta,IP,10)==0)		tipo_comando=4;
						if (ciaaPOSIX_strncmp(respuesta,CGATT,8)==0)	tipo_comando=4;

						if (tipo_comando==0)
						{
							ciaaPOSIX_memset(respuesta, 0, sizeof(respuesta));  // Limpio cadena respuesta y descarto lo recibido
							fin_cadena=0;
							estado_serial=ESPERA;      							//Vuelvo a espera
						}
					}
					if ((tipo_comando==1) && (fin_cadena==1))   // Si es un comando del tipo 1, espero 1 /n para detectar fin de respuesta
					{
						respuesta_recibida=1;
						if (ciaaPOSIX_strncmp(respuesta,R_CIPSEND,sizeof(R_CIPSEND))==0)		respuesta_ok=1;
					}
					if ((tipo_comando==2) && (fin_cadena==2))   // Si es un comando del tipo 2, espero 2 /n para detectar fin de respuesta
					{
						respuesta_recibida=1;
						if (ciaaPOSIX_strncmp(respuesta,R_APN,sizeof(R_APN))==0)				respuesta_ok=1;
						if (ciaaPOSIX_strncmp(respuesta,R_CIIR,sizeof(R_CIIR))==0)				respuesta_ok=1;
						if (ciaaPOSIX_strncmp(respuesta,R_CIFSR,8)==0)							respuesta_ok=1;
						if (ciaaPOSIX_strncmp(respuesta,R_CIPSHUT,sizeof(R_CIPSHUT))==0)		respuesta_ok=1;
						if (ciaaPOSIX_strncmp(respuesta,R_GPSPWR,sizeof(R_GPSPWR))==0)			respuesta_ok=2;
						if (ciaaPOSIX_strncmp(respuesta,R_GPSOUT,sizeof(R_GPSOUT))==0)			respuesta_ok=2;
						if (ciaaPOSIX_strncmp(respuesta,R_GPSIPR,sizeof(R_GPSIPR))==0)			respuesta_ok=2;
						if (ciaaPOSIX_strncmp(respuesta,R_GPSRST,sizeof(R_GPSRST))==0)			respuesta_ok=2;
					}
					if ((tipo_comando==4) && (fin_cadena==4))   // Si es un comando del tipo 4, espero 4 /n para detectar fin de respuesta
					{
						respuesta_recibida=1;
						if (ciaaPOSIX_strncmp(respuesta,R_CGATT,sizeof(R_CGATT))==0)	respuesta_ok=1;
						if (ciaaPOSIX_strncmp(respuesta,R_IP,sizeof(R_IP))==0)			respuesta_ok=1;
					}

					if (respuesta_recibida==1)   // Si encuentro respuesta, evio evento si es correcta o hay error
					{
						estado_serial=ESPERA;      							//Vuelvo a esperar
						if (respuesta_ok==1)
						{
							SetEvent(GsmTask, EVENTOK);
						}
						else
						{
							if (respuesta_ok==2)
							{
								SetEvent(GpsTask, EVENTOKGPS);
							}else
							{
								SetEvent(GsmTask, EVENTERROR);
							}
						}
						respuesta_ok=0;
						respuesta_recibida=0;
						ciaaPOSIX_memset(respuesta, 0, sizeof(respuesta));  // Limpio cadena respuesta y descarto lo recibido
						fin_cadena=0;
						tipo_comando=0;
					}
					break;
				}
				case COMANDO:
				{
					buf_aux[0]=buf[i];
					ciaaPOSIX_strcat(respuesta, buf_aux);	// Copio buffer en cadena de respuesta
					if (buf[i] == '<')
					{
						estado_serial=ESPERA;      							//Vuelvo a espera
						if (ciaaPOSIX_strncmp(respuesta,R_SAK,4)==0)		// Si es ACK enio evento correpondiente
						{
							SetEvent(GsmTask, EVENTACK);
						}
						ciaaPOSIX_memset(respuesta, 0, sizeof(respuesta));  // Limpio cadena respuesta y descarto lo recibido
					}
				break;
				}
			 }
         }
      }
   }
}

/** \brief Periodic Task
 *
 * This task is activated by the Alarm ActivatePeriodicTask.
 * This task copies the status of the inputs bits 0..3 to the output bits 0..3.
 * This task also blinks the output 4
 */
TASK(DigitalInTask)
{
   /*
    * Example:
    *    Read inputs 0..3
    */
	char message3[] = "Tarea Digital in\n";
//   ciaaPOSIX_write(fd_uart_usb, message3, ciaaPOSIX_strlen(message3));

   /* variables to store input/output status */
	uint8_t inputs = 0, outputs = 0;

   /* read inputs */
    ciaaPOSIX_read(fd_in, &inputs, 1);
    if ((inputs&0x01) != estado_in1)
    {
	    cambioestado=1;
	    pos_data.event = CAMBIO_IN1;				// evento cambio in1 (201)
	    if ((inputs&0x01) == 1)
	    {
		    estado_in1=1;
		    valor_in[0]='1';
		    pos_data.IN1 = 1;

	    }else
	    {
		    estado_in1=0;
		    valor_in[0]='0';
		    pos_data.IN1 = 0;
	    }
	    if (pos_data.log >= 9999) pos_data.log = 0;
	    pos_data.log += 1;
	    put(pos_data,&cola,&cabeza,&items); 		// guardo evento en cola de envio
    }
    if ((inputs&0x02) != estado_in2)
    {
    	cambioestado=1;
	    pos_data.event = CAMBIO_IN2;				// evento cambio in2
	    if ((inputs&0x02) == 2)
   	    {
   		    estado_in2=2;
   		    valor_in[1]='1';
   		    pos_data.IN2 = 1;
   	    }else
   	    {
   		    estado_in2=0;
   		    valor_in[1]='0';
   		    pos_data.IN2 = 0;
   	    }
	    if (pos_data.log >= 9999) pos_data.log = 0;
	    pos_data.log += 1;
	   	put(pos_data,&cola,&cabeza,&items); 		// guardo evento en cola de envio
    }
    if ((inputs&0x04) != estado_in3)
    {
	    cambioestado=1;
	    pos_data.event = CAMBIO_IN3;				// evento cambio in3
   	    if ((inputs&0x04) == 4)
   	    {
   		    estado_in3=4;
   		    valor_in[2]='1';
   		    pos_data.IN3 = 1;
   	    }else
   	    {
   		    estado_in3=0;
   		    valor_in[2]='0';
   		    pos_data.IN3 = 0;
   	    }
   	    if (pos_data.log >= 9999) pos_data.log = 0;
   	 	pos_data.log += 1;
   	 	put(pos_data,&cola,&cabeza,&items); 		// guardo evento en cola de envio
    }
	if ((inputs&0x08) != estado_in4)
	{
	   cambioestado=1;
	   pos_data.event = CAMBIO_IN4;					// evento cambio in4
	   if ((inputs&0x08) == 8)
	   {
		   estado_in4=8;
		   valor_in[3]='1';
		   pos_data.IN4 = 1;
	   }else
	   {
		   estado_in4=0;
		   valor_in[3]='0';
		   pos_data.IN4 = 0;
	   }
	   if (pos_data.log >= 9999) pos_data.log = 0;
	   pos_data.log += 1;
	   put(pos_data,&cola,&cabeza,&items); 			// guardo evento en cola de envio
	}

	if (cambioestado == 1)
	{
		blinkled();
		/* Genero evento de cambio de estado*/
		cambioestado=0;
	/* TEST_1: Visualizo por UART el estado de las salidas*/
	#ifdef Test_DigitalInTask
	   char messageinestado[] = "\r Estado de las entradas: ";
	   ciaaPOSIX_write(fd_uart_usb, messageinestado, ciaaPOSIX_strlen(messageinestado));
	   ciaaPOSIX_write(fd_uart_usb, valor_in, 4);
	#endif
	}
	/* Activates the SerialEchoTask task */
	ActivateTask(AnalogInTask);
	/* end PeriodicTask */
	TerminateTask();
	}
TASK(AnalogInTask)
{
   /*
    * Example:
    */
   int32_t count;
   uint16_t hr_ciaaDac,adc_1,adc_2,adc_3;

   /* Read ADC1. */

 //  ciaaPOSIX_read(fd_adc, &adc_1, sizeof(adc_1));

   /* Read CH3 , ADC0. */

   ciaaPOSIX_read(fd_adc_0, &adc_3, sizeof(adc_3));

   /* Read CH1 , ADC1. */

   ciaaPOSIX_read(fd_adc_1, &adc_1, sizeof(adc_1));

//	 ciaaPOSIX_write(fd_uart_usb, hr_ciaaDac, ciaaPOSIX_strlen(hr_ciaaDac));
//   ciaaPOSIX_ioctl(fd_adc_0, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_2);
//   ciaaPOSIX_read(fd_adc_0, &adc_2, sizeof(adc_2));

   /* Si cambia el valor actual +-40% genero evento*/

   if(adc_1 > (1.4*valor_ch1) || adc_1 < (0.6*valor_ch1))
   {
	   pos_data.event = CAMBIO_ADC1;				// evento cambio adc1
	   estado_ch1=1;
   }
   if(adc_3 > (1.4*valor_ch3) || adc_3 < (0.6*valor_ch3))
   {
	   pos_data.event = CAMBIO_ADC2;				// evento cambio adc2
	   estado_ch3=1;
   }

   valor_ch1=adc_1;    // Guardo valor actual
   valor_ch3=adc_3;
   pos_data.ADC1 = adc_1;
   pos_data.ADC2 = adc_3;

/* TEST_3: Visualizo por UART el estado de las salidas*/

#ifdef Test_AnalogInTask
   char messageinestado[] = "\r Estado de las entradas analogicas:  ";
   ciaaPOSIX_write(fd_uart_usb, messageinestado, ciaaPOSIX_strlen(messageinestado));
   char str[4];
   itoa(valor_ch1,str,10);
   ciaaPOSIX_write(fd_uart_usb, str,ciaaPOSIX_strlen(str));
   char messageinestado2[] = "    ";
   ciaaPOSIX_write(fd_uart_usb, messageinestado2, ciaaPOSIX_strlen(messageinestado2));
   itoa(valor_ch3,str,10);
   ciaaPOSIX_write(fd_uart_usb, str,ciaaPOSIX_strlen(str));
   if ((estado_ch1|estado_ch3) == 1)
   {
	   char messageinestado1[] = "\r --Evento por cambio de estado!!!  ";
	   ciaaPOSIX_write(fd_uart_usb, messageinestado1, ciaaPOSIX_strlen(messageinestado1));
   }
#endif

   /*Reset banderas de cambio de estado*/
   if ((estado_ch1|estado_ch3) == 1)
   {
	   if (pos_data.log >= 9999) pos_data.log = 0;
	   pos_data.log += 1;
	   put(pos_data,&cola,&cabeza,&items); //guardo evento en cola de envio
	   estado_ch1=0;
  	   estado_ch3=0;
   }

   /* Activates the ModBustask */
   ActivateTask(ModBusTask);

   /* end AnalogInTask */
   TerminateTask();
}

TASK(ModBusTask)
{
   /*
    * Example:
    *    Read inputs 0..3, update outputs 0..3.
    *    Blink output 4
    */
   char message5[] = "Tarea Mod bus \r";
 //  ciaaPOSIX_write(fd_uart_usb, message5, ciaaPOSIX_strlen(message5));
   /* variables to store input/output status */
   ciaaPOSIX_printf("ModBusTask");

   /* end ModBusTask */
   TerminateTask();
}

TASK(LedsTask)
{
   /*
    *    Blink Led rojo si statusgps=0, On Led Rojo si statusgps=1
    *    Blink Led verde si statusgsm=0, On Led Verde si statusgsm=1
    */
   uint8_t outputs;

   /* TEST_2: Titilo ambos leds 10 veces y luego los dejo fijos*/
#ifdef Test_LedsTask

   if (Periodic_Task_Counter == 20)
   {
	   statusgsm=1;
	   statusgps=1;
   }

#endif

   ciaaPOSIX_read(fd_out, &outputs, 1);

   if (statusgsm == 0)
   {
	   outputs ^= 0x20;     //Blink Ledverde
   }else
   {
	   outputs |= 0x20;     //On Ledverde
   }
   if (statusgps == 0)
   {
	   outputs ^= 0x08;     //Blink Ledrojo
   }else
   {
	   outputs |= 0x08;     //On Ledrojo
   }

   ciaaPOSIX_write(fd_out, &outputs, 1);
   Periodic_Task_Counter++;
   /* end LedsTask */
   TerminateTask();
}

TASK(GsmTask)
{
   /*
    * Example:
    *
    */
	/* TEST_2: Titilo ambos leds 10 veces y luego los dejo fijos*/
#ifdef Test_GsmTask
//   char message5[] = "Tarea LED  \r";

#endif

	char respuesta[30];
	int32_t res;      		/* return value variable for posix calls  */
	int8_t dato_gsm[20];
	EventMaskType Events;
	int8_t buf[20];   		/* buffer for uart operation              */
	uint8_t outputs;  		/* to store outputs status                */
	int32_t ret;      		/* return value variable for posix calls  */
	x=1;
	while (1){
	/* Estados gsm */
	if( estado_gsm > ultimo_estado_gsm )
		{
			FSM_inicializada = 0;
		}
		if( !FSM_inicializada )
		{
			FSM_inicializada = 1;
			estado_gsm = RED;
		}
		switch ( estado_gsm )
		{
			case RED:                                          				// Consulto si hay conexion a la red gsm
			{
				ciaaPOSIX_write(fd_uart_gsm, CGATT, ciaaPOSIX_strlen(CGATT));  //Consulto si tiene señal gprs
				SetRelAlarm(SetEventTimeOut, 2000, 0);						//Activo time out 2 segundos
				WaitEvent(EVENTOK | EVENTERROR | EVENTTIMEOUT);				//Espero respuesta
				GetEvent(GsmTask, &Events);
				ClearEvent(Events);
				if (Events & EVENTOK)
				{
					CancelAlarm(SetEventTimeOut);
					estado_gsm = SET;								//Si tiene red seteo parametros,Sino vuelvo a consultar
					statusgsm=1;
				}
				if (Events & EVENTERROR)
				{
					CancelAlarm(SetEventTimeOut);
					SetRelAlarm(SetEventTimeOut, 1000, 0);			//Activo time out 2 segundos
					WaitEvent(EVENTTIMEOUT);
					GetEvent(GsmTask, &Events);
					ClearEvent(Events);
					statusgsm=0;
				}
				break;
			}

			case SET:
			{
				if(x==1) ciaaPOSIX_write(fd_uart_gsm, CIPSHUT, ciaaPOSIX_strlen(CIPSHUT));
				if(x==2) ciaaPOSIX_write(fd_uart_gsm, APN, ciaaPOSIX_strlen(APN)); 			 //Consulto si tiene señal gprs
				if(x==3) ciaaPOSIX_write(fd_uart_gsm, CIIR, ciaaPOSIX_strlen(CIIR));  		 //Consulto si tiene señal gprs "AT+CIICR";
				if(x==4) ciaaPOSIX_write(fd_uart_gsm, CIFSR, ciaaPOSIX_strlen(CIFSR));  	 //Consulto si tiene señal gprs "AT+CIFSR";
				if(x==5) ciaaPOSIX_write(fd_uart_gsm, CIPSHUT, ciaaPOSIX_strlen(CIPSHUT));
				if(x==6) ciaaPOSIX_write(fd_uart_gsm, IP, ciaaPOSIX_strlen(IP)); 		 	 //Configuro ip y puerto ;
				if(x<7)
				{
					SetRelAlarm(SetEventTimeOut, 10000, 0);				//Activo time out 2 segundos
					WaitEvent(EVENTOK | EVENTERROR | EVENTTIMEOUT);		//Espero respuesta
					GetEvent(GsmTask, &Events);
					ClearEvent(Events);
					if (Events & EVENTOK)
					{
						CancelAlarm(SetEventTimeOut);
						x++;											// Si respuesta es correcta envio siguiente comando
					}
					if (Events & EVENTERROR)
					{
						CancelAlarm(SetEventTimeOut);
						SetRelAlarm(SetEventTimeOut, 1000, 0);			//Activo time out 2 segundos
						WaitEvent(EVENTTIMEOUT);
						GetEvent(GsmTask, &Events);
						ClearEvent(Events);
						estado_gsm = RED;								// Si hay error comienzo nuevamente el ciclo
						if (x >= 5)
							x=5;
						else
							x=1;
					}
				}
				if(x>=7)
				{
					estado_gsm = SEND;
					x=8;
				}
				SetRelAlarm(SetEventTimeOut, 2000, 0);				//Activo time out 2000 mseg para envio entre comandos
				WaitEvent(EVENTTIMEOUT);							//Espero respuesta
				GetEvent(GsmTask, &Events);
				ClearEvent(Events);
				break;
			}

			case SEND:
			{
				switch ( estado_send )
				{
					case SEND_ESPERA:                       // Consulto si pendientes en la cola de envio
					{
						if(items > 0)
						{
							estado_send=SEND1;
							ciaaPOSIX_memset(paquete_1, 0, sizeof(paquete_1));  		 // Limpio paquete_1 de datos
							ciaaPOSIX_memset(paquete_2, 0, sizeof(paquete_2));
							ciaaPOSIX_memset(paquete_3, 0, sizeof(paquete_3));
							get(&send_data,&cola,&cabeza,&items); 						 // Si no hay paquete_1 pendiente, lo saco de la cola y lo formateo
							genero_paquete_RUS07(send_data,paquete_1,paquete_2);		 // Lo guardo en send_data para enviarlo en formato RUS07
							genero_paquete_GP(send_data,paquete_3);				   		 // y RGP

						}
						break;
					}
					case SEND1:                             // Envio Paquete 1
					{
						////////// Preparo modulo gsm para envio de comando /////////
						ciaaPOSIX_write(fd_uart_gsm, CIPSEND, ciaaPOSIX_strlen(CIPSEND)); 	// Envio comando para enviar datos
						SetRelAlarm(SetEventTimeOut, 3000, 0);								// Activo time out 3 segundos
						WaitEvent(EVENTOK | EVENTERROR | EVENTTIMEOUT);						// Espero respuesta
						GetEvent(GsmTask, &Events);
						ClearEvent(Events);
						if (Events & EVENTOK)
						{
							CancelAlarm(SetEventTimeOut);
							/////////////Modulo listo para enviar comando /////////////
							ciaaPOSIX_write(fd_uart_gsm, paquete_1, ciaaPOSIX_strlen(paquete_1));
							ciaaPOSIX_write(fd_uart_gsm, paquete_2, ciaaPOSIX_strlen(paquete_2));
							Send_Event = 1;
							estado_send = ACK1;
							estado_gsm = ACKNOLEGE;
						}
						if (Events & EVENTERROR)
						{
							CancelAlarm(SetEventTimeOut);
							SetRelAlarm(SetEventTimeOut, 1000, 0);					//Activo time out 2 segundos
							WaitEvent(EVENTTIMEOUT);
							GetEvent(GsmTask, &Events);
							ClearEvent(Events);
							estado_gsm = RED;										// Si hay error comienzo nuevamente el ciclo
						}
						break;
					}
					case ACK1:
					{
						if (Send_Event == 0)
							estado_send = SEND2;			// Si llega ACK paso al siguiente
						else
							estado_send = SEND1;			// Si no llego ACK vuelvo a enviar paquete
						break;
					}
					case SEND2:                             // Envio Paquete 1
					{
						////////// Preparo modulo gsm para envio de comando /////////
						ciaaPOSIX_write(fd_uart_gsm, CIPSEND, ciaaPOSIX_strlen(CIPSEND)); 	// Envio comando para enviar datos
						SetRelAlarm(SetEventTimeOut, 3000, 0);								// Activo time out 3 segundos
						WaitEvent(EVENTOK | EVENTERROR | EVENTTIMEOUT);						// Espero respuesta
						GetEvent(GsmTask, &Events);
						ClearEvent(Events);
						if (Events & EVENTOK)
						{
							CancelAlarm(SetEventTimeOut);
							SetRelAlarm(SetEventTimeOut, 1000, 0);				//Genero time out 1 segundo
							WaitEvent(EVENTTIMEOUT);
							GetEvent(GsmTask, &Events);
							ClearEvent(Events);
							/////////////Modulo listo para enviar comando /////////////
							ciaaPOSIX_write(fd_uart_gsm, paquete_3, ciaaPOSIX_strlen(paquete_3));
							//ciaaPOSIX_write(fd_uart_gsm, last_position,ciaaPOSIX_strlen(last_position));
							#ifdef Test_GSM
							//ciaaPOSIX_write(fd_uart_gsm, last_position, ciaaPOSIX_strlen(last_position)); // Si respuesta es correcta Envio POSICION
							#endif
							Send_Event = 1;
							estado_send = ACK2;
							estado_gsm = ACKNOLEGE;
						}
						if (Events & EVENTERROR)
						{
							CancelAlarm(SetEventTimeOut);
							SetRelAlarm(SetEventTimeOut, 1000, 0);					//Activo time out 2 segundos
							WaitEvent(EVENTTIMEOUT);
							GetEvent(GsmTask, &Events);
							ClearEvent(Events);
							estado_gsm = RED;										// Si hay error comienzo nuevamente el ciclo
						}
						break;
					}
					case ACK2:                             // Envio Paquete 1
					{
						if (Send_Event == 0)
							estado_send = SEND_ESPERA;		// Si llega ACK paso al siguiente
						else
							estado_send = SEND2;			// Si no llego ACK vuelvo a enviar paquete
						break;
					}

				}
				SetRelAlarm(SetEventTimeOut, 100, 0);				//Genero time out 1 segundo
				WaitEvent(EVENTTIMEOUT);
				GetEvent(GsmTask, &Events);
				ClearEvent(Events);
				break;
			}

			case ACKNOLEGE:
			{
				SetRelAlarm(SetEventTimeOut, 15000, 0);						// Activo time out 15 segundos
				WaitEvent(EVENTACK | EVENTTIMEOUT);							// Espero ACK O TIMEOUT
				GetEvent(GsmTask, &Events);
				ClearEvent(Events);
				if (Events & EVENTACK)
				{
					CancelAlarm(SetEventTimeOut);
					//////ciaaPOSIX_write(fd_uart_usb, last_position, ciaaPOSIX_strlen(last_position)); // Si respuesta es correcta Envio POSICION
					ciaaPOSIX_write(fd_uart_gsm, finrespuesta, ciaaPOSIX_strlen(finrespuesta)); 	// Caracteres de fin de respuesta, para que se borre buffer de respuesta en tarea serial task
					Send_Event = 0;
					estado_gsm = SEND;
#ifdef Test_GSM
	ciaaPOSIX_write(fd_uart_usb, "llego ack \n\r",13 );
#endif
				}
				if (Events & EVENTTIMEOUT)
				{
					estado_gsm = RED;
					x=5;
				}
				break;
			}
		}
	//	char str[6];
	//	itoa(pos_data.hora,str,10);
	//	ciaaPOSIX_write(fd_uart_usb, str,ciaaPOSIX_strlen(str));
		blinkled();
   	}
}

TASK(GpsTask)
{
   /*
    *    Consulta datos de posicion, fecha y hora.
    */

	EventMaskType Events;
	int x=1;
	while (1){
		switch ( estado_gps )
		 {
			case ENCIENDO_GPS:
			{
				if(x==1) ciaaPOSIX_write(fd_uart_gsm, GPSIPR, ciaaPOSIX_strlen(GPSIPR)); 		 // CONFIGURO SALIDA A 9600
				if(x==2) ciaaPOSIX_write(fd_uart_gsm, GPSOUT, ciaaPOSIX_strlen(GPSOUT));  		 // ACTIVO PAQUETE GPGGA;
				if(x==3) ciaaPOSIX_write(fd_uart_gsm, GPSPWR, ciaaPOSIX_strlen(GPSPWR));	  	 // ENCIENDO MODULO GPS;
				if(x==4) ciaaPOSIX_write(fd_uart_gsm, GPSRST, ciaaPOSIX_strlen(GPSRST));		 // RESET MODULO GPS
				if(x<5)
				{
					SetRelAlarm(SetEventTimeOutgps, 1000, 0);				//Activo time out 2 segundos
					WaitEvent(EVENTOKGPS | EVENTTIMEOUTGPS);					//Espero respuesta
					GetEvent(GpsTask, &Events);
					ClearEvent(Events);
					if (Events & EVENTOKGPS)
					{
						CancelAlarm(SetEventTimeOutgps);
						x++;											// Si respuesta es correcta envio siguiente comando
					}
				}
				if(x>=5)
				{
					estado_gps = FIN_GPS;
					x=0;
				}
				SetRelAlarm(SetEventTimeOutgps, 500, 0);				//Activo time out 2000 mseg para envio entre comandos
				WaitEvent(EVENTTIMEOUTGPS);							//Espero respuesta
				GetEvent(GpsTask, &Events);
				ClearEvent(Events);
				break;
			}
			case FIN_GPS:
			{
				estado_gps=ENCIENDO_GPS;
				ActivateTask(GsmTask);
				TerminateTask();				//TERMINO TAREA GPS
				break;
			}
		 }
	}
}

TASK(EventTask)
{
   /*
    * Example:
    *    Read inputs 0..3, update outputs 0..3.
    *    Blink output 4
    */
	time_event_position++;							//Cada 1 min genero evento y lo pongo en la cola de envios
	if (time_event_position==TIME_POSITION)
	{
		if (pos_data.log >= 9999) pos_data.log = 0;
		pos_data.log += 1;
		pos_data.event = CAMBIO_POS;				// evento posicion ciaa
		put(pos_data,&cola,&cabeza,&items); //guardo evento en cola de envio
		time_event_position=0;
	}
	TerminateTask();
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

