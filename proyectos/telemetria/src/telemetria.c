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
//#define Test_AnalogInTask
//#define Test_DigitalInTask
//#define Test_LedTask


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

/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart0;
/** \brief File descriptor of the USB uart
 *
 * Device path /dev/serial/uart/1
 */
static int32_t fd_uart1;
/** \brief File descriptor of the RS232 uart
 *
 * Device path /dev/serial/uart/2
 */
static int32_t fd_uart2;

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

/*==================[Variables Task GSM]===============================*/
enum ESTADOS_GSM
			{
				 RED=0,R_RED,SET,R_SET,SEND,ACKNOLEGE,ERROR,DELAY,ultimo_estado_gsm
			};
int estado_gsm;
int FSM_inicializada=0,i=0,h=0,x=0,delay=0;

int8_t respuesta[100];

char APN[50]="AT+CSTT=\"internet.ctimovil.com.ar\",\"gprs\",\"gprs\" \r";
char IP[50]="AT+CIPSTART=\"UDP\",\"190.12.119.147\",\"6097\" \r";
char last_position [50]=">RUS04,111222000121;ID=1234567<";
char CIIR[]="AT+CIICR \r";
char CIFSR[]="AT+CIFSR \r";
char CGATT[]="AT+CGATT? \r";
char SINRED[]="SIN RED : ERROR \r";
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

   /* open UART connected to RS232 connector */
   fd_uart0 = ciaaPOSIX_open("/dev/serial/uart/0", ciaaPOSIX_O_RDWR);

   /* open UART connected to USB bridge (FT2232) */
   fd_uart1 = ciaaPOSIX_open("/dev/serial/uart/1", ciaaPOSIX_O_RDWR);

   /* open UART connected to RS232 connector */
   fd_uart2 = ciaaPOSIX_open("/dev/serial/uart/2", ciaaPOSIX_O_RDWR);

   /* change baud rate for uart usb */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change baud rate for RS232 */
   /* NEW */
   ciaaPOSIX_ioctl(fd_uart2, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_9600);

   /* change baud rate for RS485 */
   /* NEW */
   ciaaPOSIX_ioctl(fd_uart0, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

   /* change FIFO TRIGGER LEVEL for uart usb */
   ciaaPOSIX_ioctl(fd_uart2, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* change FIFO TRIGGER LEVEL for RS485 */
   ciaaPOSIX_ioctl(fd_uart0, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

   /* change FIFO TRIGGER LEVEL for RS232 */
   ciaaPOSIX_ioctl(fd_uart1, ciaaPOSIX_IOCTL_SET_FIFO_TRIGGER_LEVEL, (void *)ciaaFIFO_TRIGGER_LEVEL3);

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
   SetRelAlarm(ActivateDigitalInTask, 200, 500);  //Cada 500 ms
   SetRelAlarm(ActivateLedsTask, 100, 300);  //Cada 300 ms
 //  SetRelAlarm(ActivateGsmTask, 150, 1000);  //Cada 1 s
   /*Contadores a cero*/
   Contador_In1=0;
   Contador_In2=0;
   Contador_In3=0;
   Contador_In4=0;

   /* Activates the SerialEchoTask task */
//   ActivateTask(SerialEchoTask);
   ActivateTask(SerialTask);
   ActivateTask(GsmTask);
   /* end InitTask */
   TerminateTask();
}

/** \brief Serial Echo Task
 *
 * This tasks waits for input data from fd_uart1 and writes the received data
 * to fd_uart1 and fd_uart2. This taks alos blinkgs the output 5.
 *
 */
TASK(SerialTask)
{
   int8_t buf[20];   /* buffer for uart operation              */
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */
   int i=0;
   ciaaPOSIX_memset(respuesta, 0, sizeof(respuesta));  		// Limpio cadena respuesta
   ciaaPOSIX_memset(buf, 0, sizeof(buf)); 					// Limpio el buffer
   int estadorespuesta=0;
   enum ESTADOS_RESPUESTA{ECOCOMANDO=0,R1,R2,R3,PARSEORESPUESTA};
   int estado_respuesta=0;
   int tipo_comando=0;

   while(1)
   {
      /* wait for any character ... */
      ret = ciaaPOSIX_read(fd_uart2, buf, 20);

      if(ret > 0)
      {
         /* also write them to the other device */
         ciaaPOSIX_write(fd_uart1, buf, ret);
         ciaaPOSIX_strcat(respuesta, buf);			//copio bufer en cadena de respuesta
         ciaaPOSIX_memset(buf, 0, sizeof(buf)); 	// Limpio el buffer

         i=ciaaPOSIX_strlen(respuesta);				//Busco fin de respuesta '\r'

         if (respuesta[i-1] == '\n')				// detecto caracter que indica fin parcial de respuesta
         {
        	 switch(estado_respuesta)
        	 {
        	 	 case(ECOCOMANDO):
				 {

        	 		if (ciaaPOSIX_strncmp(respuesta,CGATT,8)==0)	tipo_comando=1;
        	 		if (ciaaPOSIX_strncmp(respuesta,APN,8)==0)		tipo_comando=2;
        	 		if (ciaaPOSIX_strncmp(respuesta,CIIR,8)==0)		tipo_comando=3;
        	 		if (ciaaPOSIX_strncmp(respuesta,CIFSR,8)==0)	tipo_comando=4;
        	 		if (ciaaPOSIX_strncmp(respuesta,IP,8)==0)		tipo_comando=5;

        	 		if (tipo_comando>0)
        	 		{
        	 			estado_respuesta=R1;  //paso al siguiente estado
        	 		}else
        	 		{
        	 			ciaaPOSIX_memset(respuesta, 0, sizeof(respuesta));  // Limpio cadena respuesta y descarto lo recibido
        	 		}
        	 		break;
				 }
        	 	case(R1):
				{
        	 		estado_respuesta=R2;  //paso al siguiente estado
        	 		break;
				}
        	 	case(R2):
				{
					estado_respuesta=R3;  //paso al siguiente estado
					break;
				}
        	 	case(R3):
				{
					estado_respuesta=PARSEORESPUESTA;  //paso al siguiente estado
					break;
				}
        	 	case(PARSEORESPUESTA):
				{
        	 		//if (tipo_comando==1 && ciaaPOSIX_strncmp(respuesta,R_CGATT,8)==0)	SETEVENT(R_OK);



        	 		estado_respuesta=ECOCOMANDO;  //paso al siguiente estado
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
//   ciaaPOSIX_write(fd_uart1, message3, ciaaPOSIX_strlen(message3));

   /* variables to store input/output status */
	uint8_t inputs = 0, outputs = 0;

   /* read inputs */
    ciaaPOSIX_read(fd_in, &inputs, 1);

    if ((inputs&0x01) != estado_in1)
    {
	    cambioestado=1;
	    if ((inputs&0x01) == 1)
	    {
		    estado_in1=1;
		    valor_in[0]='1';
	    }else
	    {
		    estado_in1=0;
		    valor_in[0]='0';
	    }
    }
   if ((inputs&0x02) != estado_in2)
   {
	   cambioestado=1;
	   if ((inputs&0x02) == 2)
   	   {
   		   estado_in2=2;
   		   valor_in[1]='1';
   	   }else
   	   {
   		   estado_in2=0;
   		   valor_in[1]='0';
   	   }
      }
   if ((inputs&0x04) != estado_in3)
   {
	   cambioestado=1;
   	   if ((inputs&0x04) == 4)
   	   {
   		   estado_in3=4;
   		   valor_in[2]='1';
   	   }else
   	   {
   		   estado_in3=0;
   		   valor_in[2]='0';
   	   }
      }
   if ((inputs&0x08) != estado_in4)
   {
	   cambioestado=1;
   	   if ((inputs&0x08) == 8)
   	   {
   		   estado_in4=8;
   		   valor_in[3]='1';
   	   }else
   	   {
   		   estado_in4=0;
   		   valor_in[3]='0';
   	   }
      }
   if (cambioestado == 1)
   {

/* TEST_1: Visualizo por UART el estado de las salidas*/

#ifdef Test_DigitalInTask
	   char messageinestado[] = "Estado de las entradas: \r";
	   ciaaPOSIX_write(fd_uart1, messageinestado, ciaaPOSIX_strlen(messageinestado));
	   ciaaPOSIX_write(fd_uart1, valor_in, 4);
#endif

	   /* Genero evento de cambio de estado*/
	   cambioestado=0;
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
    *    Read inputs 0..3, update outputs 0..3.
    *    Blink output 4
    */
   char message4[] = "Tarea Analog in \r";
//   ciaaPOSIX_write(fd_uart1, message4, ciaaPOSIX_strlen(message4));
   /* variables to store input/output status */
   ciaaPOSIX_printf("task analog");


   int32_t count;
   uint16_t hr_ciaaDac,adc_1,adc_2,adc_3;

   /* Read ADC1. */

 //  ciaaPOSIX_read(fd_adc, &adc_1, sizeof(adc_1));

   /* Read CH3 , ADC0. */

   ciaaPOSIX_read(fd_adc_0, &adc_3, sizeof(adc_3));

   /* Read CH1 , ADC1. */

   ciaaPOSIX_read(fd_adc_1, &adc_1, sizeof(adc_1));

   //ciaaPOSIX_write(fd_uart1, hr_ciaaDac, ciaaPOSIX_strlen(hr_ciaaDac));
//   ciaaPOSIX_ioctl(fd_adc_0, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_2);
//   ciaaPOSIX_read(fd_adc_0, &adc_2, sizeof(adc_2));

   /* Si cambia el valor actual +-40% genero evento*/

   if(adc_1 > (1.4*valor_ch1) || adc_1 < (0.6*valor_ch1))
   {
	   estado_ch1=1;
   }
   if(adc_3 > (1.4*valor_ch3) || adc_3 < (0.6*valor_ch3))
   {
       estado_ch3=1;
   }

   valor_ch1=adc_1;    // Guardo valor actual
   valor_ch3=adc_3;

/* TEST_3: Visualizo por UART el estado de las salidas*/

#ifdef Test_AnalogInTask
   char messageinestado[] = "\r Estado de las entradas analogicas:  ";
   ciaaPOSIX_write(fd_uart1, messageinestado, ciaaPOSIX_strlen(messageinestado));
   char str[4];
   itoa(valor_ch1,str,10);
   ciaaPOSIX_write(fd_uart1, str,ciaaPOSIX_strlen(str));
   char messageinestado2[] = "    ";
   ciaaPOSIX_write(fd_uart1, messageinestado2, ciaaPOSIX_strlen(messageinestado2));
   itoa(valor_ch3,str,10);
   ciaaPOSIX_write(fd_uart1, str,ciaaPOSIX_strlen(str));
   if ((estado_ch1|estado_ch3) == 1)
   {
	   char messageinestado1[] = "\r --Evento por cambio de estado!!!  ";
	   ciaaPOSIX_write(fd_uart1, messageinestado1, ciaaPOSIX_strlen(messageinestado1));
   }
#endif

   /*Reset banderas de cambio de estado*/
   if ((estado_ch1|estado_ch3) == 1)
   {
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
 //  ciaaPOSIX_write(fd_uart1, message5, ciaaPOSIX_strlen(message5));
   /* variables to store input/output status */
   ciaaPOSIX_printf("ModBusTask");

   /* end ModBusTask */
   TerminateTask();
}

TASK(LedsTask)
{
   /*
    * Example:
    *    Blink Led rojo si statusgps=0, On Led Rojo si statusgps=1
    *    Blink Led verde si statusgsm=0, On Led Verde si statusgsm=1
    */
   uint8_t outputs;

   /* TEST_2: Titilo ambos leds 10 veces y luego los dejo fijos*/
#ifdef Test_LedsTask

//   char message5[] = "Tarea LED  \r";
//   ciaaPOSIX_write(fd_uart1, message5, ciaaPOSIX_strlen(message5));
   /* variables to store input/output status */
//   ciaaPOSIX_printf("LEDSTask");

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
//   ciaaPOSIX_write(fd_uart1, message5, ciaaPOSIX_strlen(message5));
   /* variables to store input/output status */
//   ciaaPOSIX_printf("LEDSTask");
#endif

	char respuesta[30];
	int32_t res;      /* return value variable for posix calls  */
	int8_t dato_gsm[20];

	int8_t buf[20];   /* buffer for uart operation              */
	uint8_t outputs;  /* to store outputs status                */
	int32_t ret;      /* return value variable for posix calls  */
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
				h=0;
				i=0;
				ciaaPOSIX_write(fd_uart2, CGATT, ciaaPOSIX_strlen(CGATT));  //Consulto si tiene señal gprs
				estado_gsm = R_RED;											//Espero respuesta
				TerminateTask();
				break;
			}

			case R_RED:
			{
				//ret = ciaaPOSIX_read(fd_uart1, buf, 20);
				if (ret > 0)
				{
					if (ciaaPOSIX_strncmp(buf,"+CGATT: 1",9)==0) // Si tengo red sigo , sino espero 30 segundos y vuelvo a intentar
					{
						estado_gsm = SET;
						x=0;
					}
					else
					{
						estado_gsm = ERROR;
					}
					ciaaPOSIX_memset(buf, 0, sizeof(buf));  // Limpio el buffer
				}
	//			estado_gsm = SEND;
				break;
			}

			case SET:
			{
				x++;
				//Serial_flush (UART_2);								//Limpio Buffer UART
				if(x==1) ciaaPOSIX_write(fd_uart1, APN, ciaaPOSIX_strlen(APN));  //Consulto si tiene señal gprs
				if(x==2) ciaaPOSIX_write(fd_uart1, CIIR, ciaaPOSIX_strlen(CIIR));  //Consulto si tiene señal gprs "AT+CIICR";
				if(x==3) ciaaPOSIX_write(fd_uart1, CIFSR, ciaaPOSIX_strlen(CIFSR));  //Consulto si tiene señal gprs "AT+CIFSR";
				if(x==4) ciaaPOSIX_write(fd_uart1, IP, ciaaPOSIX_strlen(IP));  //Consulto si tiene se�al gprs IP);
			//	Serial_println (2);
				if(x<5) estado_gsm = R_SET;
				if(x==5)estado_gsm = SEND;
				i=0;
				//memset(respuesta, 0, sizeof(respuesta));  			//Pongo a cero la cadena
				break;
			}
			case R_SET:
			{
				ret = ciaaPOSIX_read(fd_uart1, buf, 20);
				if (ret > 0)
				{
//					if (buf[ret-1] == '\r')  				// si llega Enter detecto fin de respuesta
//					{
						//i=0;
						estado_gsm = SET;
						if (x==1 && !ciaaPOSIX_strncmp(buf,"OK",2)==0) 			estado_gsm = ERROR;	// Si no es correcta la respuesta salto a error
						if (x==2 && !ciaaPOSIX_strncmp(buf,"OK",2)==0) 			estado_gsm = ERROR;
						if (x==3)								 				estado_gsm = SET;
						if (x==4 && !ciaaPOSIX_strncmp(buf,"CONNECT OK",10)==0) estado_gsm = ERROR;

						memset(respuesta, 0, sizeof(respuesta));  // Pongo a cero la cadena
//					}
				}
				break;
			}/*
			case SEND:
			{
				Serial_printString( UART_2, "ACA ENVIO DATOSSSS !!!");
				Serial_println (2);
				Serial_printString( UART_2,"AT+CIPSEND=30");
				Serial_printString( UART_2,last_position);
				estado_gsm = ACKNOLEGE;
				delay=0;
				i=0;
				break;
			}
			case ACKNOLEGE:
			{
				delay++;
				if(delay==1500)
				{
					estado_gsm = SEND;   //espero 15 segundos y vuelvo a intentar (1500)
					Serial_printString( UART_2, "no llego ack reenvio datos !!!");
					delay=0;
					break;
				}

				if (Serial_available(UART_2))
				{
					respuesta[i]=Serial_read(UART_2);
					if (respuesta[i] == '<')							//si llega < detecto fin de comando
					{
						i=0;
						estado_gsm = SET;

						if (!strncmp(respuesta,">ACK<",4)==0)
						{
							Serial_printString( UART_2, "Sin red");
							estado_gsm = SEND;							// Si no es correcta la respuesta salto a error
							memset(respuesta, 0, sizeof(respuesta));  	// Pongo a cero la cadena
							delay=0;
						}
					}
					//if (respuesta[i] == '>')		i++;				// Si llega > detecto inicio comando
				}
				break;
			}
			*/case ERROR:
			{
				h++;
				if(h==1)ciaaPOSIX_write(fd_uart1, SINRED, ciaaPOSIX_strlen(SINRED));
				if(h==500) estado_gsm = RED;   //espero 30 segundos y vuelvo a intentar (3000)
				break;
			}/*
			case DELAY:
			{
				h++;
				if(h==1)Serial_printString( UART_2, "Sin red");
				if(h==1500) estado_gsm = RED;   //espero 15 segundos y vuelvo a intentar (1500)
				break;
			}*/
		}
		blinkled();
   	   /* end LedsTask */
	}//TerminateTask();
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

