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
static int32_t fd_adc;
static int32_t fd_adc_3;
static int32_t fd_adc_2;

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

uint8_t statusgps=0;
uint8_t statusgsm=0;


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
   ciaaPOSIX_ioctl(fd_uart2, ciaaPOSIX_IOCTL_SET_BAUDRATE, (void *)ciaaBAUDRATE_115200);

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
   fd_adc_3 = ciaaPOSIX_open("/dev/serial/aio/in/0", ciaaPOSIX_O_RDONLY);

   /* open CIAA CH3 */
   ciaaPOSIX_ioctl(fd_adc_3, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);
   ciaaPOSIX_ioctl(fd_adc_3, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_3);

   /* open CIAA CH2 */
 //  ciaaPOSIX_ioctl(fd_adc_2, ciaaPOSIX_IOCTL_SET_SAMPLE_RATE, 100000);
 //  ciaaPOSIX_ioctl(fd_adc_2, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_2);

   /* activate example tasks */
   Periodic_Task_Counter = 0;
   SetRelAlarm(ActivateDigitalInTask, 200, 500);  //Cada 500 ms
   SetRelAlarm(ActivateLedsTask, 100, 300);  //Cada 500 ms
   /*Contadores a cero*/
   Contador_In1=0;
   Contador_In2=0;
   Contador_In3=0;
   Contador_In4=0;

   /* Activates the SerialEchoTask task */
   ActivateTask(SerialEchoTask);

   /* end InitTask */
   TerminateTask();
}

/** \brief Serial Echo Task
 *
 * This tasks waits for input data from fd_uart1 and writes the received data
 * to fd_uart1 and fd_uart2. This taks alos blinkgs the output 5.
 *
 */
TASK(SerialEchoTask)
{
   int8_t buf[20];   /* buffer for uart operation              */
   uint8_t outputs;  /* to store outputs status                */
   int32_t ret;      /* return value variable for posix calls  */

   ciaaPOSIX_printf("SerialEchoTask...\n");
   /* send a message to the world :) */
   char message[] = "Hi! :)\nSerialEchoTask: Waiting for characters...\n";
   char message1[] = "Hi! :)\nSerialEchoTask: ESPERAR CARACTERES uart0...\n";
   char message2[] = "Hi! :)\nSerialEchoTask: ESPERAR CARACTERES uart2...\n";
   ciaaPOSIX_write(fd_uart0, message1, ciaaPOSIX_strlen(message1));
   ciaaPOSIX_write(fd_uart1, message, ciaaPOSIX_strlen(message));
   ciaaPOSIX_write(fd_uart2, message2, ciaaPOSIX_strlen(message2));
   while(1)
   {
      /* wait for any character ... */
      ret = ciaaPOSIX_read(fd_uart0, buf, 20);

      if(ret > 0)
      {
         /* ... and write them to the same device */
         ciaaPOSIX_write(fd_uart0, buf, ret);

         /* also write them to the other device */
         ciaaPOSIX_write(fd_uart1, buf, ret);

         /* also write them to the other device */
         ciaaPOSIX_write(fd_uart2, buf, ret);
      }

      /* blink output 5 with each loop */
      ciaaPOSIX_read(fd_out, &outputs, 1);
      outputs ^= 0x20;
      ciaaPOSIX_write(fd_out, &outputs, 1);
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
	   if ((inputs&0x01) == 1)
	   {
		   char messagein11[] = "IN1=1 \r";
		   ciaaPOSIX_write(fd_uart1, messagein11, ciaaPOSIX_strlen(messagein11));
		   estado_in1=1;
	   }else
	   {
		   char messagein10[] = "IN1=0 \r";
		   ciaaPOSIX_write(fd_uart1, messagein10, ciaaPOSIX_strlen(messagein10));
		   estado_in1=0;
	   }
   }
   if ((inputs&0x02) != estado_in2)
      {
   	   if ((inputs&0x02) == 2)
   	   {
   		   char messagein21[] = "IN2=1 \r";
   		   ciaaPOSIX_write(fd_uart1, messagein21, ciaaPOSIX_strlen(messagein21));
   		   estado_in2=2;
   	   }else
   	   {
   		   char messagein20[] = "IN2=0 \r";
   		   ciaaPOSIX_write(fd_uart1, messagein20, ciaaPOSIX_strlen(messagein20));
   		   estado_in2=0;
   	   }
      }
   if ((inputs&0x04) != estado_in3)
      {
   	   if ((inputs&0x04) == 4)
   	   {
   		   char messagein31[] = "IN3=1 \r";
   		   ciaaPOSIX_write(fd_uart1, messagein31, ciaaPOSIX_strlen(messagein31));
   		   estado_in3=4;
   	   }else
   	   {
   		   char messagein30[] = "IN3=0 \r";
   		   ciaaPOSIX_write(fd_uart1, messagein30, ciaaPOSIX_strlen(messagein30));
   		   estado_in3=0;
   	   }
      }
   if ((inputs&0x08) != estado_in4)
      {
   	   if ((inputs&0x08) == 8)
   	   {
   		   char messagein41[] = "IN4=1 \r";
   		   ciaaPOSIX_write(fd_uart1, messagein41, ciaaPOSIX_strlen(messagein41));
   		   estado_in4=8;
   	   }else
   	   {
   		   char messagein40[] = "IN4=0 \r";
   		   ciaaPOSIX_write(fd_uart1, messagein40, ciaaPOSIX_strlen(messagein40));
   		   estado_in4=0;
   	   }
      }

   blinkled();
   /* read outputs */
 //  ciaaPOSIX_read(fd_out, &outputs, 1);

   /* update outputs with inputs */
   //outputs &= 0xF0;
   //outputs |= inputs & 0x0F;

   /* blink */
 //  outputs ^= 0x10;

   /* write */
 //  ciaaPOSIX_write(fd_out, &outputs, 1);

   /* Print Task info */
   //Periodic_Task_Counter++;
//   ciaaPOSIX_printf("Periodic Task: %d\n", Periodic_Task_Counter);

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

   /* Read ADC2. */
   ciaaPOSIX_ioctl(fd_adc_3, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_2);
   ciaaPOSIX_read(fd_adc_3, &adc_2, sizeof(adc_2));

   /* Read ADC3. */
   ciaaPOSIX_ioctl(fd_adc_3, ciaaPOSIX_IOCTL_SET_CHANNEL, ciaaCHANNEL_3);
   ciaaPOSIX_read(fd_adc_3, &adc_3, sizeof(adc_3));

   //ciaaPOSIX_write(fd_uart1, hr_ciaaDac, ciaaPOSIX_strlen(hr_ciaaDac));

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
    *    Blink Led rojo si statusgps=0
    *    Blink Led verde si statusgsm=0
    */
   uint8_t outputs;
   char message5[] = "Tarea LED  \r";
   ciaaPOSIX_write(fd_uart1, message5, ciaaPOSIX_strlen(message5));
   /* variables to store input/output status */
   ciaaPOSIX_printf("LEDSTask");
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

   if (Periodic_Task_Counter == 20)
   {
	   statusgsm=1;
	   statusgps=1;
   }

   /* end LedsTask */
   TerminateTask();
}
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/** @} doxygen end group definition */
/*==================[end of file]============================================*/

