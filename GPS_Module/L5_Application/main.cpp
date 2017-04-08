/*
 *     SocialLedge.com - Copyright (C) 2013
 *
 *     This file is part of free software framework for embedded processors.
 *     You can use it and/or distribute it as long as this copyright header
 *     remains unmodified.  The code is free for personal use and requires
 *     permission to use in a commercial product.
 *
 *      THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 *      OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 *      MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 *      I SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
 *      CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
 *     You can reach the author of this software at :
 *          p r e e t . w i k i @ g m a i l . c o m
 */

/**
 * @file
 * @brief This is the application entry point.
 * 			FreeRTOS and stdio printf is pre-configured to use uart0_min.h before main() enters.
 * 			@see L0_LowLevel/lpc_sys.h if you wish to override printf/scanf functions.
 *
 */
#include "tasks.hpp"
#include "examples/examples.hpp"
#include<stdio.h>
#include<utilities.h>
#include<io.hpp>
#include "lpc_sys.h"
#include "lpc17xx.h"
#include "uart0_min.h"
#include"semphr.h"
SemaphoreHandle_t gps_semaphore;
#define BUFSIZE 77
uint8_t UART2Buffer[BUFSIZE];
uint32_t UART2Count=0;
uint32_t UART2TxEmpty=1;
int count=0;
class UART2 : public scheduler_task
{
    public:
        UART2(uint8_t priority) : scheduler_task("task", 2000, priority)
        {
            /* Nothing to init */
        }
        bool init(void)
        {
        	LPC_SC->PCONP|=(1<<24);                                                 //Activate UART2 in PCONP register
        	LPC_SC->PCLKSEL1 &=~(3<<16);                                            //Reset the clock
        	LPC_SC->PCLKSEL1 |= (1<<16);                                            //Set clock as 48MHz
        	LPC_UART2->LCR = (1<<7);                                                //Set DLAB bit
        	LPC_UART2->LCR |= (3<<0);                                               //Set 8-bit character length
        	LPC_UART2->DLM=0;                                                       //Set divisor latch MSB as 0
        	LPC_UART2->DLL = (sys_get_cpu_clock() / (16 * 38400))+0.875;               //set divisor latch LSB
        	LPC_UART2->LCR &= ~(1<<7);                                              //Reset the DLAB bit
        	LPC_UART2->FCR |=(1<<0);                                                //Enable the FIFO
        	LPC_PINCON->PINSEL4 &=~ (3<<16)|(3<<18);                                //Set P2.8 as TxD2
        	LPC_PINCON->PINSEL4 |= (2<<16)|(2<<18);                                 //Set P2.9 as RxD2
        	LPC_UART2->FCR|=(1<<1)|(1<<2);                                          //Clear all bytes in Rx and Tx FIFO
        	NVIC_EnableIRQ(UART2_IRQn);                                         //Enable IRQ
        	LPC_UART2->IER |= (1<<0)|(1<<1)|(1<<2);                             //Enable RBR, THRE, and Rx Line Status interrupts
            return true;
        }
        bool run(void *p)
        {
        	return true;
        }
};

void UARTSend(uint8_t *BufferPtr, uint32_t Length )
{
		UART2TxEmpty=1;
		for(uint32_t i=0;i<Length;i++)                      //Transmit a string of length "Lenghth"
		{
			delay_ms(1);
			while ( !(UART2TxEmpty & 0x01) );                 //Wait while the UART2 Tx FIFO is full. Proceeeds when it becomes empty.
			LPC_UART2->THR = BufferPtr[i];                    //Put the data in the THR register
			UART2TxEmpty = 0;                                 //Indicate that Tx FIFO is not empty, contains valid data.
		}                                                   // UART2TxEmpty is made 1 by the interrupt handler when TX FIFO becomes empty
}

extern "C"
{
        void UART2_IRQHandler(void)                                              //UART Interrupt handler
        {
        	uint8_t IIRValue, LSRValue;
        	uint8_t Dummy;
        	IIRValue=LPC_UART2->IIR;                                             //Read the IIR register (Interrupt Identification register)
        	IIRValue>>=1;                                                        //Shift right by one bit
        	IIRValue &= 0x07;                                                    //perform AND operation on last three bits [3:1] with 1 to read their values
        	LSRValue=LPC_UART2->LSR;                                             //Read the LSR register
        	if ( LSRValue & ((1<<1)|(1<<2)|(1<<3)|(1<<7)|(1<<4)) )               //Check if the error bits are set in LSRValue
        	    {
        	      Dummy = LPC_UART1->RBR;                                        //Read the RBR register and return. (Only way to clear the error)
        	      return;
        	    }
        	if ( LSRValue & (1<<0) )                                             //If there is valid data in the Receive buffer read it and print it
        	{
        		UART2Buffer[UART2Count] = LPC_UART2->RBR;
        		//printf("%c",UART2Buffer[UART2Count]);
        		UART2Count++;
        		if ( UART2Count == BUFSIZE )
        		{
        			UART2Count = 0;
        			xSemaphoreGiveFromISR(gps_semaphore,NULL);
        		}
        	}
        	if ( IIRValue == 0x01 )                                              //If THRE interrupt is enabled
        	{
        			LSRValue = LPC_UART2->LSR;
        			if ( LSRValue & (1<<5) )                                    //If the Tx FIFO is empty return UART2TxEmpty=1
        			{
        				UART2TxEmpty = 1;
        			}
        			else
        			{
        				UART2TxEmpty = 0;
        			}
        		}
        }
}
void Print_Latitude_and_Longitude(void *p)
{
	while(1)
	{
		int start=0,comma=0,once=0;
		if(xSemaphoreTake(gps_semaphore,9999999))
		{
			if(UART2Buffer[43]=='1')
			{
			for(int i=0;i<77;i++)
			{
				//printf("%c",UART2Buffer);
				//printf("%c",UART2Buffer[i]);
				if(UART2Buffer[i]=='$')
				{
					start=1;
				}
				if(start==1&&UART2Buffer[i]==',')
				{
					comma++;
				}
				if(comma==2&&once==0)
				{
					count=0;
					printf("\nLatitude: ");
					for(int j=i+1;count<11;j++)
					{
						if(j==77)
						{
							j=0;
						}
						printf("%c",UART2Buffer[j]);
						count++;
					}
					once=1;
				}
				if(comma==4)
				{
					count=0;
					printf("\nLongitude: ");
					for(int j=i+1;count<12;j++)
					{
						if(j==77)
						{
							j=0;
						}
						printf("%c",UART2Buffer[j]);
						count++;
					}
					start=0;
					comma=0;
					once=0;
				}

			}
			}
		}
	}
}


/**
 * The main() creates tasks or "threads".  See the documentation of scheduler_task class at scheduler_task.hpp
 * for details.  There is a very simple example towards the beginning of this class's declaration.
 *
 * @warning SPI #1 bus usage notes (interfaced to SD & Flash):
 *      - You can read/write files from multiple tasks because it automatically goes through SPI semaphore.
 *      - If you are going to use the SPI Bus in a FreeRTOS task, you need to use the API at L4_IO/fat/spi_sem.h
 *
 * @warning SPI #0 usage notes (Nordic wireless)
 *      - This bus is more tricky to use because if FreeRTOS is not running, the RIT interrupt may use the bus.
 *      - If FreeRTOS is running, then wireless task may use it.
 *        In either case, you should avoid using this bus or interfacing to external components because
 *        there is no semaphore configured for this bus and it should be used exclusively by nordic wireless.
 */
int main(void)
{
	vSemaphoreCreateBinary( gps_semaphore);
		xSemaphoreTake( gps_semaphore, 0);

			scheduler_add_task(new UART2(PRIORITY_LOW));
			xTaskCreate(Print_Latitude_and_Longitude, (const char*)"printing_gps", STACK_BYTES(2048), 0, 1, 0);
    /**
     * A few basic tasks for this bare-bone system :
     *      1.  Terminal task provides gateway to interact with the board through UART terminal.
     *      2.  Remote task allows you to use remote control to interact with the board.
     *      3.  Wireless task responsible to receive, retry, and handle mesh network.
     *
     * Disable remote task if you are not using it.  Also, it needs SYS_CFG_ENABLE_TLM
     * such that it can save remote control codes to non-volatile memory.  IR remote
     * control codes can be learned by typing the "learn" terminal command.
     */
    scheduler_add_task(new terminalTask(PRIORITY_HIGH));

    /* Consumes very little CPU, but need highest priority to handle mesh network ACKs */
    scheduler_add_task(new wirelessTask(PRIORITY_CRITICAL));

    /* Change "#if 0" to "#if 1" to run period tasks; @see period_callbacks.cpp */
    #if 0
    const bool run_1Khz = false;
    scheduler_add_task(new periodicSchedulerTask(run_1Khz));
    #endif

    /* The task for the IR receiver to "learn" IR codes */
    // scheduler_add_task(new remoteTask  (PRIORITY_LOW));

    /* Your tasks should probably used PRIORITY_MEDIUM or PRIORITY_LOW because you want the terminal
     * task to always be responsive so you can poke around in case something goes wrong.
     */

    /**
     * This is a the board demonstration task that can be used to test the board.
     * This also shows you how to send a wireless packets to other boards.
     */
    #if 0
        scheduler_add_task(new example_io_demo());
    #endif

    /**
     * Change "#if 0" to "#if 1" to enable examples.
     * Try these examples one at a time.
     */
    #if 0
        scheduler_add_task(new example_task());
        scheduler_add_task(new example_alarm());
        scheduler_add_task(new example_logger_qset());
        scheduler_add_task(new example_nv_vars());
    #endif

    /**
	 * Try the rx / tx tasks together to see how they queue data to each other.
	 */
    #if 0
        scheduler_add_task(new queue_tx());
        scheduler_add_task(new queue_rx());
    #endif

    /**
     * Another example of shared handles and producer/consumer using a queue.
     * In this example, producer will produce as fast as the consumer can consume.
     */
    #if 0
        scheduler_add_task(new producer());
        scheduler_add_task(new consumer());
    #endif

    /**
     * If you have RN-XV on your board, you can connect to Wifi using this task.
     * This does two things for us:
     *   1.  The task allows us to perform HTTP web requests (@see wifiTask)
     *   2.  Terminal task can accept commands from TCP/IP through Wifly module.
     *
     * To add terminal command channel, add this at terminal.cpp :: taskEntry() function:
     * @code
     *     // Assuming Wifly is on Uart3
     *     addCommandChannel(Uart3::getInstance(), false);
     * @endcode
     */
    #if 0
        Uart3 &u3 = Uart3::getInstance();
        u3.init(WIFI_BAUD_RATE, WIFI_RXQ_SIZE, WIFI_TXQ_SIZE);
        scheduler_add_task(new wifiTask(Uart3::getInstance(), PRIORITY_LOW));
    #endif

    scheduler_start(); ///< This shouldn't return
    return -1;
}
