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
#include <stdio.h>
#include<LPC17xx.h>
#include "utilities.h"
#include "L4_IO\fat\disk\spi_flash.h"
#include "L4_IO\fat\disk\disk_defines.h"
#include "tasks.hpp"
#include "printf_lib.h"
 #include "uart0_min.h"
#include "lpc_pwm.hpp"
#include "examples/examples.hpp"
#include "io.hpp"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
/*------------------For Servo && DC motor-------------*/
unsigned int msTcMax =0;
QueueHandle_t Right_Sensor_data;
QueueHandle_t Left_Sensor_data;
QueueHandle_t Front_Sensor_data;
QueueHandle_t Back_Sensor_data;

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
//Class implemented for Servo and DC task
class servomotor_task : public scheduler_task
{
    public:
	servomotor_task(uint8_t priority) : scheduler_task("servomotortask", 2000, priority)
        {
             //Nothing to init
        }
		//Servomotor Hardware pins initialization
        bool init(void)
        {
        	/*----------------Servo motor initialization-------------*/
        	 msTcMax = (sys_get_cpu_clock() / 50);
        	//using PWM1.2, GPIO PIN - 2.1
        	 LPC_SC->PCONP |= (1 << 6);
        	 LPC_SC->PCLKSEL0 &= ~(3 <<12); // Clear clock Bits
        	 LPC_SC->PCLKSEL0 |=  (1 << 12); // CCLK / 1
        	  //Enable Counters,PWM module
        	 LPC_PWM1->MCR |= (1<<1);  //Reset on PWMMR0, reset TC if it matches MR0
        	 LPC_PWM1->MR0 =   msTcMax;    //set PWM cycle(Ton+Toff)=100)
        	 LPC_PWM1->TCR = (1<<0) | (1<<3);
	         LPC_PWM1->CTCR &= ~(0xF << 0);
	         LPC_PINCON->PINSEL4 &= ~(3 <<2); // P2.1
        	 LPC_PINCON->PINSEL4 |= (1<<2); // P2.1
        	 LPC_PWM1->PCR |=(1<<10);//The PWM2 output enabled.
        	 /*-------------------------------------------------------*/
        	 /*-----------------DC motor initialization-------------*/
        	 //	IN1=0 and IN2=0 -> Motor1 idle
			//	IN1=0 and IN2=1 -> Motor1 Anti-clock wise direction
			//	IN1=1 and IN2=0 -> Motor1 Clock wise direction
			//	IN1=1 and IN2=1 -> Motor1 idle

			//	IN3=0 and IN4=0 -> Motor2 idle
			//	IN3=0 and IN4=1 -> Motor2 Anti-clock wise direction
			//	IN3=1 and IN4=0 -> Motor2 Clock wise direction
			//	IN3=1 and IN4=1 -> Motor2 idle

			//Enable
			//ENA, ENB pin is connected to the 5V DC to drive the motor

			//	P2.0 -> IN1, IN3;
			//  P2.1 -> IN2, IN4
			LPC_GPIO2->FIODIR |= ((1 << 0)|(1 << 2));	//Set P2.0, P2.1 as output pins
			/*-------------------------------------------------------*/

            return true;
        }
        void set(float percent){
             const unsigned int valueNeeded = (percent * msTcMax) / 100;
        	 LPC_PWM1->MR2 = valueNeeded;
        	 // Enable the latch
        	 LPC_PWM1->LER |= (1 << 2);
        }
        void move_forward (void)
		{
			//	IN1=1 and IN2=0 -> Motor1 Clock wise direction
			//	IN3=1 and IN4=0 -> Motor2 Clock wise direction
			LPC_GPIO2->FIOSET |=  (1 << 0);	// Set IN1, IN3 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 2); // Set IN2, IN4 = 0
			u0_dbg_printf("\n Clockwise direction ");
		}
		void move_reverse (void)
		{
			//	IN1=0 and IN2=1 -> Motor1 Anti-clock wise direction
			//	IN3=0 and IN4=1 -> Motor2 Anti-clock wise direction
			LPC_GPIO2->FIOCLR |=  (1 << 0);	// Set IN1, IN3 = 0
			LPC_GPIO2->FIOSET |=  (1 << 2); // Set IN2, IN4 = 1
			u0_dbg_printf("\n Anti-clockwise direction ");
		}
		void stop(void)
		{
			//	IN1=0 and IN2=0 -> Motor1 Idle
			// 	IN3=0 and IN4=0 -> Motor2 Idle
			LPC_GPIO2->FIOCLR |=  (1 << 0);	// Set IN1, IN3 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 2); // Set IN2, IN4 = 0
			u0_dbg_printf("\n Stop ");
		}
        bool run(void *p)
        {

			uint32_t r_val, l_val, f_val, b_val;
			xQueueReceive( Right_Sensor_data, &r_val, ( TickType_t ) 10 );
			xQueueReceive( Left_Sensor_data,  &l_val, ( TickType_t ) 10 );
			xQueueReceive( Front_Sensor_data, &f_val, ( TickType_t ) 10 );
			xQueueReceive( Back_Sensor_data,  &b_val, ( TickType_t ) 10 );

        	//set(7.5);
        	//vTaskDelay(15);
			f_val = 11;
			b_val = 9;
			l_val = 9;
			r_val = 9;

			if (f_val > 10)
			{
				move_forward();
			}
			else if(l_val>10){
				set(5.2);
				move_forward();
			}
			else if(r_val>10){
				set(10.0);
				move_forward();
			}
			else if (b_val > 10)
			{
				move_reverse();
			}
			else
			{
				stop();
			}
			vTaskDelay(15);
            return true;
      }
};

//Servo and Dc motor function
void servo_motor(void){
	//using PWM1.2, GPIO PIN - 2.1
	 Right_Sensor_data = xQueueCreate(3, sizeof(uint32_t));
	 Left_Sensor_data  = xQueueCreate(3, sizeof(uint32_t));
	 Front_Sensor_data = xQueueCreate(3, sizeof(uint32_t));
	 Back_Sensor_data  = xQueueCreate(3, sizeof(uint32_t));
	 scheduler_add_task(new servomotor_task(PRIORITY_LOW));
}

int main(void)
{


	//servo motor interfacing
	servo_motor();

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
