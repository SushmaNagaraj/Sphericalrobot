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
#include "tasks.hpp"
#include "printf_lib.h"
#include "uart0_min.h"
#include "lpc_pwm.hpp"
#include "io.hpp"
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "eint.h"
#include "utilities.h"

/*------------------For Sensors-------------*/
SemaphoreHandle_t Triggersonar;
SemaphoreHandle_t startRight,startLeft,startFront,startBack,getRight,getLeft,getFront,getBack;

/*------------------For Servo && DC motor-------------*/
unsigned int msTcMax =0;
QueueHandle_t Right_Sensor_data;
QueueHandle_t Left_Sensor_data;
QueueHandle_t Front_Sensor_data;
QueueHandle_t Back_Sensor_data;
uint32_t obst_limit = 25;

//Interrupts for the sensors
extern "C"
{
	void sphero_RIT_IRQHandler()
	{
		LPC_RIT->RICTRL |= 1<<0; //write 1 to clear bit
		long taskWoken = 0;
		xSemaphoreGiveFromISR(Triggersonar,&taskWoken);
		if(taskWoken)
		{
			taskWoken = 0 ;
			vPortYield();
		}
	}

	void sphero_EINT3_IRQHandler()
	{
		long taskwoken = 0;
//---------------- hardware interrupt on port 0.0 for Right sensor -------------//
		if(LPC_GPIOINT->IO0IntStatR & 1<<0)
		{
			LPC_GPIOINT->IO0IntClr = 1<<0;
			xSemaphoreGiveFromISR(startRight,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}
		else if(LPC_GPIOINT->IO0IntStatF & 1<<0)
		{
			LPC_GPIOINT->IO0IntClr = 1<<0;
			xSemaphoreGiveFromISR(getRight,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}

//---------------- hardware interrupt on port 0.1 for Left sensor -------------//
		if(LPC_GPIOINT->IO0IntStatR & 1<<1)
		{
			LPC_GPIOINT->IO0IntClr = 1<<1;
			xSemaphoreGiveFromISR(startLeft,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}
		else if(LPC_GPIOINT->IO0IntStatF & 1<<1)
		{
			LPC_GPIOINT->IO0IntClr = 1<<1;
			xSemaphoreGiveFromISR(getLeft,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}

//---------------- hardware interrupt on port 0.29 for Front sensor -------------//
		if(LPC_GPIOINT->IO0IntStatR & 1<<29)
		{
			LPC_GPIOINT->IO0IntClr = 1<<29;
			xSemaphoreGiveFromISR(startFront,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}
		else if(LPC_GPIOINT->IO0IntStatF & 1<<29)
		{
			LPC_GPIOINT->IO0IntClr = 1<<29;
			xSemaphoreGiveFromISR(getFront,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}

//---------------- hardware interrupt on port 0.30 for Back sensor -------------//
		if(LPC_GPIOINT->IO0IntStatR & 1<<30)
		{
			LPC_GPIOINT->IO0IntClr = 1<<30;
			xSemaphoreGiveFromISR(startBack,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}
		else if(LPC_GPIOINT->IO0IntStatF & 1<<30)
		{
			LPC_GPIOINT->IO0IntClr = 1<<30;
			xSemaphoreGiveFromISR(getBack,&taskwoken);
			if(taskwoken)
			{
				taskwoken = 0 ;
				vPortYield();
			}
		}
	}
}

//class for triggering the sensor
class sonar : public scheduler_task
{
	public:
	sonar(uint8_t priority) : scheduler_task("sonar",512,priority,NULL)
	{
	}

	bool init(void)
	{
		LPC_GPIO1->FIODIR |= 1<<1;
		LPC_GPIO0->FIODIR |= 1<<26;

		LPC_GPIOINT->IO0IntEnR |= 1<<0;		//	ENABLED RISING EDGE INTERRUPT ON 0.0
		LPC_GPIOINT->IO0IntEnF |= 1<<0;		//	ENABLED FALLING EDGE INTERRUPT ON 0.0

		LPC_GPIOINT->IO0IntEnR |= 1<<1;		//	ENABLED RISING EDGE INTERRUPT ON 0.1
		LPC_GPIOINT->IO0IntEnF |= 1<<1;		//	ENABLED FALLING EDGE INTERRUPT ON 0.1

		LPC_GPIOINT->IO0IntEnR |= 1<<29;	//	ENABLED RISING EDGE INTERRUPT ON 0.29
		LPC_GPIOINT->IO0IntEnF |= 1<<29;	//	ENABLED FALLING EDGE INTERRUPT ON 0.29

		LPC_GPIOINT->IO0IntEnR |= 1<<30;	//	ENABLED RISING EDGE INTERRUPT ON 0.30
		LPC_GPIOINT->IO0IntEnF |= 1<<30;	//	ENABLED FALLING EDGE INTERRUPT ON 0.30

		return true;
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(Triggersonar,portMAX_DELAY))
		{
			LPC_GPIO1->FIOCLR = 1<<1;		// 	LED on
			LPC_GPIO0->FIOSET = 1<<26;		//	Triggering sonar
			delay_us(20);					//	Delay of 25 uS
			LPC_GPIO0->FIOCLR = 1<<26;		//	Stopped triggering
			LPC_GPIO1->FIOSET = 1<<1;		//	LED off
			puts("\n-\n");
		}
		return true;
	}
};

//Right sensor distance calculated class
class Rightsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Rightsensor(uint8_t priority) : scheduler_task("right sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
		Right_Sensor_data = xQueueCreate(1, sizeof(uint32_t));
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startRight,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getRight,portMAX_DELAY))
		{
			duration = (sys_get_uptime_us() - Time)/147;
			u0_dbg_printf("right %lu\n",duration);
			xQueueSend(Right_Sensor_data, &duration,0);
		}
		return true;
	}
};

//Left sensor distance calculated class

class Leftsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Leftsensor(uint8_t priority) : scheduler_task("left sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
		Left_Sensor_data = xQueueCreate(1, sizeof(uint32_t));
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startLeft,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getLeft,portMAX_DELAY))
		{
			duration = (sys_get_uptime_us() - Time)/147;
			u0_dbg_printf("left %lu\n",duration);
			xQueueSend(Left_Sensor_data, &duration,0);
		}
		return true;
	}
};

//Front sensor distance calculated class
class Frontsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Frontsensor(uint8_t priority) : scheduler_task("Front sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
		Front_Sensor_data = xQueueCreate(1, sizeof(uint32_t));
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startFront,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getFront,portMAX_DELAY))
		{
			duration = (sys_get_uptime_us() - Time)/147;
			u0_dbg_printf("front %lu\n",duration);
			xQueueSend(Front_Sensor_data, &duration,0);
		}
		return true;
	}
};

//Back sensor distance calculated class
class Backsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Backsensor(uint8_t priority) : scheduler_task("Back sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
		Back_Sensor_data = xQueueCreate(1, sizeof(uint32_t));
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startBack,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getBack,portMAX_DELAY))
		{
			duration = (sys_get_uptime_us() - Time)/147;
			u0_dbg_printf("back %lu\n",duration);
			xQueueSend(Back_Sensor_data, &duration, 0);
		}
		return true;
	}
};

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
			//  P2.2 -> IN2, IN4
			LPC_GPIO2->FIODIR |= ((1 << 2)|(1 << 3)|(1 << 4)|(1 << 5));	//Set P2.2, P2.3, P2.4, P2.5 as output pins
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
			LPC_GPIO2->FIOSET |=  (1 << 2);	// Set IN1 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 3); // Set IN2 = 0
			LPC_GPIO2->FIOSET |=  (1 << 4); // Set IN3 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 5); // Set IN4 = 0
			//u0_dbg_printf("\n Clockwise direction ");
		}
		void move_reverse (void)
		{
			//	IN1=0 and IN2=1 -> Motor1 Anti-clock wise direction
			//	IN3=0 and IN4=1 -> Motor2 Anti-clock wise direction
			LPC_GPIO2->FIOCLR |=  (1 << 2);	// Set IN1 = 0
			LPC_GPIO2->FIOSET |=  (1 << 3); // Set IN2 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 4); // Set IN3 = 0
			LPC_GPIO2->FIOSET |=  (1 << 5); // Set IN4 = 1
			//u0_dbg_printf("\n Anti-clockwise direction ");
		}
		void stop(void)
		{
			//	IN1=0 and IN2=0 -> Motor1 Idle
			// 	IN3=0 and IN4=0 -> Motor2 Idle
			LPC_GPIO2->FIOCLR |=  (1 << 2);	// Set IN1 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 3); // Set IN2 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 4); // Set IN3 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 5); // Set IN4 = 0
			//u0_dbg_printf("\n Stop ");
		}
        bool run(void *p)
        {

			uint32_t r_val, l_val, f_val, b_val;
			vTaskDelay(160);
			xQueueReceive( Right_Sensor_data, &r_val, 160);
			xQueueReceive( Left_Sensor_data,  &l_val, 160);
			xQueueReceive( Front_Sensor_data, &f_val, 160);
			xQueueReceive( Back_Sensor_data,  &b_val, 160);

			u0_dbg_printf("Right dc = %i\n", r_val);
			u0_dbg_printf("left dc = %i\n", l_val);
			u0_dbg_printf("front dc = %i\n", f_val);
			u0_dbg_printf("back dc = %i\n", b_val);

			/*f_val = 11;
			l_val = 8;
			r_val = 9;
			b_val = 11;*/


			if(f_val < obst_limit && l_val < obst_limit && r_val < obst_limit ){
				if(b_val > obst_limit){
					move_reverse();
				}else{
					stop();
				}
			}
			else{
				if(f_val < obst_limit && l_val < obst_limit && r_val > obst_limit ){
					set(10.0);
				}
				if(f_val < obst_limit && l_val > obst_limit && r_val < obst_limit ){
					set(5.2);
				}
				if(f_val < obst_limit && l_val > obst_limit && r_val > obst_limit ){
					if(l_val > r_val){
						set(6.2);
					}else{
						set(9.0);
					}
				}
				if(f_val > obst_limit && l_val < obst_limit && r_val < obst_limit ){
					if(l_val < r_val){
						set(9.0);
					}else{
						set(7.0);
					}
				}
				if(f_val > 10 && l_val < 10 && r_val > 10 ){
					set(10.0);
				}
				if(f_val > 10 && l_val > 10 && r_val < 10 ){
					set(5.2);
				}

					vTaskDelay(1);
					set(7.5);
					vTaskDelay(1);
					move_forward();
			}
            return true;
      }
};

//sensor fucntion
void sensor(void){

	vSemaphoreCreateBinary(Triggersonar);

	vSemaphoreCreateBinary(startRight);
	vSemaphoreCreateBinary(getRight);

	vSemaphoreCreateBinary(startLeft);
	vSemaphoreCreateBinary(getLeft);

	vSemaphoreCreateBinary(startFront);
	vSemaphoreCreateBinary(getFront);

	vSemaphoreCreateBinary(startBack);
	vSemaphoreCreateBinary(getBack);

	LPC_PINCON->PINSEL2 |= 0<<2;
	LPC_GPIO1->FIODIR |= 1<<0;
	LPC_SC->PCONP |= 1<<16;             // Power Control for Peripherals register: power up RIT clock
	LPC_SC->PCLKSEL1 |= (1<<26);  		// Peripheral clock selection: divide clock by 8 (run RIT clock by 12MHz)
	LPC_RIT->RICOUNTER = 0;             // set counter to zero
	LPC_RIT->RICOMPVAL = 0x00FFFFFF;    // interrupt tick every second
	LPC_RIT->RICTRL |= 1<<1;            // clear timer when counter reaches value
	LPC_RIT->RICTRL |= 1<<3;            // enable timer

	scheduler_add_task(new sonar(PRIORITY_CRITICAL));
	scheduler_add_task(new Rightsensor(5));
	scheduler_add_task(new Leftsensor(4));
	scheduler_add_task(new Frontsensor(3));
	scheduler_add_task(new Backsensor(2));

	//enable interrupt
	NVIC_EnableIRQ(RIT_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);
}

//Servo and Dc motor function
void dc_servo_motor(void){
	//using PWM1.2, GPIO PIN - 2.1
	 scheduler_add_task(new servomotor_task(1));
}


int main(void)
{

	//sensor interfacing
	sensor();
	//servo motor interfacing
	dc_servo_motor();

    scheduler_start(); ///< This shouldn't return
    return -1;
}
