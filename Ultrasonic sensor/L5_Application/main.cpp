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
#include "io.hpp"
#include "utilities.h"
#include "printf_lib.h"
#include "eint.h"
#include "semphr.h"


SemaphoreHandle_t Triggersonar;
SemaphoreHandle_t startRight,startLeft,startFront,startBack,getRight,getLeft,getFront,getBack;

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

class Rightsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Rightsensor(uint8_t priority) : scheduler_task("right sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startRight,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getRight,portMAX_DELAY))
		{
			duration = sys_get_uptime_us() - Time;
			printf("right %lu\n",duration/147);
		}
		return true;
	}
};

class Leftsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Leftsensor(uint8_t priority) : scheduler_task("left sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startLeft,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getLeft,portMAX_DELAY))
		{
			duration = sys_get_uptime_us() - Time;
			printf("left %lu\n",duration/147);
		}
		return true;
	}
};

class Frontsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Frontsensor(uint8_t priority) : scheduler_task("Front sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startFront,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getFront,portMAX_DELAY))
		{
			duration = sys_get_uptime_us() - Time;
			printf("Front %lu\n",duration/147);
		}
		return true;
	}
};

class Backsensor : public scheduler_task
{
	private:
	uint32_t Time,duration;

	public:
	Backsensor(uint8_t priority) : scheduler_task("Back sensor",1024,priority,NULL)
	{
		Time = 0;
		duration = 0;
	}

	bool run(void *p)
	{
		if(xSemaphoreTake(startBack,portMAX_DELAY))
		{
			Time = sys_get_uptime_us();
		}

		if(xSemaphoreTake(getBack,portMAX_DELAY))
		{
			duration = sys_get_uptime_us() - Time;
			printf("Back %lu\n",duration/147);
		}
		return true;
	}
};


int main(void)
{

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

	scheduler_add_task(new Rightsensor(4));
	scheduler_add_task(new Leftsensor(3));
	scheduler_add_task(new Frontsensor(2));
	scheduler_add_task(new Backsensor(1));

	//enable interrupt
	NVIC_EnableIRQ(RIT_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);

    scheduler_start(); ///< This shouldn't return
    return -1;
}
