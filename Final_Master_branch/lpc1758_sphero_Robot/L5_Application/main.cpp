#include <stdio.h>
#include "tasks.hpp"
#include "io.hpp"
#include "utilities.h"
#include "printf_lib.h"
#include "eint.h"
#include "semphr.h"
#include "queue.h"
#include <string.h>
#include <algorithm>
#include <storage.hpp>
#include "L4_IO/FAT/disk/sd.h"
#include "uart3.hpp"
#include <fstream>

#define BUFSIZE 77
using namespace std;
volatile char start = ' ';
uint8_t UART2Buffer[BUFSIZE];
uint32_t UART2Count=0;
uint32_t UART2TxEmpty=1;
unsigned int msTcMax=0;
int offset=0;
char *writable;
char* send;
volatile char file = ' ';

QueueHandle_t gps_write_queue;
SemaphoreHandle_t sending_to_queue,
					gps_semaphore,
					sd_card_mutex,
					main_sem,
					stop_sem;

typedef enum {
   left_SensorQueueId=0,
   right_SensorQueueId,
   front_SensorQueueId,
   back_SensorQueueId
} sharedHandleId_t;

SemaphoreHandle_t Triggersonar,
					startRight,
					startLeft,
					startFront,
					startBack,
					getRight,
					getLeft,
					getFront,
					getBack;

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

	void sphero_UART2_IRQHandler(void)                  		//UART Interrupt handler
	{
		uint8_t IIRValue, LSRValue;
		IIRValue=LPC_UART2->IIR;                                //Read the IIR register (Interrupt Identification register)
		IIRValue>>=1;                                           //Shift right by one bit
		IIRValue &= 0x07;                                       //perform AND operation on last three bits [3:1] with 1 to read their values
		LSRValue=LPC_UART2->LSR;                                //Read the LSR register
		if ( LSRValue & ((1<<1)|(1<<2)|(1<<3)|(1<<7)|(1<<4)) )  //Check if the error bits are set in LSRValue
			{
			  return;
			}
		if ( LSRValue & (1<<0) )                                //If there is valid data in the Receive buffer read it and print it
		{
			UART2Buffer[UART2Count] = LPC_UART2->RBR;
			UART2Count++;
			if ( UART2Count == BUFSIZE )
			{
				UART2Count = 0;
				xSemaphoreGiveFromISR(gps_semaphore,NULL);
			}
		}
	}

    void UART3_IRQHandler()
    {

        //Extract the first bit from LSR register
        uint8_t LSR_chk = LPC_UART3->LSR & 0x01;

        //check the RDR bit
        if(LSR_chk)
        {
            char c = LPC_UART3->RBR;
            //u0_dbg_printf("%c",c);//print the received character
            if(c == 'F')
                file = c;
            else
                start = c;


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
			//LPC_GPIO1->FIOCLR = 1<<1;		// 	LED on
			LPC_GPIO0->FIOSET = 1<<26;		//	Triggering sonar
			delay_us(25);					//	Delay of 25 uS
			LPC_GPIO0->FIOCLR = 1<<26;		//	Stopped triggering
			//LPC_GPIO1->FIOSET = 1<<1;		//	LED off
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

		QueueHandle_t Right_Q = xQueueCreate(5,sizeof(uint32_t));
		addSharedObject(right_SensorQueueId,Right_Q);
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
			duration /= 147;
			xQueueSend(getSharedObject(right_SensorQueueId),&duration,0);
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
		QueueHandle_t Left_Q = xQueueCreate(5,sizeof(uint32_t));
		addSharedObject(left_SensorQueueId,Left_Q);
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
			duration /= 147;
			xQueueSend(getSharedObject(left_SensorQueueId),&duration,0);
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
		QueueHandle_t Front_Q = xQueueCreate(5,sizeof(uint32_t));
		addSharedObject(front_SensorQueueId,Front_Q);
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
			duration /= 147;
			xQueueSend(getSharedObject(front_SensorQueueId),&duration,0);
		}
		return true;
	}
};

class motor : public scheduler_task
{
	private:
		bool key=false;
    public:
	motor(uint8_t priority) : scheduler_task("Motors", 5000, priority)
        {
        }

        bool init(void)
        {
			/*---------------Servo motor initialization-------------*/
			msTcMax = (sys_get_cpu_clock() / 625);		// 67133 FOR 1400microseconds
			LPC_SC->PCONP |= (1 << 6); 					//PWM1 power/clock control bit

			//Peripheral clock selection for PWM1 12 and 13 bit
			LPC_SC->PCLKSEL0 &= ~(3 <<12); 				// Clear clock Bits
			LPC_SC->PCLKSEL0 |=  (1 << 12); 			// CCLK / 1

			//Enable Counters,PWM module
			LPC_PWM1->MCR |= (1<<1);					//Reset on PWMMR0, reset TC if it matches MR0
			LPC_PWM1->MR0 =   msTcMax;					//set PWM cycle(Ton+Toff)=100)(1400microseconds), Reset on PWMMR0: the PWMTC will be reset if PWMMR0 matches it.
			LPC_PWM1->TCR = (1<<0) | (1<<3);
			LPC_PWM1->CTCR &= ~(0xF << 0); 				//the TC is incremented when the Prescale Counter matches the Prescale Register

			//using PWM1.2, GPIO PIN - 2.1
			LPC_PINCON->PINSEL4 &= ~(3 <<2); 			// P2.1 clearing to 00 value
			LPC_PINCON->PINSEL4 |= (1<<2); 				// P2.1 setting value to 01 for PWM1.2
			LPC_PWM1->PCR |=(1<<10);					//The PWM2 output enabled.
        	/*-------------------------------------------------------*/

			/*-----------------DC motor initialization---------------*/
			//	IN1/3=0 and IN2/4=0 -> Motor1/2 idle
			//	IN1/3=0 and IN2/4=1 -> Motor1/2 Anti-clock wise direction
			//	IN1/3=1 and IN2/4=0 -> Motor1/2 Clock wise direction
			//	IN1/3=1 and IN2/4=1 -> Motor1/2 idle

			//Enable
			//ENA, ENB pin is connected to the 5V DC to drive the motor

			//	P2.0 -> IN1, IN3;
			//  P2.2 -> IN2, IN4

			LPC_GPIO2->FIODIR |= ((1 << 2)|(1 << 3)|(1 << 4)|(1 << 5));	//Set P2.2, P2.3, P2.4, P2.5 as output pins
			LPC_GPIO1->FIODIR |= ((1 << 0)|(1 << 1)|(1 << 4)|(1 << 8));
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
			//	IN1=1 and IN2=0 -> Motor1 Clock wise direction
			//	IN3=1 and IN4=0 -> Motor2 Clock wise direction
			LPC_GPIO2->FIOSET |=  (1 << 2);	// Set IN1 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 3); // Set IN2 = 0
			LPC_GPIO2->FIOSET |=  (1 << 4); // Set IN3 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 5); // Set IN4 = 0

			LPC_GPIO1->FIOSET |=  (1 << 0);	// Set IN1 = 1
			LPC_GPIO1->FIOCLR |=  (1 << 1); // Set IN2 = 0
			LPC_GPIO1->FIOSET |=  (1 << 4); // Set IN3 = 1
			LPC_GPIO1->FIOCLR |=  (1 << 8); // Set IN4 = 0
			//u0_dbg_printf("\n Clockwise direction ");
		}
		void move_reverse (void)
		{
			//	IN1=0 and IN2=1 -> Motor1 Anti-clock wise direction
			//	IN3=0 and IN4=1 -> Motor2 Anti-clock wise direction
			LPC_GPIO2->FIOCLR |=  (1 << 2);	// Set IN1 = 0
			LPC_GPIO2->FIOSET |=  (1 << 3); // Set IN2 = 1
			LPC_GPIO2->FIOCLR |=  (1 << 4); // Set IN3 = 0
			LPC_GPIO2->FIOSET |=  (1 << 5); // Set IN4 = 1

			LPC_GPIO1->FIOCLR |=  (1 << 0);	// Set IN1 = 0
			LPC_GPIO1->FIOSET |=  (1 << 1); // Set IN2 = 1
			LPC_GPIO1->FIOCLR |=  (1 << 4); // Set IN3 = 0
			LPC_GPIO1->FIOSET |=  (1 << 8); // Set IN4 = 1
			//u0_dbg_printf("\n Anti-clockwise direction ");
		}
		void stop(void)
		{
			//	IN1=0 and IN2=0 -> Motor1 Idle
			// 	IN3=0 and IN4=0 -> Motor2 Idle
			LPC_GPIO2->FIOCLR |=  (1 << 2);	// Set IN1 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 3); // Set IN2 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 4); // Set IN3 = 0
			LPC_GPIO2->FIOCLR |=  (1 << 5); // Set IN4 = 0

			LPC_GPIO1->FIOCLR |=  (1 << 0);	// Set IN1 = 0
			LPC_GPIO1->FIOCLR |=  (1 << 1); // Set IN2 = 0
			LPC_GPIO1->FIOCLR |=  (1 << 4); // Set IN3 = 0
			LPC_GPIO1->FIOCLR |=  (1 << 8); // Set IN4 = 0
			//u0_dbg_printf("\n Stop ");
		}

        bool run(void *p)
        {
        	int left_val=0,right_val=0,front_val=0;

        	xQueueReceive(getSharedObject(left_SensorQueueId),&left_val,portMAX_DELAY);
        	xQueueReceive(getSharedObject(front_SensorQueueId),&front_val,portMAX_DELAY);
        	xQueueReceive(getSharedObject(right_SensorQueueId),&right_val,portMAX_DELAY);

        	//u0_dbg_printf("\n %d %d %d",left_val,front_val,right_val);

        	if(key && ((left_val==0 || left_val>255) || (right_val==0 || right_val>255) || (front_val==0 || front_val>255)))// || (left_val==0 || left_val>255)))
        		sys_reboot();
        	else
        		key = true;

        	right_val += 5;

        	if(xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE)
        	{

					char buff_r_val[4], buff_l_val[4], buff_f_val[4];
					sprintf(buff_r_val, "%i", right_val);
					sprintf(buff_l_val, "%i", left_val);
					sprintf(buff_f_val, "%i", front_val);

					char main_buff[16];
					strcpy(main_buff, buff_r_val);
					strcat(main_buff," , ");
					strcat(main_buff, buff_l_val);
					strcat(main_buff," , ");
					strcat(main_buff, buff_f_val);
					strcat(main_buff,"\n");

					int len =0;
					int i=0;
					while(main_buff[i] != '\n' )
					{
						len++;
						i++;
					}
					len=len+2;
					Storage::write("1:sd_card.txt", main_buff, len, offset);
					offset=offset+len;
					xSemaphoreGive(sd_card_mutex);
			}

			//---------------------- SERVO MOTOR LOGIC  -------------------//
			if((right_val<5 && left_val<5) || (right_val == left_val))
			{
				// keep servo position in center
				set(72.5);
			}
			else
			{
				double_t deflection = right_val - left_val;
				int32_t total = right_val + left_val;

				deflection = ((deflection/total)*100)/2;

				// add this value to center servo value to get the new value //
				set(72.5+deflection);
			}

			//---------------------- DC MOTOR LOGIC -----------------------//

			if(front_val<15)
			{
				stop();
				set(72.5);
			}
			else
			{
				move_forward();
			}
            return true;
      }
};

class Print_Latitude_and_Longitude : public scheduler_task
{
public:
	Print_Latitude_and_Longitude(uint8_t priority) : scheduler_task("printing_gps", 2048, priority)
	{

	}

	bool run(void *p)
	{
		string gps_data="\n";static int writing_temperature_accelerometer=0;
		string temp_degree_latitude="", temp_decimal_latitude="",temp_decimal_longitude="",temp_degree_longitude="";
		char direction=' ';
		char latitude_and_longitude_string[25];
		float latitude=0, longitude=0;
		int start_gps=0,comma=0,once=0;
		uint32_t count=0;

		if(xSemaphoreTake(gps_semaphore,portMAX_DELAY))
		{
			if(UART2Buffer[43]=='1')
			{
			for(int i=0;i<77;i++)
			{
				if(UART2Buffer[i]=='$')
				{
					start_gps=1;
				}
				if(start_gps==1&&UART2Buffer[i]==',')
				{
					comma++;
				}
				if(comma==2&&once==0)
				{
					count=0;
					gps_data="\nLatitude: ";
					for(int j=i+1;count<11;j++)
					{
						if(j==77)
						{
							j=0;
						}
						gps_data+=UART2Buffer[j];
						if(count>1&&count<9)
						{
							temp_decimal_latitude+=UART2Buffer[j];
						}
						if(count<9)
						{
							temp_degree_latitude+=UART2Buffer[j];
						}
						if(count==10)
						{
							direction= UART2Buffer[j];
						}
						count++;
					}
					const char *temp_degree_latitude_cstr=temp_degree_latitude.c_str();
					const char *temp_decimal_latitude_cstr=temp_decimal_latitude.c_str();
					float value_degree_latitude=strtof(temp_degree_latitude_cstr,NULL);
					float value_decimal_latitude=strtof(temp_decimal_latitude_cstr,NULL);
					int degrees_latitude=value_degree_latitude/100;
					float decimal_latitude=value_decimal_latitude/60;
					if(direction=='N')
					{
						latitude=0+degrees_latitude+decimal_latitude;
					}
					else if(direction=='S')
					{
						latitude=0-(degrees_latitude+decimal_latitude);
					}
					once=1;
				}
				if(comma==4)
				{
					count=0;
					//printf(" Longitude: ");
					gps_data+=" Longitude: ";
					for(int j=i+1;count<12;j++)
					{
						if(j==77)
						{
							j=0;
						}
						if(count>2 && count<10)
						{
							temp_decimal_longitude+=UART2Buffer[j];
						}
						if(count<10)
						{
							temp_degree_longitude+=UART2Buffer[j];
						}
						if(count==11)
						{
							direction= UART2Buffer[j];
						}
						//printf("%c",UART2Buffer[j]);
						gps_data+=UART2Buffer[j];
						count++;
					}
					const char *temp_degree_longitude_cstr=temp_degree_longitude.c_str();
					const char *temp_decimal_longitude_cstr=temp_decimal_longitude.c_str();
					float value_degree_longitude=strtof(temp_degree_longitude_cstr,NULL);
					float value_decimal_longitude=strtof(temp_decimal_longitude_cstr,NULL);
					int degrees_longitude=value_degree_longitude/100;
					float decimal_longitude=value_decimal_longitude/60;
					if(direction=='E')
					{
						longitude=0+degrees_longitude+decimal_longitude;
					}
					else if(direction=='W')
					{
						longitude=0-(degrees_longitude+decimal_longitude);
					}
					sprintf(latitude_and_longitude_string,"$L%f,%f,\n",latitude,longitude);
					printf("\n%s\n", latitude_and_longitude_string);
					writable = new char[gps_data.size() + 1];
					copy(gps_data.begin(), gps_data.end(),writable);
					writable[gps_data.size()] = '\0'; // don't forget the terminating 0
					printf("\n");
					for(i=0;i<46;i++)
					{
						printf("%c",writable[i]);
					}
					printf("\n");
					start=0;
					comma=0;
					once=0;
					send=latitude_and_longitude_string;
					xSemaphoreGive(sending_to_queue);
				}

			}
			}
			else
			{
				writable = new char [15];
				writable="Not available\n";
				for(unsigned int i=0;i<15;i++)
				{
					printf("%c",writable[i]);
				}
				send=writable;
				xSemaphoreGive(sending_to_queue);

			}
			if(xSemaphoreTake(sd_card_mutex, portMAX_DELAY))
			{
				int len =0;
				int i=0;
				while(send[i] != '\0' )
				{
					len++;
					i++;
				}
				Storage::write("1:sd_card.txt", send, len, offset);
				offset=offset+len;
				if(writing_temperature_accelerometer%4==0)
				{
					char temperature[50];
					int temp=TS.getFarenheit();
					int acc_X=AS.getX();
					int acc_Y=AS.getY();
					int acc_Z=AS.getZ();
					printf("%d\n",temp);
					sprintf(temperature, "Temperature: %d\n acc_X: %d, acc_Y: %d, acc_Z: %d\n", temp,acc_X,acc_Y,acc_Z);
					len=0;
					i=0;
					while(temperature[i]!= '\0')
					{
						i++;
						len++;
					}
					writing_temperature_accelerometer=0;
					Storage::write("1:sd_card.txt",temperature, len, offset);
					offset=offset+len;
				}
				xSemaphoreGive(sd_card_mutex);
			}
		}
		writing_temperature_accelerometer++;
	return true;
	}

};

class uart_blue:public scheduler_task
{
	public:
	uart_blue(uint8_t priority):scheduler_task("send",1024,priority)
	{
	}

	bool init(void)
	 {
		//Stop the sd_card sending task
		scheduler_task *compute = scheduler_task::getTaskPtrByName("sd_read");
		vTaskSuspend(compute->getTaskHandle());

		//u0_dbg_printf("Step2\n");
		return true;
	}

	bool run(void *p)
	{
		static bool flag1 = true;
		//u0_dbg_printf("step 3\n");
		//vTaskDelay(1000);
		if(start == '0')
		{
			if(flag1 ==true)
			{
				scheduler_task *sonar_ptr = scheduler_task::getTaskPtrByName("sonar");
				vTaskSuspend(sonar_ptr->getTaskHandle());
				scheduler_task *printing_ptr = scheduler_task::getTaskPtrByName("printing_gps");
				vTaskSuspend(printing_ptr->getTaskHandle());
				scheduler_task *motor_ptr = scheduler_task::getTaskPtrByName("Motors");
				vTaskSuspend(motor_ptr->getTaskHandle());

				LPC_GPIO2->FIOCLR |=  (1 << 2);	// Set IN1 = 0
				LPC_GPIO2->FIOCLR |=  (1 << 3); // Set IN2 = 0
				LPC_GPIO2->FIOCLR |=  (1 << 4); // Set IN3 = 0
				LPC_GPIO2->FIOCLR |=  (1 << 5); // Set IN4 = 0
				char chk = ' ';
				//Add a char to end of string

				scheduler_task *sd_ptr = scheduler_task::getTaskPtrByName("sd_read");
				vTaskResume(sd_ptr->getTaskHandle());
				flag1 = false;
				start = '1';

			}
		}
		return true;
	}
};

Uart3& u3 = Uart3::getInstance();

class read_sdcard:public scheduler_task
{

    public:
    read_sdcard(uint8_t priority):scheduler_task("sd_read",2048,priority)
    {
    }

    void uart3_tx(char u_tx)
    {
        //Check for transmit set bit
        uint8_t chk_tx_bit = 1<<6;
        //Place character value in transmit register
        LPC_UART3->THR = u_tx;
        while(!(LPC_UART3->LSR & chk_tx_bit));
    }

    bool run(void *p)
    {
        static int offset =0;
        static int count = 10;

       if(file == 'F')
       {
           if(xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE)
           {
              char chk = ' ';
              do
              {
                  Storage::read("1:gps.txt",&chk,sizeof(chk),offset++);
              }while(chk != 'L');

              char data[25] = { 0 };
              Storage::read("1:gps.txt", data, sizeof(data)-1, --offset);
              uint8_t i =0;
              u0_dbg_printf("%s\n",data);

             while(data[i] != '\0')
              {
                  uart3_tx(data[i]);
                  i++;
              }

             if(data[23] == 'S')
             {
                vTaskDelay(1000);
                 sys_reboot();
             }
              offset  += 25;
              vTaskDelay(1000);
              xSemaphoreGive(sd_card_mutex);
          }
      }
       else
       {
           if(xSemaphoreTake(sd_card_mutex, portMAX_DELAY) == pdTRUE)
           {

              char chk = ' ';
              do
              {
                  FRESULT status = FR_INT_ERR;
                  FIL file;
                  unsigned int bytesRead = 0;

                  // Open Existing file
                  if (FR_OK == (status = f_open(&file, "1:sd_card.txt", FA_OPEN_EXISTING | FA_READ)))
                  {
                      if(offset)
                      {
                          f_lseek(&file, offset);
                      }
                      status = f_read(&file, &chk, 1, &bytesRead);
                      offset++;
                      if( f_eof(&file))
                          sys_reboot();
                      f_close(&file);
                  }

              }while(chk != 'L');

              char data[24] = { 0 };
              Storage::read("1:sd_card.txt", data, sizeof(data)-1, --offset);
              uint8_t i =0;
              u0_dbg_printf("%s\n",data);

             while(data[i] != '\0')
              {
                  uart3_tx(data[i]);
                  i++;
              }

             if(data[22] == 'S')
             {
                vTaskDelay(1000);
                 sys_reboot();
             }
              offset  += 24;
              vTaskDelay(1000);
              xSemaphoreGive(sd_card_mutex);
          }
       }
        return true;
    }
};

void initialize_uart3()
{
		// UART3 --> P4.28 TXD3;    P4.29 RXD3
		BIT(LPC_PINCON->PINSEL9).b25_24 = 3; //Set bits 25:24 of PINSEL9 to 11 for TX3
		BIT(LPC_PINCON->PINSEL9).b27_26 = 3; //Set bits 27:26 of PINSEL9 to 11 for RX3

		// Power Enable
		LPC_SC->PCONP |= (1 << 25); // Power Enable - UART3

		//Clock selection
		BIT(LPC_SC->PCLKSEL1).b19_18 = 1; // Set CLK for UART3 (CLK = 01)

		// Set Baud rate
		//to access register DLL and DLM to set baud rate, enable DLAB in LCR
		LPC_UART3->LCR = (1 << 7); // Set DLAB in register LCR for UART3

		const uint32_t baud_rate = 9600;
		const uint16_t div16 = sys_get_cpu_clock()/(16 * baud_rate);
		LPC_UART3->DLM = (div16 >> 8); // div/256
		LPC_UART3->DLL = (div16 >> 0); // div%256

		// set 8-bit word-length select i.e. set to 11
		BIT(LPC_UART3->LCR).b1_0 = 3; // UART3
		BIT(LPC_UART3->LCR).b7 = 0; //resetting DLAB = 0

		//  FIFO Enable and FIFO Rx Reset in FIFO Control Register (FCR bit 0 & 1)
		BIT(LPC_UART3->FCR).b0 = 1;

		// Enable UART3 Interrupt Enable register
		BIT(LPC_UART3->IER).b0 = 1; //RBR Interrupt Enable
		BIT(LPC_UART3->IER).b2 = 1; //RX Line Status Interrupt Enable

		NVIC_EnableIRQ(UART3_IRQn);
}

void initialization()
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
	vSemaphoreCreateBinary(main_sem);
	vSemaphoreCreateBinary(stop_sem);
	sd_card_mutex = xSemaphoreCreateMutex();


	LPC_PINCON->PINSEL2 |= 0<<2;
	LPC_GPIO1->FIODIR |= 1<<0;

	// RIT drivers
	LPC_SC->PCONP |= 1<<16;             // Power Control for Peripherals register: power up RIT clock
	LPC_SC->PCLKSEL1 |= (1<<26);  		// Peripheral clock selection: divide clock by 8 (run RIT clock by 12MHz)
	LPC_RIT->RICOUNTER = 0;             // set counter to zero
	LPC_RIT->RICOMPVAL = 0x00FFFFFF;    // interrupt tick every second
	LPC_RIT->RICTRL |= 1<<1;            // clear timer when counter reaches value


	// UART2 driver for GPS module
	LPC_SC->PCONP|=(1<<24);                                      	//Activate UART2 in PCONP register
	LPC_SC->PCLKSEL1 &=~(3<<16);                                    //Reset the clock
	LPC_SC->PCLKSEL1 |= (1<<16);                                    //Set clock as 48MHz
	LPC_UART2->LCR = (1<<7);                                        //Set DLAB bit
	LPC_UART2->LCR |= (3<<0);                                       //Set 8-bit character length
	LPC_UART2->DLM=0;                                               //Set divisor latch MSB as 0
	LPC_UART2->DLL = (sys_get_cpu_clock() / (16 * 38400))+0.875;    //set divisor latch LSB
	LPC_UART2->LCR &= ~(1<<7);                                      //Reset the DLAB bit
	LPC_UART2->FCR |=(1<<0);                                        //Enable the FIFO
	LPC_PINCON->PINSEL4 &=~ (3<<16)|(3<<18);                        //Set P2.8 as TxD2
	LPC_PINCON->PINSEL4 |= (2<<16)|(2<<18);                         //Set P2.9 as RxD2
	LPC_UART2->FCR|=(1<<1)|(1<<2);                                  //Clear all bytes in Rx and Tx FIFO
	NVIC_EnableIRQ(UART2_IRQn);                                     //Enable IRQ
	LPC_UART2->IER |= (1<<0)|(1<<1)|(1<<2);                         //Enable RBR, THRE, and Rx Line Status interrupts

	gps_write_queue = xQueueCreate(1, sizeof(void *));
	vSemaphoreCreateBinary( gps_semaphore);
	xSemaphoreTake( gps_semaphore, 0);
	vSemaphoreCreateBinary(sending_to_queue);
	xSemaphoreTake(sending_to_queue, 0);

	// UART3 driver for Bluetooth module
	initialize_uart3();

}

int main(void)
{

	//Set P2.2, P2.3, P2.4, P2.5 as output pins
	LPC_GPIO2->FIODIR |= ((1 << 2)|(1 << 3)|(1 << 4)|(1 << 5));

	//Signal to Stop motor for preventing it from malfunctioning
	LPC_GPIO2->FIOCLR |=  (1 << 2);	// Set IN1 = 0
	LPC_GPIO2->FIOCLR |=  (1 << 3); // Set IN2 = 0
	LPC_GPIO2->FIOCLR |=  (1 << 4); // Set IN3 = 0
	LPC_GPIO2->FIOCLR |=  (1 << 5); // Set IN4 = 0

	/* All necessary driver settings are done here like
	 * Semaphores      - for triggering and sensing sensor data & SD card
	 * RIT interrupt   - for periodic sensor semaphore releasing,
	 * UART2 interrupt - for GPS module
	 */
	initialization();

	/*
	 * It is waiting for start signal from mobile application and checking is SD card is inserted or not.
	 * 		variables			Values
	 * 		start				' ' (initial)
	 * 							0	(stop signal)
	 * 							1	(start signal)
	 *
	 * 		sd_initialize()		RES_OK = 0,		0: Successful
	 *							RES_ERROR,		 1: R/W Error
	 *							RES_WRPRT,		 2: Write Protected
	 *							RES_NOTRDY,		 3: Not Ready
	 *							RES_PARERR		 4: Invalid Parameter
	 */
	while(start == ' ' && sd_initialize()==RES_NOTRDY);

	// It is ready to go, so starting RIT timer.
	LPC_RIT->RICTRL |= 1<<3;            // enable RIT timer


	scheduler_add_task(new sonar(10));

	scheduler_add_task(new Rightsensor(8));
	scheduler_add_task(new Frontsensor(7));
	scheduler_add_task(new Leftsensor(5));

	scheduler_add_task(new motor(4));
	scheduler_add_task(new Print_Latitude_and_Longitude(3));

	scheduler_add_task(new uart_blue(PRIORITY_LOW));
	scheduler_add_task(new read_sdcard(PRIORITY_LOW));


	// Enabling interrupts for RIT and GPIO(sensors).
	NVIC_EnableIRQ(RIT_IRQn);
	NVIC_EnableIRQ(EINT3_IRQn);

    scheduler_start(); ///< This shouldn't return
    return -1;
}
