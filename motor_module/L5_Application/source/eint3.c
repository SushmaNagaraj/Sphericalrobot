#include <stdlib.h>
#include<LPC17xx.h>

/*uint32_t p2_rising;
uint32_t p0_rising;

//For timer 0 delay creation
void delay()
{
              LPC_TIM0->TCR = 1;

              while((LPC_TIM0->IR & 1) == 0);

              LPC_TIM0->IR |= 1;

              LPC_TIM0->TCR = 2;

}
//port2 callback function
void port2callback(void){
		for(int i=0;i<13;i++) {
			if(p2_rising & (1<<i))
			u0_dbg_printf("Pin number in port 2 is ----> %d \n",i);
		}
}
//port0 callback function
void port0callback(void){
	for(int i=0;i<32;i++) {
		if(p0_rising & (1<<i))
		u0_dbg_printf("Pin number in port 0 is ----> %d \n",i);
	}
}

//ISR routine for External 3 interrupt
void  ExInt3_IRQHandler(void){
	//Read all the ports' rising and falling isr status
	p2_rising = LPC_GPIOINT->IO2IntStatR;
	p0_rising = LPC_GPIOINT->IO0IntStatR;
	delay();
	if(p2_rising){
		port2callback();
		LPC_GPIOINT->IO2IntClr = 0xFFFFFFFF;
	}
	if(p0_rising){
		port0callback();
		LPC_GPIOINT->IO0IntClr = 0xFFFFFFFF;
	}
}*/
