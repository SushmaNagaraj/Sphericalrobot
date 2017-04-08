/*
 * sensor.cpp
 *
 *  Created on: Oct 20, 2016
 *      Author: arthurnguyen
 */
#include "sensor.hpp"
#include "printf_lib.h"
#define RED 0x80
#define YELLOW 0xC0
#define GREEN 0x40
//COM_BRIDGE_RESET_t com_can_msg_t;
int frontStart = 0;
int frontStop = 0;
int backStart = 0;
int backStop = 0;
int leftStart = 0;
int leftStop = 0;
int rightStart = 0;
int rightStop = 0;
int frontDistance = 0;
int backDistance = 0;
int leftDistance = 0;
int rightDistance = 0;

void frontstartTimer(void)
{
    frontStart = (int)sys_get_uptime_us();
}
void frontstopTimer(void)
{
   frontStop = (int)sys_get_uptime_us() - frontStart;
   frontDistance = frontStop/147;
   u0_dbg_printf("1: %d\n",frontDistance);
}
void backstartTimer(void)
{
    backStart = (int)sys_get_uptime_us();
}
void backstopTimer(void)
{
    backStop = (int)sys_get_uptime_us() - backStart;
    backDistance = backStop/147;
    u0_dbg_printf("2: %d\n",backDistance);
}
void leftstartTimer(void)
{
    leftStart = (int)sys_get_uptime_us();
}
void leftstopTimer(void)
{
    leftStop = (int)sys_get_uptime_us() - leftStart;
    leftDistance = leftStop/147;
    u0_dbg_printf("3: %d\n",leftDistance);
}
void rightstartTimer(void)
{
    rightStart = (int)sys_get_uptime_us();
}
void rightstopTimer(void)
{
    rightStop = (int)sys_get_uptime_us() - rightStart;
    rightDistance = rightStop/147;
    u0_dbg_printf("4: %d\n",rightDistance);
}
