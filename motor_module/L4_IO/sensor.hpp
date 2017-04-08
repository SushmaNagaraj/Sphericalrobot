/*
 * sensor.h
 *
 *  Created on: Oct 20, 2016
 *      Author: arthurnguyen
 */
#ifndef L5_APPLICATION_SENSOR_HPP_
#define L5_APPLICATION_SENSOR_HPP_
#include "tasks.hpp"
#include "gpio.hpp"
#include "ssp1.h"
#include "io.hpp"
#include <stdio.h>
extern int leftDistance;
extern int rightDistance;
extern int frontDistance;
extern int backDistance;
//Start and stop signal
extern bool startProcess;
//Time
extern int frontStart;
extern int frontStop;
extern int backStart;
extern int backStop;
extern int leftStart;
extern int leftStop;
extern int rightStart;
extern int rightStop;
void frontstartTimer(void);
void frontstopTimer(void);
void backstartTimer(void);
void backstopTimer(void);
void leftstartTimer(void);
void leftstopTimer(void);
void rightstartTimer(void);
void rightstopTimer(void);
#endif /* L5_APPLICATION_SENSOR_HPP_ */
