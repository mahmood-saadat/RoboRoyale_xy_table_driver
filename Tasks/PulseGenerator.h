/*
 * PulseGenerator.h
 *
 *  Created on: Nov 18, 2021
 *      Author: M.Saadat (m.saadat@mail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 *
 *  V1.0.0: Base release
 */

#ifndef __PULSEGENERATOR_H__
#define __PULSEGENERATOR_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


class PulseGenerator{
public:
	//------------------------------ Public static variables  -------------------------------

private:
	//------------------------------ Private static variables  ------------------------------

	//------------------------------ Private variables  -------------------------------------
	TaskHandle_t 			mainTask;
	uint32_t				stackWatermarkPrintLastMillis = 0;
	bool					is_command_new = false;
	uint32_t				x_pulse_frequency = 1000;
	uint32_t				x_pulse_last_frequency_command = 1000;
	uint32_t				y_pulse_frequency = 1000;
	uint32_t				y_pulse_last_frequency_command = 1000;

	bool					is_ready		= false;

public:
	//------------------------------ Public functions  -------------------------------------
	PulseGenerator();
	void begin();
	bool IsReady();
	void SetX(float x);
	void SetY(float y);
	void SetXY(float x, float y);
	void SetXY(float x, float y, float x_speed, float y_speed);
	void GetXY(float * x, float * y);

protected:

private:
	//------------------------------ Private functions  ------------------------------------
	static TaskFunction_t 	TaskStart(void * pvParameters);
	void 					MainTask();
	void					PrintStackWatermark();
	void 					StartPulses();
	void 					StopPulses();
};

extern PulseGenerator pulseGenerator;
#endif
