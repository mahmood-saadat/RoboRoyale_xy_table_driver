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

#ifndef __TASKS_MOTORDRIVER_H__
#define __TASKS_MOTORDRIVER_H__

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


class MotorDriver{
public:
	typedef enum MOTOR_STATE{
		IDLE,
		ACCELERATION,
		DECELERATION,
		CONSTANT_SPEED
	}motor_state_t;
	//------------------------------ Public static variables  -------------------------------

private:
	//------------------------------ Private static variables  ------------------------------


	//------------------------------ Private variables  -------------------------------------
	TaskHandle_t 			mainTask;
	uint32_t				stackWatermarkPrintLastMillis = 0;
	bool					is_command_new = false;
	uint32_t				x_pulse_frequency = 1000;
//	uint32_t				x_pulse_last_frequency_command = 1000;
	uint32_t				y_pulse_frequency = 1000;
//	uint32_t				y_pulse_last_frequency_command = 1000;
	bool					is_acceleration_command = false;
	motor_state_t			x_motor_state = IDLE;
	motor_state_t			y_motor_state = IDLE;

	float					x_command_speed	= 5.0f; // mm/s - This is set from outside of library. It can be negative. The command from outside of this module is always
	// positive; The sign must be set according to the state and location command.
	float					x_current_speed	= 0.0f; // mm/s - The current speed
	float					x_target_speed = 0.0f; // mm/s - The target speed depending on the state
	float					x_command_acceleration = 1.0f;	//mm/s^2
	float					x_current_acceleration = 1.0f;
//	float					x_integral	= 0.0f;
//	float 					x_previos_err = 0.0f;

	float					y_command_speed	= 5.0f;
	float					y_current_speed	= 0.0f;
	float					y_target_speed = 0.0f;
	float					y_command_acceleration = 1.0f;
	float					y_current_acceleration = 1.0f;
//	float					y_integral	= 0.0f;
//	float 					y_previos_err = 0.0f;


	bool					is_ready		= false;

public:
	//------------------------------ Public functions  -------------------------------------
	MotorDriver();
	void begin();
	bool IsReady();
//	void SetX(float x);
//	void SetY(float y);
	void SetXY(float x, float y);
	void SetXY(float x, float y, float x_speed, float y_speed);
	void SetXY(float x, float y, float x_speed, float y_speed, float x_acceleration, float y_acceleration);
	void GetXY(float * x, float * y);
	void SetXYCurrentPosition(float x, float y);
	motor_state_t GetXState();
	motor_state_t GetYState();

protected:

private:
	//------------------------------ Private functions  ------------------------------------
	static TaskFunction_t 	TaskStart(void * pvParameters);
	void 					MainTask();
	void					PrintStackWatermark();
	void					TimerInit();
	void 					StartPulses(uint32_t x_pulse_frequency, uint32_t y_pulse_frequency);
	void 					StartPulses(float x_speed, float y_speed);
	void 					StopPulses();

	uint32_t				XSpeedToFrequency(float speed);
	uint32_t				YSpeedToFrequency(float speed);

	//float					CalculateSpeed(float current_speed, float acceleration, float dt);
//	void 					CalculateSpeed(
//			float location_command, float current_location, float & integral,
//			float & prev_error, float & current_speed, float accel);
	void					UpdatePulseFrequency(void);
	void 					UpdateXState(void);
	void 					UpdateYState(void);

	float					GetXrem();
	float					GetYrem();
};

extern MotorDriver motorDriver;
#endif
