/*
 * MotorDriver.cpp
 *
 *  Created on: Nov 18, 2021
 *      Author: M.Saadat (m.saadat@mail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 */

#include "MotorDriver.h"

#include <esp_task_wdt.h>
#include <Arduino.h>

#define 	MOTOR_DRIVER_WDT_PULSE_GENERATOR_TASK_TIMEOUT 	5000
#define		MOTOR_DRIVER_PULSE_GENERATOR_DEAD_BAND			1
#define		MOTOR_DRIVER_ESP_FREQUENCY						80000000
#define		MOTOR_DRIVER_TIMER_DIVIDER						2

#define		MOTOR_DRIVER_X_PULSE_PIN						14
#define		MOTOR_DRIVER_X_DIRECTION_PIN					13
#define		MOTOR_DRIVER_Y_PULSE_PIN						12
#define		MOTOR_DRIVER_Y_DIRECTION_PIN					5

#define		MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN			4
#define		MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN			2
#define		MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN				16
#define		MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN				15

#define		MOTOR_DRIVER_X_SCREW_PITCH						4.0f	// mm
#define		MOTOR_DRIVER_X_PULSE_PER_REVOLUTION				1000
#define		MOTOR_DRIVER_Y_SCREW_PITCH						4.0f	// mm
#define		MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION				1000

#define		MOTOR_DRIVER_X_COARSE							580.0f//364.0f
#define		MOTOR_DRIVER_Y_COARSE							610.0f//500.0f

#define		ABS(x)  		(x<0)?-x:x


MotorDriver 			motorDriver;

hw_timer_t * x_pulse_generator_timer = NULL;
hw_timer_t * y_pulse_generator_timer = NULL;

float					x_target_location 			= 0.0f;
float					y_target_location 			= 0.0f;

int32_t					x_current_half_pulse_counter			= 0;
int32_t					y_current_half_pulse_counter			= 0;
int32_t					x_target_half_pulse_counter				= 0;
int32_t					y_target_half_pulse_counter				= 0;

bool					is_x_zero_detected						= false;
bool					is_y_zero_detected						= false;
bool					is_x_end_detected						= false;
bool					is_y_end_detected						= false;

/**
 * @brief This ISR will be called on the selected timeouts.
 * This ISR generates the pulses for the X axis stepper motor driver and set the direction pin.
 * In this ISR the states of the limit switches is checked before pulsing the driver.
 *
 */
void IRAM_ATTR onXTimer()
{
	if(x_current_half_pulse_counter < x_target_half_pulse_counter)
	{
		if(digitalRead(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 1);
			digitalWrite(MOTOR_DRIVER_X_PULSE_PIN, !digitalRead(MOTOR_DRIVER_X_PULSE_PIN));
			x_current_half_pulse_counter ++;
		}
		else
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
			is_x_end_detected = true;
		}
	}
	else if(x_current_half_pulse_counter > x_target_half_pulse_counter)
	{
		if(digitalRead(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
			digitalWrite(MOTOR_DRIVER_X_PULSE_PIN, !digitalRead(MOTOR_DRIVER_X_PULSE_PIN));
			x_current_half_pulse_counter --;
		}
		else
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 1);
			x_current_half_pulse_counter = -1500;//0;
			x_target_half_pulse_counter = 0;
			x_target_location = 0.0f;
			is_x_zero_detected = true;
		}
	}

}

/**
 * @brief This ISR will be called on the selected timeouts.
 * This ISR generate the pulses for the Y axis stepper motor driver and set the direction pin.
 * In this ISR the states of the limit switches is checked before pulsing the driver.
 *
 */
void IRAM_ATTR onYTimer()
{

	if(y_current_half_pulse_counter < y_target_half_pulse_counter)
	{
		if(digitalRead(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 1);
			digitalWrite(MOTOR_DRIVER_Y_PULSE_PIN, !digitalRead(MOTOR_DRIVER_Y_PULSE_PIN));
			y_current_half_pulse_counter ++;
		}
		else
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);
			is_y_end_detected = true;
		}
	}
	else if(y_current_half_pulse_counter > y_target_half_pulse_counter)
	{
		if(digitalRead(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);
			digitalWrite(MOTOR_DRIVER_Y_PULSE_PIN, !digitalRead(MOTOR_DRIVER_Y_PULSE_PIN));
			y_current_half_pulse_counter --;
		}
		else
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 1);
			y_current_half_pulse_counter = -1500;//0;
			y_target_half_pulse_counter = 0;
			y_target_location = 0.0f;
			is_y_zero_detected = true;
		}
	}
}

MotorDriver::MotorDriver(){
	mainTask = NULL;
}

/*
 * Initialise the class
 */
void MotorDriver::begin(){
	//Serial.println("[MotorDriver] Setup is started ...");

	pinMode(MOTOR_DRIVER_X_PULSE_PIN, OUTPUT);
	pinMode(MOTOR_DRIVER_X_DIRECTION_PIN, OUTPUT);
	pinMode(MOTOR_DRIVER_Y_PULSE_PIN, OUTPUT);
	pinMode(MOTOR_DRIVER_Y_DIRECTION_PIN, OUTPUT);

	pinMode(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN, INPUT_PULLUP);

	digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
	digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);

	TimerInit();

	xTaskCreate((TaskFunction_t)MotorDriver::TaskStart, "MotorDriverTask", 8192, this, 50, &mainTask);

	//Serial.println("[MotorDriver] Setup is done.");

}

/*
 * Task function needed to be static and in static members, we could not access class members easily,
 * so we created this function and we passed the class as input argument, then we call our main function from here.
 */
TaskFunction_t MotorDriver::TaskStart(void * pvParameters)
{
	MotorDriver* manager;
	esp_task_wdt_init(MOTOR_DRIVER_WDT_PULSE_GENERATOR_TASK_TIMEOUT, true);
	esp_task_wdt_add(NULL);
	manager = (MotorDriver*) pvParameters;
	manager->MainTask();
	// It will never get here. The main task never returns.
	return 0;
}
/*
 * The main loop
 */
void MotorDriver::MainTask()
{
	SetXY(-MOTOR_DRIVER_X_COARSE-5, -MOTOR_DRIVER_Y_COARSE-5, 10.0f, 10.0f);

	while(is_x_zero_detected == false || is_y_zero_detected == false)
	{
		//Serial.printf("[MotorDriver] Waiting for zero!\r\n");
		esp_task_wdt_reset();
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}

	is_ready = true;
	SetXY(MOTOR_DRIVER_X_COARSE, 0, 5.0f, 5.0f);

	while(1){
		esp_task_wdt_reset();


//		//Serial.printf("[MotorDriver] Current: %d, Target: %d\r\n", x_current_half_pulse_counter, x_target_half_pulse_counter);
		PrintStackWatermark();
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

void MotorDriver::PrintStackWatermark()
{
	if(millis() > (stackWatermarkPrintLastMillis + 10000)){
		stackWatermarkPrintLastMillis = millis();
		//Serial.printf("[MotorDriver] Task Stack left: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
		//Serial.printf("[MotorDriver] X Current: %d, Target: %d, freq: %d\r\n", x_current_half_pulse_counter, x_target_half_pulse_counter, x_pulse_frequency);
		//Serial.printf("[MotorDriver] Y Current: %d, Target: %d\r\n", y_current_half_pulse_counter, y_target_half_pulse_counter);
	}
}

bool MotorDriver::IsReady()
{
	return is_ready;
}

void MotorDriver::TimerInit()
{
	x_pulse_generator_timer = timerBegin(0, MOTOR_DRIVER_TIMER_DIVIDER, true);
	timerAttachInterrupt(x_pulse_generator_timer, &onXTimer, true);
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(x_pulse_generator_timer, 40000000, true);
	/* Start an alarm */
	timerAlarmEnable(x_pulse_generator_timer);

	y_pulse_generator_timer = timerBegin(1, MOTOR_DRIVER_TIMER_DIVIDER, true);
	timerAttachInterrupt(y_pulse_generator_timer, &onYTimer, true);
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(y_pulse_generator_timer, 40000000, true);
	/* Start an alarm */
	timerAlarmEnable(y_pulse_generator_timer);
}

void MotorDriver::StartPulses(uint32_t x_pulse_frequency, uint32_t y_pulse_frequency)
{
	uint32_t x_prescaler = 0;
	uint32_t y_prescaler = 0;
	StopPulses();

	if(x_pulse_frequency == 0)
	{
		x_pulse_frequency = 10;
	}
	if(y_pulse_frequency == 0)
	{
		y_pulse_frequency = 10;
	}

	x_prescaler = MOTOR_DRIVER_ESP_FREQUENCY/MOTOR_DRIVER_TIMER_DIVIDER/x_pulse_frequency/2;
	if(0 == x_prescaler)
	{
		x_prescaler = 1;
	}
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(x_pulse_generator_timer, x_prescaler, true);

	y_prescaler = MOTOR_DRIVER_ESP_FREQUENCY/MOTOR_DRIVER_TIMER_DIVIDER/y_pulse_frequency/2;
	if(0 == y_prescaler)
	{
		y_prescaler = 1;
	}
	/* Repeat the alarm (third parameter), set to 1 to alarm every single clock */
	timerAlarmWrite(y_pulse_generator_timer, y_prescaler, true);

}

void MotorDriver::StartPulses(float x_speed, float y_speed)
{
	StartPulses(XSpeedToFrequency(x_speed), YSpeedToFrequency(y_speed));
}

void MotorDriver::StartPulses()
{
	StartPulses(x_pulse_frequency, y_pulse_frequency);
}

void MotorDriver::StopPulses()
{

}

void MotorDriver::SetX(float x)
{
	if(x < MOTOR_DRIVER_X_COARSE)
	{
		x_target_location = x;
	}
	else
	{
		x_target_location = MOTOR_DRIVER_X_COARSE;
	}
	x_target_half_pulse_counter = x_target_location * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;

	if(x_pulse_frequency != x_pulse_last_frequency_command)
	{
		x_pulse_last_frequency_command = x_pulse_frequency;
		StartPulses();
	}

	is_command_new = true;
}

void MotorDriver::SetY(float y)
{
	if(y < MOTOR_DRIVER_Y_COARSE)
	{
		y_target_location = y;
	}
	else
	{
		y_target_location = MOTOR_DRIVER_Y_COARSE;
	}

	y_target_half_pulse_counter = y_target_location * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;

	if(y_pulse_frequency != y_pulse_last_frequency_command)
	{
		y_pulse_last_frequency_command = y_pulse_frequency;
		StartPulses();
	}
	is_command_new = true;
}

void MotorDriver::SetXY(float x, float y)
{
	if(x < MOTOR_DRIVER_X_COARSE)
	{
		x_target_location = x;
	}
	else
	{
		x_target_location = MOTOR_DRIVER_X_COARSE;
	}
	if(y < MOTOR_DRIVER_Y_COARSE)
	{
		y_target_location = y;
	}
	else
	{
		y_target_location = MOTOR_DRIVER_Y_COARSE;
	}
	x_target_half_pulse_counter = x_target_location * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
	y_target_half_pulse_counter = y_target_location * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;

	if(x_pulse_frequency != x_pulse_last_frequency_command)
	{
		x_pulse_last_frequency_command = x_pulse_frequency;
		StartPulses();
	}
	if(y_pulse_frequency != y_pulse_last_frequency_command)
	{
		y_pulse_last_frequency_command = y_pulse_frequency;
		StartPulses();
	}

	is_command_new = true;
}

/**
 * speed is mm/s
 */
void MotorDriver::SetXY(float x, float y, float x_speed, float y_speed)
{
	if(x < MOTOR_DRIVER_X_COARSE)
	{
		x_target_location = x;
	}
	else
	{
		x_target_location = MOTOR_DRIVER_X_COARSE;
	}
	if(y < MOTOR_DRIVER_Y_COARSE)
	{
		y_target_location = y;
	}
	else
	{
		y_target_location = MOTOR_DRIVER_Y_COARSE;
	}
	x_target_half_pulse_counter = x_target_location * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
	y_target_half_pulse_counter = y_target_location * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;
	x_pulse_frequency = x_speed * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH;
	if(x_pulse_frequency < 10 || x_pulse_frequency > 20000)
	{
		x_pulse_frequency = 1000;
	}

	y_pulse_frequency = y_speed * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH;
	if(y_pulse_frequency < 10 || y_pulse_frequency > 20000)
	{
		y_pulse_frequency = 1000;
	}

	if(x_pulse_frequency != x_pulse_last_frequency_command)
	{
		x_pulse_last_frequency_command = x_pulse_frequency;
		StartPulses();
	}
	if(y_pulse_frequency != y_pulse_last_frequency_command)
	{
		y_pulse_last_frequency_command = y_pulse_frequency;
		StartPulses();
	}

	is_command_new = true;
}

void MotorDriver::GetXY(float * x, float * y)
{
	*x = x_current_half_pulse_counter * MOTOR_DRIVER_X_SCREW_PITCH / 2 / MOTOR_DRIVER_X_PULSE_PER_REVOLUTION;
	*y = y_current_half_pulse_counter * MOTOR_DRIVER_Y_SCREW_PITCH / 2 / MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION;
}

void MotorDriver::SetXYCurrentPosition(float x, float y)
{
	x_current_half_pulse_counter = x * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
	y_current_half_pulse_counter = y * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;
}

uint32_t MotorDriver::XSpeedToFrequency(float speed)
{
	uint32_t frequency = abs(speed) * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH;
	if(frequency < 10)
	{
		frequency = 10;
	}
	if(frequency > 20000)
	{
		frequency = 20000;
	}
	return (frequency);
}

uint32_t MotorDriver::YSpeedToFrequency(float speed)
{
	uint32_t frequency = abs(speed) * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH;
	if(frequency < 10)
	{
		frequency = 10;
	}
	if(frequency > 20000)
	{
		frequency = 20000;
	}
	return (frequency);
}
