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

#include "../Functions/debug.h"

#include <esp_task_wdt.h>
#include <Arduino.h>

#define		MOTOR_DRIVER_LOOP_MS							1
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

#define		MOTOR_DRIVER_X_DEFAULT_ACCELERAYION				10.0f// mm/s^2
#define		MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION				10.0f// mm/s^2

#define		MOTOR_DRIVER_X_MIN_SPEED						0.01f// mm/s
#define		MOTOR_DRIVER_Y_MIN_SPEED						0.01f// mm/s
#define		MOTOR_DRIVER_X_MAX_SPEED						40.0f// mm/s
#define		MOTOR_DRIVER_Y_MAX_SPEED						40.0f// mm/s

#define		MOTOR_DRIVER_X_ERR_MAX							1000.0f
#define		MOTOR_DRIVER_Y_ERR_MAX							1000.0f

#define		MOTOR_DRIVER_LOCATION_DEADBAND					0.05f
#define		MOTOR_DRIVER_LOCATION_REM_MIN					MOTOR_DRIVER_LOCATION_DEADBAND


// PID Controller Constants
#define 	KP 												0.001 		// Proportional Gain
#define 	KI 												0.0001 		// Integral Gain
#define 	KD 												0.00001 		// Derivative Gain

#define		ABS(x)  		(x<0)?-x:x
#define		SIGN(x)			((x>=0)?(int)1:(int)-1)


MotorDriver 			motorDriver;

hw_timer_t * x_pulse_generator_timer = NULL;
hw_timer_t * y_pulse_generator_timer = NULL;

// These are used in isr. We can not use the class members unless we make it public or make static setter and getters
float					x_command_location 			= 0.0f;
float					y_command_location 			= 0.0f;
bool					is_x_speed_positive = true;
bool					is_y_speed_positive = true;

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
	//if(x_current_half_pulse_counter < x_target_half_pulse_counter)
	if(is_x_speed_positive && ((x_current_half_pulse_counter != x_target_half_pulse_counter) ||
			(motorDriver.GetXState() != MotorDriver::IDLE)))
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
	//else if(x_current_half_pulse_counter > x_target_half_pulse_counter)
	else if(!is_x_speed_positive && ((x_current_half_pulse_counter != x_target_half_pulse_counter) ||
			(motorDriver.GetXState() != MotorDriver::IDLE)))
	{
		if(digitalRead(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
			digitalWrite(MOTOR_DRIVER_X_PULSE_PIN, !digitalRead(MOTOR_DRIVER_X_PULSE_PIN));
			x_current_half_pulse_counter --;
		}
		else
		{
//			digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 1);
//			x_current_half_pulse_counter = -1500;//0;
//			x_target_half_pulse_counter = 0;
//			x_command_location = 0.0f;
//			is_x_zero_detected = true;
		}
	}
	if(digitalRead(MOTOR_DRIVER_X_START_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 1);
		motorDriver.LimitXStartEvent();
		x_current_half_pulse_counter = -1500;//0;
		x_target_half_pulse_counter = 0;
		x_command_location = 0.0f;
		is_x_zero_detected = true;
	}
	if(digitalRead(MOTOR_DRIVER_X_END_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_X_DIRECTION_PIN, 0);
		motorDriver.LimitXEndEvent();
		is_x_end_detected = true;
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

	//if(y_current_half_pulse_counter < y_target_half_pulse_counter)
	if(is_y_speed_positive && ((y_current_half_pulse_counter != y_target_half_pulse_counter) ||
			(motorDriver.GetYState() != MotorDriver::IDLE)))
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
	//else if(y_current_half_pulse_counter > y_target_half_pulse_counter)
	else if(!is_y_speed_positive && ((y_current_half_pulse_counter != y_target_half_pulse_counter) ||
			(motorDriver.GetYState() != MotorDriver::IDLE)))
	{
		if(digitalRead(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN) != 1)
		{
			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);
			digitalWrite(MOTOR_DRIVER_Y_PULSE_PIN, !digitalRead(MOTOR_DRIVER_Y_PULSE_PIN));
			y_current_half_pulse_counter --;
		}
		else
		{
//			digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 1);
//			y_current_half_pulse_counter = -1500;//0;
//			y_target_half_pulse_counter = 0;
//			y_command_location = 0.0f;
//			is_y_zero_detected = true;
		}
	}
	if(digitalRead(MOTOR_DRIVER_Y_START_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 1);
		motorDriver.LimitYStartEvent();
		y_current_half_pulse_counter = -1500;//0;
		y_target_half_pulse_counter = 0;
		y_command_location = 0.0f;
		is_y_zero_detected = true;
	}
	if(digitalRead(MOTOR_DRIVER_Y_END_LIMIT_SWITCH_PIN) == 1)
	{
		digitalWrite(MOTOR_DRIVER_Y_DIRECTION_PIN, 0);
		motorDriver.LimitYEndEvent();
		is_y_zero_detected = true;
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
	uint32_t counter = 0;
	SetXY(-MOTOR_DRIVER_X_COARSE-5, -MOTOR_DRIVER_Y_COARSE-5, 10.0f, 10.0f);

	while(is_x_zero_detected == false || is_y_zero_detected == false)
	{
		UpdatePulseFrequency();
		//Serial.printf("[MotorDriver] Waiting for zero!\r\n");
		esp_task_wdt_reset();
		vTaskDelay(MOTOR_DRIVER_LOOP_MS/portTICK_PERIOD_MS);
	}

	is_ready = true;
	SetXY(MOTOR_DRIVER_X_COARSE, 0, 5.0f, 5.0f);

	while(1){
		esp_task_wdt_reset();

		if(counter>2000)
		{
			SetXY(0.0f, 0.0f, 20.0f, 20.0f);
		}
		else if(counter>1700)
		{
			SetXY(150, 150, 20.0f, 20.0f);
		}
		else if(counter>1000)
		{
			SetXY(10, 10, 5.0f, 5.0f);
		}
		else if(counter>50)
		{
			SetXY(100, 100, 25.0f, 25.0f);
		}
		counter ++;
		if(counter > 2300)
		{
			counter = 0;
		}

		UpdatePulseFrequency();
//		//Serial.printf("[MotorDriver] Current: %d, Target: %d\r\n", x_current_half_pulse_counter, x_target_half_pulse_counter);
		PrintStackWatermark();
		vTaskDelay(MOTOR_DRIVER_LOOP_MS/portTICK_PERIOD_MS);
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

/**
 * This method should be called periodically to update the speed
 */
void MotorDriver::UpdatePulseFrequency()
{

#if 0
//	float x_next_speed = 0.0f;// = CalculateSpeed(x_current_speed, x_target_acceleration, MOTOR_DRIVER_LOOP_DT_MS);
//	float y_next_speed = 0.0f;// = CalculateSpeed(y_current_speed, y_target_acceleration, MOTOR_DRIVER_LOOP_DT_MS);
//
	float x_rem = 0.0f;// = 0.5f*(x_current_speed - x_target_speed)*(x_current_speed - x_target_speed)/x_target_acceleration;
	float y_rem = 0.0f;// = 0.5f*(y_current_speed - y_target_speed)*(y_current_speed - y_target_speed)/y_target_acceleration;

	int8_t x_accel_sign = 1;
	int8_t y_accel_sign = 1;

	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);



	if(abs(x_current_speed) < (x_command_speed) && abs(x_command_speed) > MOTOR_DRIVER_X_MIN_SPEED)
	{
		x_rem = 0.95f*0.5f*(abs(x_current_speed) - x_command_speed)*(abs(x_current_speed) - x_command_speed)/x_command_acceleration;
	}
	else if(abs(x_command_speed) > MOTOR_DRIVER_X_MIN_SPEED)
	{
		x_rem = 0.95f*0.5f*(x_current_speed)*(x_current_speed)/x_command_acceleration;
	}
	if(abs(y_current_speed) < (y_command_speed) && abs(y_command_speed) > MOTOR_DRIVER_Y_MIN_SPEED && y_motor_state == ACCELERATION)
	{
		y_rem = 0.95f*0.5f*(abs(y_current_speed) - y_command_speed)*(abs(y_current_speed) - y_command_speed)/y_command_acceleration;
	}
	else if(abs(y_command_speed) > MOTOR_DRIVER_Y_MIN_SPEED && y_motor_state != ACCELERATION)
	{
		y_rem = 0.95f*0.5f*(y_current_speed)*(y_current_speed)/y_command_acceleration;
	}
//
//	if(abs(x_target_location - x_cur) > x_rem)
//	{
//		x_next_speed = CalculateSpeed(x_current_speed, x_target_acceleration, MOTOR_DRIVER_LOOP_DT_MS);
//		if(x_current_speed < x_target_speed)
//		{
//			x_current_speed = x_next_speed;
//		}
//	}
//	else
//	{
//		x_next_speed = CalculateSpeed(x_current_speed, -1.0f * x_target_acceleration, MOTOR_DRIVER_LOOP_DT_MS);
//		if(x_next_speed > MOTOR_DRIVER_X_MIN_SPEED)
//		{
//			x_current_speed = x_next_speed;
//		}
//	}
//
//	if(abs(y_target_location - y_cur) > y_rem)
//	{
//		y_next_speed = CalculateSpeed(y_current_speed, y_target_acceleration, MOTOR_DRIVER_LOOP_DT_MS);
//		if(y_current_speed < y_target_speed)
//		{
//			y_current_speed = y_next_speed;
//		}
//	}
//	else
//	{
//		y_next_speed = CalculateSpeed(y_current_speed, -1.0f * y_target_acceleration, MOTOR_DRIVER_LOOP_DT_MS);
//		if(y_next_speed > MOTOR_DRIVER_Y_MIN_SPEED)
//		{
//			y_current_speed = y_next_speed;
//		}
//	}
//
//	DEBUG_printf("y-rem: %f, y-speed: %f\n", y_rem, y_current_speed);

	if(abs(x_command_location - x_cur) > x_rem)
	{
		x_accel_sign = 1;
		x_motor_state = ACCELERATION;
	}
	else
	{
		x_accel_sign = -1;
		if(x_motor_state == CONSTANT_SPEED)
		{
			x_motor_state = DECELERATION;
			x_command_speed = 0.0f;
		}
	}

	if(abs(y_command_location - y_cur) > y_rem)
	{
		y_accel_sign = 1;
		y_motor_state = ACCELERATION;
	}
	else
	{
		y_accel_sign = -1;
		if(y_motor_state == CONSTANT_SPEED)
		{
			x_motor_state = DECELERATION;
			x_command_speed = 0.0f;
		}
	}


	DEBUG_printf("[Y] sp: %f, com: %f, loc; cur: %f, com: %f, rem: %f, sign: %d, accel com: %f\n",
			y_current_speed, y_command_speed, y_cur, y_command_location, y_rem, y_accel_sign, y_command_acceleration);


//	CalculateSpeed(x_command_location, x_cur, x_integral, x_previos_err, x_current_speed, x_command_acceleration);
//	CalculateSpeed(y_command_location, y_cur, y_integral, y_previos_err, y_current_speed, y_command_acceleration);
//
	if(x_cur < x_command_location)
	{
		x_current_speed = ((x_accel_sign*x_current_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + x_current_speed);
	}
	else
	{
		x_current_speed = ((x_accel_sign*-1*x_current_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + x_current_speed);
	}
	if(y_cur < y_command_location)
	{
		y_current_speed = ((y_accel_sign*y_current_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + y_current_speed);
	}
	else
	{
		y_current_speed = ((y_accel_sign*-1*y_current_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + y_current_speed);
	}


	if(x_current_speed >= 0.0f)
	{
		if(x_current_speed < MOTOR_DRIVER_X_MIN_SPEED)
		{
			x_current_speed = MOTOR_DRIVER_X_MIN_SPEED;
		}
		else if(x_accel_sign > 0 && abs(x_current_speed) > abs(x_command_speed))
		{
			x_current_speed = x_command_speed;
			x_motor_state = CONSTANT_SPEED;
		}
		else if(abs(x_current_speed) > abs(x_command_speed))
		{
			x_current_speed = x_command_speed;
		}
		is_x_speed_positive = true;
	}
	else
	{
		if(x_current_speed > (-1*MOTOR_DRIVER_X_MIN_SPEED))
		{
			x_current_speed = (-1*MOTOR_DRIVER_X_MIN_SPEED);
		}
		else if(x_accel_sign > 0 && abs(x_current_speed) > abs(x_command_speed))
		{
			x_current_speed = x_command_speed;
			x_motor_state = CONSTANT_SPEED;
		}
		else if(abs(x_current_speed) > abs(x_command_speed))
		{
			x_current_speed = x_command_speed;
		}
		is_x_speed_positive = false;
	}
	if(abs(x_cur - x_command_location) <= MOTOR_DRIVER_LOCATION_DEADBAND)
	{
		x_command_speed = 0.0f;
		if(abs(x_current_speed) < (10.0f*MOTOR_DRIVER_X_MIN_SPEED))
		{
			x_current_acceleration = 0.0f;
		}
	}
	else
	{
		x_current_acceleration = x_command_acceleration;
	}

	if(y_current_speed >= 0.0f)
	{
		if(y_current_speed < MOTOR_DRIVER_Y_MIN_SPEED)
		{
			y_current_speed = MOTOR_DRIVER_Y_MIN_SPEED;
		}
		else if(y_accel_sign > 0 && abs(y_current_speed) > abs(y_command_speed))
		{
			y_current_speed = y_command_speed;
			y_motor_state = CONSTANT_SPEED;
		}
		else if(abs(y_current_speed) > abs(y_command_speed))
		{
			y_current_speed = y_command_speed;
		}
		is_y_speed_positive = true;
	}
	else
	{
		if(y_current_speed > (-1*MOTOR_DRIVER_Y_MIN_SPEED))
		{
			y_current_speed = (-1*MOTOR_DRIVER_Y_MIN_SPEED);
		}
		else if(y_accel_sign > 0 && y_current_speed < (-1*y_command_speed))
		{
			y_current_speed = -1*y_command_speed;
			y_motor_state = CONSTANT_SPEED;
		}
		is_y_speed_positive = false;
	}

	if(abs(y_cur - y_command_location) <= MOTOR_DRIVER_LOCATION_DEADBAND)
	{
		y_command_speed = 0.0f;
		if(abs(y_current_speed) < (10.0f*MOTOR_DRIVER_Y_MIN_SPEED))
		{
			y_current_acceleration = 0.0f;
		}
	}
	else
	{
		y_current_acceleration = y_command_acceleration;
	}


//	if(x_previos_err > MOTOR_DRIVER_X_ERR_MAX)
//	{
//		x_previos_err = MOTOR_DRIVER_X_ERR_MAX;
//	}
//	else if(x_previos_err < (-1*MOTOR_DRIVER_X_ERR_MAX))
//	{
//		x_previos_err = -1*MOTOR_DRIVER_X_ERR_MAX;
//	}
//	if(y_previos_err > MOTOR_DRIVER_Y_ERR_MAX)
//	{
//		y_previos_err = MOTOR_DRIVER_Y_ERR_MAX;
//	}
//	else if(y_previos_err < (-1*MOTOR_DRIVER_Y_ERR_MAX))
//	{
//		y_previos_err = -1*MOTOR_DRIVER_Y_ERR_MAX;
//	}
//
//	if(x_integral > MOTOR_DRIVER_X_ERR_MAX)
//	{
//		x_integral = MOTOR_DRIVER_X_ERR_MAX;
//	}
//	else if(x_integral < (-1*MOTOR_DRIVER_X_ERR_MAX))
//	{
//		x_integral = -1*MOTOR_DRIVER_X_ERR_MAX;
//	}
//	if(y_integral > MOTOR_DRIVER_X_ERR_MAX)
//	{
//		y_integral = MOTOR_DRIVER_X_ERR_MAX;
//	}
//	else if(y_integral < (-1*MOTOR_DRIVER_X_ERR_MAX))
//	{
//		y_integral = -1*MOTOR_DRIVER_X_ERR_MAX;
//	}


	StartPulses(x_current_speed, y_current_speed);
#endif

	int8_t x_accel_sign = 1;
	int8_t y_accel_sign = 1;

	float tmp_acceleration = 1.0f;

	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);

	float dt = 0.000001f;
	if(micros() > last_micros)
	{
		dt = (micros() - last_micros);
	}
	last_micros = micros();

	UpdateXState();
	UpdateYState();

	switch(x_motor_state)
	{
	case MOTOR_STATE::IDLE:

		break;

	case MOTOR_STATE::ACCELERATION:
	case MOTOR_STATE::DECELERATION:
		tmp_acceleration = abs(x_target_speed - x_current_speed)/(dt/1000000.0f);//((float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f);
		if(tmp_acceleration > x_command_acceleration)
		{
			tmp_acceleration = x_command_acceleration;
		}
		if(x_current_speed < x_target_speed)
		{
//			x_current_speed = ((tmp_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + x_current_speed);
			x_current_speed = ((tmp_acceleration*(dt/1000000.0f)) + x_current_speed);
		}
		else
		{
//			x_current_speed = ((-1*tmp_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + x_current_speed);
			x_current_speed = ((-1*tmp_acceleration*(dt/1000000.0f)) + x_current_speed);
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:

		break;

	default:
		break;
	}
	if(x_current_speed >= 0.0f)
	{
		is_x_speed_positive = true;
	}
	else
	{
		is_x_speed_positive = false;
	}


	switch(y_motor_state)
	{
	case MOTOR_STATE::IDLE:

		break;

	case MOTOR_STATE::ACCELERATION:
	case MOTOR_STATE::DECELERATION:
		tmp_acceleration = abs(y_target_speed - y_current_speed)/(dt/1000000.0f);//((float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f);
		if(tmp_acceleration > y_command_acceleration)
		{
			tmp_acceleration = y_command_acceleration;
		}
		if(y_current_speed < y_target_speed)
		{
//			y_current_speed = ((tmp_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + y_current_speed);
			y_current_speed = ((tmp_acceleration*(dt/1000000.0f)) + y_current_speed);
		}
		else
		{
//			y_current_speed = ((-1*tmp_acceleration*(float)MOTOR_DRIVER_LOOP_DT_MS/1000.0f) + y_current_speed);
			y_current_speed = ((-1*tmp_acceleration*(dt/1000000.0f)) + y_current_speed);
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:

		break;

	default:
		break;
	}
	if(y_current_speed >= 0.0f)
	{
		is_y_speed_positive = true;
	}
	else
	{
		is_y_speed_positive = false;
	}


	StartPulses(x_current_speed, y_current_speed);

	float x_rem = GetXrem();
	DEBUG_printf("[X] speed: %f, com: %f, tar: %f, loc; cur: %f, com: %f, rem: %f, mode: %d, dt:%f\n",
			x_current_speed, x_command_speed, x_target_speed, x_cur, x_command_location, x_rem, x_motor_state, dt);
}

void MotorDriver::UpdateXState(void)
{
	// Needed to add a little margin so it wouldn't switch between modes and make a smooth stop
	float x_rem = 1.3f*GetXrem();
	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);
	float x_diff = abs(x_cur - x_command_location);

	switch(x_motor_state)
	{
	case MOTOR_STATE::IDLE:
		if(x_diff > MOTOR_DRIVER_LOCATION_DEADBAND)
		{
			if(x_command_location > x_cur)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
			x_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;

	case MOTOR_STATE::ACCELERATION:
		if((SIGN(x_current_speed) == SIGN(x_target_speed)) && (abs(x_current_speed) >= abs(x_target_speed)))
		{
			x_motor_state = MOTOR_STATE::CONSTANT_SPEED;
		}
		if(abs(x_cur - x_command_location) < x_rem)
		{
			x_target_speed = MOTOR_DRIVER_X_MIN_SPEED;
			x_motor_state = MOTOR_STATE::DECELERATION;
		}
		else
		{
			if(x_cur < x_command_location)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
		}
		break;
	case MOTOR_STATE::DECELERATION:
		if(abs(x_current_speed) <= MOTOR_DRIVER_X_MIN_SPEED)
		{
			x_motor_state = MOTOR_STATE::IDLE;
		}
		if(abs(x_cur - x_command_location) > x_rem)
		{
			if(x_cur < x_command_location)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
			x_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:

		if(abs(x_cur - x_command_location) < x_rem)
		{
			x_target_speed = MOTOR_DRIVER_X_MIN_SPEED;
			x_motor_state = MOTOR_STATE::DECELERATION;
		}
		else if(
				((SIGN(x_current_speed) == SIGN(x_target_speed)) && (abs(x_current_speed) < abs(x_target_speed))) ||
				(SIGN(x_current_speed) != SIGN(x_target_speed))
		)
		{
			x_target_speed = SIGN(x_current_speed)*x_command_speed;
			x_motor_state = MOTOR_STATE::ACCELERATION;
		}
		if(x_motor_state != MOTOR_STATE::DECELERATION)
		{
			if(x_cur < x_command_location)
			{
				x_target_speed = x_command_speed;
			}
			else
			{
				x_target_speed = -1*x_command_speed;
			}
		}
		break;

	default:
		break;
	}
}

void MotorDriver::UpdateYState(void)
{
	// Needed to add a little margin so it wouldn't switch between modes and make a smooth stop
	float y_rem = 1.3f*GetYrem();
	float x_cur = 0.0f;
	float y_cur = 0.0f;
	GetXY(&x_cur, &y_cur);
	float y_diff = abs(y_cur - y_command_location);

	switch(y_motor_state)
	{
	case MOTOR_STATE::IDLE:
		if(y_diff > MOTOR_DRIVER_LOCATION_DEADBAND)
		{
			if(y_command_location > y_cur)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
			y_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;

	case MOTOR_STATE::ACCELERATION:
		if((SIGN(y_current_speed) == SIGN(y_target_speed)) && (abs(y_current_speed) >= abs(y_target_speed)))
		{
			y_motor_state = MOTOR_STATE::CONSTANT_SPEED;
		}
		if(abs(y_cur - y_command_location) < y_rem)
		{
			y_target_speed = MOTOR_DRIVER_Y_MIN_SPEED;
			y_motor_state = MOTOR_STATE::DECELERATION;
		}
		else
		{
			if(y_cur < y_command_location)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
		}
		break;
	case MOTOR_STATE::DECELERATION:
		if(abs(y_current_speed) <= MOTOR_DRIVER_Y_MIN_SPEED)
		{
			y_motor_state = MOTOR_STATE::IDLE;
		}
		if(abs(y_cur - y_command_location) > y_rem)
		{
			if(y_cur < y_command_location)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
			y_motor_state = MOTOR_STATE::ACCELERATION;
		}
		break;
	case MOTOR_STATE::CONSTANT_SPEED:

		if(abs(y_cur - y_command_location) < y_rem)
		{
			y_target_speed = MOTOR_DRIVER_Y_MIN_SPEED;
			y_motor_state = MOTOR_STATE::DECELERATION;
		}
		else if(
				((SIGN(y_current_speed) == SIGN(y_target_speed)) && (abs(y_current_speed) < abs(y_target_speed))) ||
				(SIGN(y_current_speed) != SIGN(y_target_speed))
				)
		{
			y_target_speed = SIGN(y_current_speed)*y_command_speed;
			y_motor_state = MOTOR_STATE::ACCELERATION;
		}
		if(y_motor_state != MOTOR_STATE::DECELERATION)
		{
			if(y_cur < y_command_location)
			{
				y_target_speed = y_command_speed;
			}
			else
			{
				y_target_speed = -1*y_command_speed;
			}
		}
		break;

	default:
		break;
	}
}

float MotorDriver::GetXrem()
{
//	return 0.5f*(x_current_speed - x_target_speed)*(x_current_speed - x_target_speed)/x_command_acceleration;
	float rem = 0.5f*(x_current_speed)*(x_current_speed)/x_command_acceleration;
	if(rem < MOTOR_DRIVER_LOCATION_REM_MIN)
	{
		rem = MOTOR_DRIVER_LOCATION_REM_MIN;
	}
	return rem;
}

float MotorDriver::GetYrem()
{
//	return 0.5f*(y_current_speed - y_target_speed)*(y_current_speed - y_target_speed)/y_command_acceleration;
	float rem = 0.5f*(y_current_speed)*(y_current_speed)/y_command_acceleration;
	if(rem < MOTOR_DRIVER_LOCATION_REM_MIN)
	{
		rem = MOTOR_DRIVER_LOCATION_REM_MIN;
	}
	return rem;
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

bool is_first_time = true;
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

void MotorDriver::StopPulses()
{

}

void MotorDriver::SetXY(float x, float y)
{
	SetXY(x, y, x_command_speed, y_command_speed);
}

/**
 * speed is mm/s
 */
void MotorDriver::SetXY(float x, float y, float x_speed, float y_speed)
{
	SetXY(x, y, x_speed, y_speed, MOTOR_DRIVER_X_DEFAULT_ACCELERAYION, MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION);
}


/**
 * speed is mm/s, acceleration is mm/s^2 and positive
 */
void MotorDriver::SetXY(float x, float y, float x_speed, float y_speed, float x_acceleration, float y_acceleration)
{
	if(x < MOTOR_DRIVER_X_COARSE)
	{
		x_command_location = x;
	}
	else
	{
		x_command_location = MOTOR_DRIVER_X_COARSE;
	}
	if(y < MOTOR_DRIVER_Y_COARSE)
	{
		y_command_location = y;
	}
	else
	{
		y_command_location = MOTOR_DRIVER_Y_COARSE;
	}

	if(x_acceleration < 0)
	{
		x_acceleration *= -1.0f;
	}
	if(y_acceleration < 0)
	{
		y_acceleration *= -1.0f;
	}
	if(x_speed < 0)
	{
		x_speed *= -1.0f;
	}
	if(y_speed < 0)
	{
		y_speed *= -1.0f;
	}

	x_target_half_pulse_counter = x_command_location * MOTOR_DRIVER_X_PULSE_PER_REVOLUTION / MOTOR_DRIVER_X_SCREW_PITCH * 2;
	y_target_half_pulse_counter = y_command_location * MOTOR_DRIVER_Y_PULSE_PER_REVOLUTION / MOTOR_DRIVER_Y_SCREW_PITCH * 2;

	x_command_acceleration = x_acceleration;
	y_command_acceleration = y_acceleration;
	x_command_speed = x_speed;
	y_command_speed = y_speed;

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

MotorDriver::MOTOR_STATE MotorDriver::GetYState()
{
	return y_motor_state;
}
MotorDriver::MOTOR_STATE MotorDriver::GetXState()
{
	return x_motor_state;
}

/**
 * At the moment, for both start and end, we just need to reset everthing
 */
void MotorDriver::LimitXStartEvent()
{
	x_command_speed = MOTOR_DRIVER_X_MIN_SPEED;
	x_target_speed = 0.0f;
	x_current_speed = 0.0f;
	x_command_acceleration = MOTOR_DRIVER_X_DEFAULT_ACCELERAYION;
	x_current_acceleration = 0.0f;
	x_motor_state = MOTOR_STATE::IDLE;
}
void MotorDriver::LimitXEndEvent()
{
	x_command_speed = MOTOR_DRIVER_X_MIN_SPEED;
	x_target_speed = 0.0f;
	x_current_speed = 0.0f;
	x_command_acceleration = MOTOR_DRIVER_X_DEFAULT_ACCELERAYION;
	x_current_acceleration = 0.0f;
	x_motor_state = MOTOR_STATE::IDLE;
}
void MotorDriver::LimitYStartEvent()
{
	y_command_speed = MOTOR_DRIVER_Y_MIN_SPEED;
	y_target_speed = 0.0f;
	y_current_speed = 0.0f;
	y_command_acceleration = MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION;
	y_current_acceleration = 0.0f;
	y_motor_state = MOTOR_STATE::IDLE;
}
void MotorDriver::LimitYEndEvent()
{
	y_command_speed = MOTOR_DRIVER_Y_MIN_SPEED;
	y_target_speed = 0.0f;
	y_current_speed = 0.0f;
	y_command_acceleration = MOTOR_DRIVER_Y_DEFAULT_ACCELERAYION;
	y_current_acceleration = 0.0f;
	y_motor_state = MOTOR_STATE::IDLE;
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

