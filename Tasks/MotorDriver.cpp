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
#include <WiFi.h>
#include <WiFiUdp.h>
#include "PulseGenerator.h"

#define		WDT_TASK_TIMEOUT 	3000
#define		WIFI_UDP_PORT		8745

MotorDriver 	motorDriver;
WiFiUDP 		wifiUDP;

MotorDriver::MotorDriver(){
	mainTask = NULL;
}

/*
 * Initialise the class
 */
void MotorDriver::begin(){
	Serial.println("[MotorDriver] Setup is started ...");

	xTaskCreate((TaskFunction_t)MotorDriver::TaskStart, "MotorDriverTask", 8192, this, 20, &mainTask);

	Serial.println("[MotorDriver] Setup is done.");

}

/*
 * Task function needed to be static and in static members, we could not access class members easily,
 * so we created this function and we passed the class as input argument, then we call our main function from here.
 */
TaskFunction_t MotorDriver::TaskStart(void * pvParameters){
	MotorDriver* manager;
	esp_task_wdt_init(WDT_TASK_TIMEOUT, true);
	esp_task_wdt_add(NULL);
	manager = (MotorDriver*) pvParameters;
	manager->MainTask();
	// It will never get here. The main task never returns.
	return 0;
}
/*
 * The main loop
 */
void MotorDriver::MainTask() {
	uint8_t 	receiveBuffer[255] = {0};
	int16_t 	packetSize = 0;
	bool		is_any_packet_received = false;
	wifiUDP.begin(WIFI_UDP_PORT);
	while(1){
		esp_task_wdt_reset();

		packetSize = wifiUDP.parsePacket();
		if(packetSize)
		{
			//			Serial.printf("Received %d bytes from %s, port %d\n", packetSize,
			//					wifiUDP.remoteIP().toString().c_str(), wifiUDP.remotePort());
			int len = wifiUDP.read(receiveBuffer, 255);
			if (len == 16)
			{
				//				for(int index = 0; index < len; index ++)
				//				{
				//					Serial.printf(" %x", receiveBuffer[index]);
				//				}
				//				Serial.printf("\n");

				float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
				float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
				float y = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
				float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
				if(pulseGenerator.IsReady())
				{
					pulseGenerator.SetXY(x, y, x_speed, y_speed);
					is_any_packet_received = true;

				}
				//Serial.printf("[MotorDriver] X: %f, Y: %f, Speed: %f\n", x, y, x_speed);
			}
			else
			{
				Serial.printf("[MotorDriver] Wrong data length: %d\n", len);
			}
		}

		// If any packet is received, we have the IP address to send back data
		if(is_any_packet_received)
		{
			float x = 0.0f;
			float y = 0.0f;
			uint8_t buffer[8] = {0};
			pulseGenerator.GetXY(&x, &y);
			memcpy(&buffer[0], &x, 4);
			memcpy(&buffer[4], &y, 4);
			wifiUDP.beginPacket(wifiUDP.remoteIP(), WIFI_UDP_PORT);
			wifiUDP.write(buffer, 8);
			wifiUDP.endPacket();
		}

		PrintStackWatermark();
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

void MotorDriver::PrintStackWatermark(){
	if(millis() > (stackWatermarkPrintLastMillis + 60000)){
		stackWatermarkPrintLastMillis = millis();
		Serial.printf("[MotorDriver] Task Stack left: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
	}
}

float MotorDriver::ConvertToFloat(uint8_t * buffer)
{
	float float_value = 0.0f;
	memcpy(&float_value, &buffer[0], 4);
	return float_value;
}


