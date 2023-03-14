/*
 * Commands.cpp
 *
 *  Created on: Nov 18, 2021
 *      Author: M.Saadat (m.saadat@mail.com)
 * Github: https://github.com/mahmood-saadat/ESP32-Templates/tree/master/Task%20Templates
 * This file provide as is with no guarantee of any sort.
 * Any modification and redistribution of this file is allowed as long as this description is kept at the top of the file.
 */

#include "Commands.h"

#include <esp_task_wdt.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "MotorDriver.h"

#define		WDT_TASK_TIMEOUT 	3000
#define		WIFI_UDP_PORT		8745

Commands 		commands;
WiFiUDP 		wifiUDP;

Commands::Commands(){
	mainTask = NULL;
}

/*
 * Initialise the class
 */
void Commands::begin(){
	//Serial.println("[Commands] Setup is started ...");

	xTaskCreate((TaskFunction_t)Commands::TaskStart, "CommandsTask", 8192, this, 20, &mainTask);

	//Serial.println("[Commands] Setup is done.");

}

/*
 * Task function needed to be static and in static members, we could not access class members easily,
 * so we created this function and we passed the class as input argument, then we call our main function from here.
 */
TaskFunction_t Commands::TaskStart(void * pvParameters){
	Commands* manager;
	esp_task_wdt_init(WDT_TASK_TIMEOUT, true);
	esp_task_wdt_add(NULL);
	manager = (Commands*) pvParameters;
	manager->MainTask();
	// It will never get here. The main task never returns.
	return 0;
}
/*
 * The main loop
 */
void Commands::MainTask() {
	uint8_t 	receiveBuffer[255] = {0};
	int16_t 	packetSize = 0;
	bool		is_any_udp_packet_received = false;
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
				if(motorDriver.IsReady())
				{
					motorDriver.SetXY(x, y, x_speed, y_speed);
					is_any_udp_packet_received = true;
				}
				//Serial.printf("[Commands] X: %f, Y: %f, Speed: %f\n", x, y, x_speed);
			}
			else
			{
				//Serial.printf("[Commands] Wrong UDP data length: %d\n", len);
			}
		}

		// If any packet is received, we have the IP address to send back data
		if(is_any_udp_packet_received)
		{
			float x = 0.0f;
			float y = 0.0f;
			uint8_t buffer[8] = {0};
			motorDriver.GetXY(&x, &y);
			memcpy(&buffer[0], &x, 4);
			memcpy(&buffer[4], &y, 4);
			wifiUDP.beginPacket(wifiUDP.remoteIP(), WIFI_UDP_PORT);
			wifiUDP.write(buffer, 8);
			wifiUDP.endPacket();
		}

		// Check for commands from serial port
		// The packet is as this:
		// 3 bytes header: 0xFF 0xFE 0xFF
		// 4 bytes float: X target
		// 4 bytes float: X speed
		// 4 bytes float: Y target
		// 4 bytes float: Y speed
		// 1 byte checksum: sum of the X, Y and speed bytes (everything except the header)
		while(Serial.available() >= 20)
		{
			// Check if we have the header
			if(Serial.read() == 0xFF)
			{
				if(Serial.read() == 0xFE)
				{
					if(Serial.read() == 0xFF)
					{
						// Read the data bytes
						Serial.read(receiveBuffer, 17);
						if(receiveBuffer[16] == CalculateChecksum(receiveBuffer, 16))
						{
							float x = ConvertToFloat((uint8_t*)&receiveBuffer[0]);
							float x_speed = ConvertToFloat((uint8_t*)&receiveBuffer[4]);
							float y = ConvertToFloat((uint8_t*)&receiveBuffer[8]);
							float y_speed = ConvertToFloat((uint8_t*)&receiveBuffer[12]);
							if(motorDriver.IsReady())
							{
								motorDriver.SetXY(x, y, x_speed, y_speed);
							}

							// Send back the current location
							x = 0.0f;
							y = 0.0f;
							uint8_t buffer[12] = {0};
							motorDriver.GetXY(&x, &y);
							buffer[0] = 0xFF;
							buffer[1] = 0xFE;
							buffer[2] = 0xFF;
							memcpy(&buffer[3], &x, 4);
							memcpy(&buffer[7], &y, 4);
							buffer[11] = CalculateChecksum(&buffer[3], 8);
							Serial.write(buffer, 12);
						}
					}
				}
			}
		}

		PrintStackWatermark();
		vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

void Commands::PrintStackWatermark(){
	if(millis() > (stackWatermarkPrintLastMillis + 60000)){
		stackWatermarkPrintLastMillis = millis();
		Serial.printf("[Commands] Task Stack left: %d\r\n", uxTaskGetStackHighWaterMark(NULL));
	}
}

float Commands::ConvertToFloat(uint8_t * buffer)
{
	float float_value = 0.0f;
	memcpy(&float_value, &buffer[0], 4);
	return float_value;
}

uint8_t Commands::CalculateChecksum(uint8_t * buffer, uint16_t len)
{
	uint8_t checksum = 0;
	for(int i = 0; i < len; i++)
	{
		checksum += buffer[i];
	}
	return checksum;
}


