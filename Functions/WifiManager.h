/*
 * wifi_management.h
 *
 *  Created on: 8 Jan 2023
 *      Author: swacil-electronic
 */

#ifndef WIFI_MANAGER_H_
#define WIFI_MANAGER_H_

class WifiManager{

public:
	static void WiFiOn();
	static void WiFiOff();
	static bool IsConnected();
};



#endif /* WIFI_MANAGER_H_ */
