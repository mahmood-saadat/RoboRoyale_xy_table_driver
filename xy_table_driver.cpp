// Do not remove the include below
#include "xy_table_driver.h"


//The setup function is called once at startup of the sketch
void setup()
{
	Serial.begin(115200);
	Serial.printf("Started!\r\n");
	WifiManager::WiFiOn();
	motorDriver.begin();
	pulseGenerator.begin();
}

// The loop function is called in an endless loop
void loop()
{
	vTaskDelay(5000);
}
