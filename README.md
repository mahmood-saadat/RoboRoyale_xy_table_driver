# RoboRoyale_xy_table_driver

This is the XY-Table driver for the RoboRoyale project. The XY table driver uses ESP32 and Arduino to control
two stepper motors, one for X-Axis and the other for Y-Axis.


## Description

This driver uses and ESP32 as a processor. The processor creates the Pulse and Direction signals for the stepper motor
driver. 

The wiring connections can be found in Diagrams folder.


## Getting Started

### Dependencies

* ESP32 toolchain for Arduino
* [Sloeber IDE](https://eclipse.baeyens.it/) (It can be build by Arduino IDE too)


### Installing

Simply clone the repository (look out which branch you want to use) on your local computer. Run Sloeber and import the 
project in workspace.

### Executing program

* Go to the properties of the project. On Sloeber section, select the UART port of the connected ESP32.
* Press Verify button to build the project
* Press Upload Sketch to load the program to your device

 
# Debug
For debug, connect to wifi and listen to this udp message:
 	nc -u -l 5480


## Authors

Contributors names and contact info:

* Mahmood Saadat (mahmood.saadat@durham.ac.uk)

## Version History

* 1.0.0
	* Initial Release
	
		- This is for the small XY table
* 1.1.0
	* The release for bigger table
	
		- The size of table adjusted for the new table
		
		- The frequency update functions is modified to prevent extra vibration when there is a continues command input from serial port
		
* 1.2.0
    * Acceleration
    
    	- This release has the acceleration for speed command generation.
    	
    	- Readme updated

## License

TBD

## Acknowledgments

* RoboRoyale: This work was supported by EU H2020-FET-OPEN RoboRoyale project number 964492


