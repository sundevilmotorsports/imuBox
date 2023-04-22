#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h"
#include <Wire.h>
#include <FlexCAN_T4.h>

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData;
int xAngle = 0, yAngle = 0, zAngle = 0; //calculated absolute position in the x, y, and z axes
int xLim = 0, yLim = 0, zLim = 0; //Positive absolute values of the noise threshold determined in calibrateGyro()
byte counter; //keeps track of which accelerometer values to replace with each setup() interation

//The number of loops run before the x, y, and z accelerometer values are averaged
int averagerCounter = 3;
//These store the past 3 x, y, and z accelerometer values
int xValues [averagerCounter];
int yValues [averagerCounter];
int zValues [averagerCounter];

//Objects to be sent on the CANFD bus.
CANFD_message_t canAccelMessage, canDirectionMessage;
//canAccelMessage holds x, y, z acceleration data. canDirectionMessage holds pitch, roll, and yaw (still unimplemented)


void setup() {
  // put your setup code here, to run once:
	Wire.begin();

	Serial.begin(115200);

	while(!Serial) {
		delay(1);
	}

	if(!myISM.begin() ){
		Serial.println("Did not begin.");
		while(1);
	}

	// Reset the device to default settings. This if helpful is you're doing multiple
	// uploads testing different settings. 
	myISM.deviceReset();

	// Wait for it to finish reseting
	while( !myISM.getDeviceReset() ){ 
		delay(1);
	} 

	Serial.println("Reset.");
	Serial.println("Applying settings.");
	delay(100);
	
	myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_104Hz);
	myISM.setAccelFullScale(ISM_4g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_52Hz);
	myISM.setGyroFullScale(ISM_125dps); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	myISM.setGyroFilterLP1();
	myISM.setGyroLP1Bandwidth(ISM_MEDIUM);

	// Turn on CAN bus
	FlexCAN_T4FD<CAN3, RX_SIZE_256, TX_SIZE_16> myCAN;
	myCAN.begin();

	//Set CAN baud rate
	CANFD_timings_t config;
	config.clock = CLK_24MHz;
	config.baudrate = 1000000;
	config.baudrateFD = 2000000;
	config.propdelay = 190;
	config.bus_length = 1;
	config.sample = 70;
	FD.setBaudRate(config);

	//CAN mailboxes are interrupt-driven, meaning it does stuff when a message appears
	myCAN.enableMBInterrupts();

	//Reject all incoming messages
	myCAN.setMBFilter(REJECT_ALL);

	delay(500);
	calibrateGyro();

}

void sendToBus() {
	//Check https://docs.google.com/spreadsheets/d/1tEKqx7z3uw22POPMZ_HQZkzSWVvlohXokliIyHb52fE/edit#gid=824743844 for standard
	canAccelMessage.idhit = 0x360;
	canAccelMessage.timestamp = millis();
	
	canAccelMessage.buf[0] = arrayAverage(xValues);
	canAccelMessage.buf[1] = arrayAverage(yValues);
	canAccelMessage.buf[2] = arrayAverage(zValues);

	myCan.write(canAccelMessage);

}

void loop() {

  // Check if both gyroscope and accelerometer data is available.
	if(myISM.checkStatus()){
		//myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);

		//log current gyro data for averaging later
		xValues[counter] = gyroData.xData;
		yValues[counter] = gyroData.yData;
		zValues[counter] = gyroData.zData;

		//Every 3 loop iterations, average the past 3 gyro data points and print them
		if (counter == averagerCounter-1) {

			//This commented portion was solely used for testing the acceleration/position calculations by printing them.
			/*
			calculateAccel(arrayAverage(xValues), arrayAverage(yValues), arrayAverage(zValues))
			printGyro(xAngle, yAngle, zAngle);
			*/

			sendToBus();
		} else {counter++;}

	}

	delay(10);
}