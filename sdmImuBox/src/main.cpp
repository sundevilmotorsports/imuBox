#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h"
#include <Wire.h>

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData;
int xAngle = 0, yAngle = 0, zAngle = 0;
int xLim = 0, yLim = 0, zLim = 0; //Positive absolute values of the noise threshold determined in calibrateGyro()

void calibrateGyro() {
	int xMax = 0, yMax = 0, zMax = 0;
	while(!myISM.checkGyroStatus()) {
		delay(1);
	}

	myISM.getGyro(&gyroData);
	for (int i = 0; i < 1000; i++) {
		if(abs(gyroData.xData) > xMax) xMax = gyroData.xData;
		if(abs(gyroData.yData) > yMax) yMax = gyroData.yData;
		if(abs(gyroData.zData) > zMax) zMax = gyroData.zData;
		delay(1);
	}

	xLim = abs(xMax * 2);
	yLim = abs(yMax * 2);
	zLim = abs(zMax * 2);

	Serial.println("xLim: " + String(xLim) + "\tyLim: " + String(yLim) + "\tzLim: " + String(zLim));
}

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

	delay(500);
	calibrateGyro();

}

void loop() {
  // Check if both gyroscope and accelerometer data is available.
	if( myISM.checkStatus()){
		//myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);

		/*
		Serial.print("\nAccelerometer: ");
		Serial.print("X: ");
		Serial.print(accelData.xData);
		Serial.print(" ");
		Serial.print("Y: ");
		Serial.print(accelData.yData);
		Serial.print(" ");
		Serial.print("Z: ");
		Serial.print(accelData.zData);
		Serial.println(" ");
		*/

		Serial.print("X: ");
		if (abs(gyroData.xData) > xLim) {
			Serial.print(gyroData.xData);
			xAngle = xAngle + gyroData.xData;
		}
		else {
			Serial.print("---");
		}

		Serial.print("\tY: ");
		if (abs(gyroData.yData) > yLim) {
			Serial.print(gyroData.yData);
			yAngle = yAngle + gyroData.yData;
		}
		else {
			Serial.print("---");
		}

		Serial.print("\tZ: ");
		if (abs(gyroData.zData) > zLim) {
			Serial.println(gyroData.zData);
			zAngle = zAngle + gyroData.zData;
		}
		else {
			Serial.println("---");
		}

		//gyroData.xData and its y- and z-equivalents are in millidegrees/sec.
		//For visual testing of the sensor, set conversionFactor 10,000 to convert to degrees/tenth of a second.
		//For normal usage, keep at 10 to convert to millidegrees/tenth of a second.

		double conversionFactor = 1;//10000;
		/*
		Serial.print("Gyroscope: ");
		Serial.print("X: ");
		if (abs(gyroData.xData) > 250) {
			Serial.print(gyroData.xData/conversionFactor);
			xAngle += gyroData.xData/conversionFactor;
		}
		else {
			Serial.print(0);
		}

		Serial.print(" ");

		Serial.print("    Y: ");
		if (abs(gyroData.yData) > 450) {
			Serial.print(gyroData.yData/conversionFactor);
			yAngle += gyroData.yData/conversionFactor;
		}
		else {
			Serial.print(0);
		}

		Serial.print(" ");

		Serial.print("    Z: ");
		if (abs(gyroData.zData) > 70) {
			Serial.println(gyroData.zData/conversionFactor);
			zAngle += gyroData.zData/conversionFactor;
		}
		else {
			Serial.println(0);
		}  
		*/
		Serial.print("       							    XPos: ");
		Serial.print(xAngle);
		Serial.print(" YPos: ");
		Serial.print(yAngle);
		Serial.print(" ZPos: ");
		Serial.println(zAngle);
		


		delay(10);
	}
}