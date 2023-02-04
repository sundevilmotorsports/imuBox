#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h"
#include <Wire.h>

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData;
int xAngle = 0, yAngle = 0, zAngle = 0; //calculated absolute position in the x, y, and z axes
int xLim = 0, yLim = 0, zLim = 0; //Positive absolute values of the noise threshold determined in calibrateGyro()
byte counter; //keeps track of which accelerometer values to replace with each setup() interation

//These store the past 3 x, y, and z accelerometer values to be averaged
int xValues [3];
int yValues [3];
int zValues [3];

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

void printGyro(int x, int y, int z) {

	/* Commented out whiel testing the averaging method of smoothing accel data
	Serial.print("X: ");
	if (abs(x) > xLim) {
		Serial.print(x);
		xAngle = xAngle + x;
	}
	else {
		Serial.print("---");
	}

	Serial.print("\tY: ");
	if (abs(y) > yLim) {
		Serial.print(y);
		yAngle = yAngle + y;
	}
	else {
		Serial.print("---");
	}

	Serial.print("\tZ: ");
	if (abs(z) > zLim) {
		Serial.println(z);
		zAngle = zAngle + z;
	}
	else {
		Serial.println("---");
	}

	*/

	Serial.print("X: ");
	Serial.print(x);

	Serial.print("\tY: ");
	Serial.print(y);

	Serial.print("\tZ: ");
	Serial.println(z);

	Serial.print("       							    XPos: ");
	Serial.print(xAngle);
	Serial.print(" YPos: ");
	Serial.print(yAngle);
	Serial.print(" ZPos: ");
	Serial.println(zAngle);
	
}

int arrayAverage(int* array) {

	byte total = 0;

	for (int i = 0; i<3; i++) {
		total += array[i];
	}

	return total/3.0;

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

		xValues[counter] = gyroData.xData;
		yValues[counter] = gyroData.yData;
		zValues[counter] = gyroData.zData;

		if (counter == 2) {

			if (arrayAverage(xValues) >= xLim) {
				xAngle += arrayAverage(xValues);
			}
			if (arrayAverage(yValues) >= yLim) {
				yAngle += arrayAverage(yValues);
			}
			if (arrayAverage(zValues) >= zLim) {
				zAngle += arrayAverage(zValues);
			}

			printGyro(xAngle, yAngle, zAngle);

			counter = 0;

		} else {counter++;}

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
		


		delay(10);
	}
}