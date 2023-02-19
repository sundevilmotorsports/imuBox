#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h"

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

void calculateAccel(int* averageX, int* averageY, int* averageZ) {

    if (averageX >= xLim) {
		xAngle += arrayAverage(xValues);
	}
	if (averageY >= yLim) {
		yAngle += arrayAverage(yValues);
	}
	if (averageZ >= zLim) {
		zAngle += arrayAverage(zValues);
    }

	counter = 0;
    
}