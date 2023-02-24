#pragma once

#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h"

void calibrateGyro();

void printGyro(int x, int y, int z);

int arrayAverage(int* array);

void calculateAccel(int* averageX, int* averageY, int* averageZ);