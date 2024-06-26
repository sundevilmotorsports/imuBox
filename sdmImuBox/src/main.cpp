#include <Arduino.h>
#include "SparkFun_ISM330DHCX.h"
#include <Wire.h>
#include "FlexCAN_T4.h"

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

SparkFun_ISM330DHCX myISM; 

// Structs for X,Y,Z data
sfe_ism_data_t accelData; 
sfe_ism_data_t gyroData;

uint8_t hehez = HIGH;
float pitch = 0.0, roll = 0.0;
float pitchAccel = 0.0, rollAccel = 0.0;
float kpitch = 0.98;
float kroll = 0.98;

void setup() {
  // put your setup code here, to run once:
	Wire.begin();

	Serial.begin(115200);
	pinMode(13, OUTPUT);
	pinMode(7, OUTPUT);

	delay(100);

	if( !myISM.begin() ){
		Serial.println("Did not begin.");
		while( 1 ) {
			digitalWrite(13, HIGH);
			delay(1000);
			digitalWrite(13, LOW);
			delay(1000);
		}
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
	myISM.setAccelDataRate(ISM_XL_ODR_208Hz);
	myISM.setAccelFullScale(ISM_2g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_208Hz);
	myISM.setGyroFullScale(ISM_500dps); 

	// Turn on the accelerometer's filter and apply settings. 
	myISM.setAccelFilterLP2();
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

	// Turn on the gyroscope's filter and apply settings. 
	//myISM.setGyroFilterLP1();
	//myISM.setGyroLP1Bandwidth(ISM_AGGRESSIVE);

	// Turn on CAN bus
	Can0.begin();
	Can0.setBaudRate(1000000);
	Can0.setMaxMB(16);
	Can0.enableFIFO();
	Can0.enableFIFOInterrupt();
	//Can0.mailboxStatus();

	//Reject all incoming messages
	Can0.setMBFilter(REJECT_ALL);

	delay(500);

}

void loop() {
	digitalWrite(13, HIGH);

  // Check if both gyroscope and accelerometer data is available.
	static int prevTime = millis();
	if(myISM.checkStatus()){
		float dt = ((float) millis() - (float) prevTime) / 1000.0;
		prevTime = millis();
		myISM.getAccel(&accelData);
		myISM.getGyro(&gyroData);

		int xAccel = (int) accelData.xData;
		int yAccel = (int) accelData.yData;
		int zAccel = (int) accelData.zData;
		
		CAN_message_t msg0;
		msg0.id = 0x360;
		msg0.buf[0] = (xAccel & 0xFF000000) >> 24;
		msg0.buf[1] = (xAccel & 0x00FF0000) >> 16;
		msg0.buf[2] = (xAccel & 0x0000FF00) >> 8;
		msg0.buf[3] = xAccel & 0xFF;
		msg0.buf[4] = (yAccel & 0xFF000000) >> 24;
		msg0.buf[5] = (yAccel & 0x00FF0000) >> 16;
		msg0.buf[6] = (yAccel & 0x0000FF00) >> 8;
		msg0.buf[7] = yAccel & 0xFF;
		Can0.write(msg0);

		CAN_message_t msg1;
		msg1.id = 0x361;
		msg1.buf[0] = (zAccel & 0xFF000000) >> 24;
		msg1.buf[1] = (zAccel & 0x00FF0000) >> 16;
		msg1.buf[2] = (zAccel & 0x0000FF00) >> 8;
		msg1.buf[3] = zAccel & 0xFF;

		int xGyro = (int) gyroData.xData;
		int yGyro = (int) gyroData.yData;
		int zGyro = (int) gyroData.zData;

		pitchAccel = atan(-yAccel / sqrt(pow(xAccel, 2) + pow(zAccel, 2)));	
		rollAccel = atan(xAccel / sqrt(pow(yAccel, 2) + pow(zAccel, 2)));

		pitch = kpitch * (pitch + (((float) xGyro) * dt)) + (1.00 - kpitch) * (pitchAccel * 180000.0/3.1415);


		Serial.print(pitchAccel * 180000.0/3.1415);
		Serial.print("\t");
		Serial.println(pitch);

		msg1.buf[4] = (xGyro & 0xFF000000) >> 24;
		msg1.buf[5] = (xGyro & 0x00FF0000) >> 16;
		msg1.buf[6] = (xGyro & 0x0000FF00) >> 8;
		msg1.buf[7] = xGyro & 0xFF;

		Can0.write(msg1);

		CAN_message_t msg2;
		msg2.id = 0x362;
		msg2.buf[0] = (yGyro & 0xFF000000) >> 24;
		msg2.buf[1] = (yGyro & 0x00FF0000) >> 16;
		msg2.buf[2] = (yGyro & 0x0000FF00) >> 8;
		msg2.buf[3] = yGyro & 0xFF;
		msg2.buf[4] = (zGyro & 0xFF000000) >> 24;
		msg2.buf[5] = (zGyro & 0x00FF0000) >> 16;
		msg2.buf[6] = (zGyro & 0x0000FF00) >> 8;
		msg2.buf[7] = zGyro & 0xFF;
		Can0.write(msg2);

		} 
	static uint32_t timeout = millis();
	if ( millis() - timeout > 5000 ) {
		//Can0.mailboxStatus();
		timeout = millis();

		if (hehez == HIGH) {
			hehez = LOW;
			digitalWrite(7, hehez);
		}
		else {
			hehez = HIGH;
			digitalWrite(7, hehez);
		}
	}

	//delay(5);
}