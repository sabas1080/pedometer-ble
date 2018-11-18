/******************************************************************************
Pedometer_BLE.ino

Andres Sabas @ Electronic Cats
Nov 17, 2018
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library

Description:
This sketch counts steps taken and send via BLE.

Run the sketch and open a serial window at 9600 baud.  The sketch will display
the number of steps taken since reset.  lightly tap the sensor on something at the
rate of walking to simulate having the device in your pocket.  Note that you must
take 7 regularly spaced steps before the counter starts reporting.

Push the reset button to reset the device and count.

The configuration is determined by reading the LSM6DS3 datasheet and application
note, then driving hex values to the registers of interest to set the appropriate
bits.  The sketch is based of the "LowLevelExampe" sketch.

Development environment specifics:
- Arduino IDE 1.8.4

Hardware specifics:
- Arduino WiFi Rev.2

This code is released under the [MIT License](http://opensource.org/licenses/MIT).

Please review the LICENSE.md file included with this example. If you have any questions 
or concerns with licensing, please contact hola@electroniccats.com.

Distributed as-is; no warranty is given.
******************************************************************************/
#include <ArduinoBLE.h>
#include "SparkFunLSM6DS3.h"
#include "Wire.h"

LSM6DS3Core myIMU( SPI_MODE, SPIIMU_SS );
 // BLE Pedometer
BLEService pedometerService("9269c9b1-3221-4f66-b7bc-b0a29a4355ef");
// BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic stepsTakenChar("f1660dd9-6706-4c8b-b51a-4e40302bb4d1",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

long previousMillis = 0;  // last time checked, in ms
    
void setup()
{
	Serial.begin(9600);
	while (!Serial);
	Serial.println("Processor came out of reset.\n");

   // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

	//Call .beginCore() to configure the IMU
	if( myIMU.beginCore() != 0 )
	{
		Serial.print("Error at beginCore().\n");
	}
	else
	{
		Serial.print("\nbeginCore() passed.\n");
	}

	//Error accumulation variable
	uint8_t errorAccumulator = 0;

	uint8_t dataToWrite = 0;  //Temporary variable

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	//  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_200Hz;
	dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_2g;
	dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_26Hz;

	// //Now, write the patched together data
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	errorAccumulator += myIMU.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

	
	// Enable embedded functions -- ALSO clears the pdeo step count
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_CTRL10_C, 0x3E);
	// Enable pedometer algorithm
	errorAccumulator += myIMU.writeRegister(LSM6DS3_ACC_GYRO_TAP_CFG1, 0x40);
	// Step Detector interrupt driven to INT1 pin
	errorAccumulator += myIMU.writeRegister( LSM6DS3_ACC_GYRO_INT1_CTRL, 0x10 );
	
	if( errorAccumulator )
	{
		Serial.println("Problem configuring the device.");
	}
	else
	{
		Serial.println("LSM6DS3 O.K.");
	}	
	delay(200);

   /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("PedometerMonitor");
  BLE.setAdvertisedService(pedometerService); // add the service UUID
  pedometerService.addCharacteristic(stepsTakenChar); // add the stepsTaken characteristic
  BLE.addService(pedometerService); // Add the battery service
  //stepsTakenChar.writeValue(stepsTaken); // set initial value for this characteristic

  /* Start advertising BLE.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop()
{
    // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateStepsTaken();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
	//Wait 1 second
	//delay(1000);
	
}

void updateStepsTaken() {
  uint8_t readDataByte = 0;
  uint16_t stepsTaken = 0;
  //Read the 16bit value by two 8bit operations
  myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_H);
  stepsTaken = ((uint16_t)readDataByte) << 8;
  
  myIMU.readRegister(&readDataByte, LSM6DS3_ACC_GYRO_STEP_COUNTER_L);
  stepsTaken |= readDataByte;

  stepsTakenChar.writeValue(stepsTaken);  // and update the stepsTaken characteristic
  
  //Display steps taken
  Serial.print("Steps taken: ");
  Serial.println(stepsTaken);
 }
