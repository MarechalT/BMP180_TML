// BMP180_TML_Ex.ino
// Example of using the BMP180_TML library with the sensor connected using I2C.

// Copyright (c) 2018 Thibault MARECHAL.  All right reserved.
// @: tmarechal.projects@gmail.com
// Github: https://github.com/MarechalT/BMP180_TML
//
// connect the BMP180 board to the Arduino nano like this:
//       +5V ------ VIn
//      GND ----- GND
//      A5 ----- SCL
//      A4 ----- SDA

#include "BMP180_TML.h"

//create an instance of the class
BMP180_TML bmp;
const int loopDelay = 2000;

void setup() {

  Serial.begin(9600);
  //wait for serial connection to open (only necessary on some boards)
  while (!Serial)
    ;
  //Initiate the wire library
  Wire.begin();

  //initializes the instance, checks the sensor ID and reads the calibration parameters.
  if (!bmp.init()) {
    Serial.println("init() failed. check your BMP180 Connections and I2C Address.");
    while (1)
      ;
  }
  //reset sensor to default parameters.
  bmp.reset();
  //set ultra high resolution mode for pressure measurements
  bmp.setOss(BMP180_TML::UHR);
}

void loop() {

  //Measure the Pressure
  if (!bmp.measurePressure()) {
    Serial.println("could not start pressure measurement, is a measurement already running?");
    return;
  }
  //wait for the measurement to finish. proceed as soon as hasValue() returned true.
  do {
    delay(100);
  } while (!bmp.hasValue());

  Serial.print("Pressure: ");
  Serial.print(bmp.getPressure() / 100);
  Serial.println(" hPa");

  //Measure the Temperature
  if (!bmp.measureTemperature()) {
    Serial.println("could not start temperature measurement, chip computing...");
    return;
  }
  //wait for the measurement to finish. proceed as soon as hasValue() returned true.
  do {
    delay(100);
  } while (!bmp.hasValue());

  Serial.print("Temperature: ");
  Serial.print(bmp.getTemperature());
  Serial.println(" °C");
  //delay before next loop starts
  delay(loopDelay);
}
