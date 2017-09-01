#include <Wire.h>
#include "Adafruit_BMP085.h"
#include <Filter.h>

/*************************************************** 
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout 
  ----> https://www.adafruit.com/products/391

  These displays use I2C to communicate, 2 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
long count = 0;
void setup() {
  Serial.begin(115200);
  if (!bmp.begin(1)) {
	Serial.println("Could not find a valid BMP085 sensor, check wiring!");
//  while(1){}
  }
}

unsigned long previousTime =0;
uint16_t temp;
const int window = 100;
float x[window];

ExponentialFilter<float> FilteredTemperature(1, 0);
float altReading;
void loop() {
 
  if (millis() - previousTime > 1000) {
     temp = bmp.readRawTemperature();
     previousTime = millis();
  }
  altReading = bmp.readAltitude(101325, temp);
  FilteredTemperature.Filter(altReading);

  x[count%window] = altReading;
  float avg = 0;
  for (int i = 0; i < window; i++) {
    avg += x[i];
  }
  avg /= window; 
//  Serial.print(FilteredTemperature.Current());
//  Serial.print(' ');
  Serial.println(avg);
  count++;
}

//  x[count%window] = bmp.readAltitude(101325, temp);
//  float avg = 0;
//  for (int i = 0; i < window; i++) {
//    avg += x[i];
//  }
  //avg /= window; 
  //Serial.println(avg);
  //count++;

