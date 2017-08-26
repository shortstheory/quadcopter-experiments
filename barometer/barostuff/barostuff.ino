#include <Wire.h>
#include "bmp180.h"

struct bmp180_t bmp180;
int result = 0;

void setup()
{
  Wire.begin();
  Serial.begin(9600);
  Wire.beginTransmission(0x77);
  result = bmp180_init(&bmp180);
}

void loop()
{
  Serial.println(result);
}
