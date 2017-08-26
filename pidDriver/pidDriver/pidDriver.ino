#include <PID_v1.h>
#include "Adafruit_BMP085.h"

#include <CD74HC4067.h>
#define gndSamples 100

Adafruit_BMP085 bmp;

float x[100];
float groundAlt = 0;
CD74HC4067 mux(4, 5, 6, 7);

const int triggerPin = 9; //will change pretty soon
const int servoPin = 10;

Servo servo;
long lastTime = 0;

float setpoint, input, output;
float Kp=2, Ki=5, Kd=1;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);
  if (!bmp.begin(1)) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
      while(1){}
  }
  servo.attach(servoPin);
  int count = 0;
  while (count < gndSamples) {
    groundAlt += bmp.readAltitude(101325, bmp.readRawTemperature());
    count++;
  }
  groundAlt /= gndSamples;
  Serial.println(groundAlt);
  mux.channel(0);
}

void writePWM(int value)
{
  if (micros() - lastTime > 20000 - delayVal) {
    lastTime = micros();
    servo.writeMicroseconds(delayVal);
  }
}

void loop() {
  // put your main code here, to run repeatedly:

}
