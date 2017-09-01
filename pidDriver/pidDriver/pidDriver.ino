#include <PID_v1.h>
#include <Servo.h>
#include "Adafruit_BMP085.h"

#include <CD74HC4067.h>
#define gndSamples 200
#define avgSamples 100
#define pidSampleTime 30
#define PID_MAX 1700
#define PID_MIN 1100
#define PWM_PIN 12

Adafruit_BMP085 bmp;

float x[100];
float groundAlt = 0;
float avg = 0;

CD74HC4067 mux(4, 5, 6, 7);

const int triggerPin = 9; //will change pretty soon
const int servoPin = 10;

Servo servo;
long lastTime = 0;
long barometerPreviousTime = 0;
float avgArray[avgSamples];
float altReading;

double setpoint, input, output;
double Kp=100, Ki=10, Kd=0.01;
float temp;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

bool autoMode = true;

int pwm_value;

void setup() {
  Serial.begin(115200);
  if (!bmp.begin(0)) {
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
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(PID_MIN, PID_MAX);
  pid.SetSampleTime(pidSampleTime);
}

void writePWM(int value)
{
  if (micros() - lastTime > 20000 - value) {
    lastTime = micros();
    servo.writeMicroseconds(value);
  }
}

long count = 0;

void autoSwitch(float holdAlt, bool state)
{
  if (autoMode == state) {
    return;
  }
  autoMode = state;
  if (!autoMode) {
    setpoint = holdAlt;
    mux.channel(1);
    pid.SetMode(AUTOMATIC);
  } else {
    mux.channel(0);
    pid.SetMode(MANUAL);
  }
  //change mode to auto and give it control
}

void checkMode(float holdAlt)
{
  pwm_value = pulseIn(PWM_PIN, HIGH, 16000);
  if (pwm_value > 1010) {
    autoSwitch(avg, false);
//    output = pwm_value;
  } else {
    autoSwitch(avg, true);
  }
}

int bc = 0;
void loop() {
  // put your main code here, to run repeatedly:
  bc++;
  if (millis() - barometerPreviousTime > 1000) {
    temp = bmp.readRawTemperature();
    barometerPreviousTime = millis();
    Serial.println(bc);
    bc = 0;
  }
  altReading = bmp.readAltitude(101325, temp);
  avgArray[count++%avgSamples] = altReading;

  avg = 0;
  for (int i = 0; i < avgSamples; i++) {
    avg += avgArray[i];
  }


//  setpoint = groundAlt + 10.0;
  
  avg /= avgSamples;
  input = altReading;
  checkMode(avg);
  pid.Compute();
  if (autoMode) {
    writePWM(output);
    Serial.println(output);
//    Serial.println(avg-groundAlt);
//    Serial.print(' ');
//    Serial.print(groundAlt);
//    Serial.print(' ');
//    Serial.println((output - 1100)/75.0 + 664);
  } else {
    writePWM(pwm_value);
  }
}
