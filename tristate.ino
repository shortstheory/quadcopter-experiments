#include <Servo.h>

int inPin = 2;         // the number of the input pin
int outPin = 13;       // the number of the output pin

int state = HIGH;      // the current state of the output pin
int reading;           // the current reading from the input pin
int previous = LOW;    // the previous reading from the input pin

// the follow variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long time = 0;         // the last time the output pin was toggled
long debounce = 200;   // the debounce time, increase if the output flickers
int currentMode=0;
int buttonState=0;
int lastButtonState=0;
int buttonPushCounter=0;

Servo servo;
int delayVal = 1900;

void setup()
{
  pinMode(inPin, INPUT);
  pinMode(outPin, OUTPUT);
}

void loop()
{
  buttonState = digitalRead(inPin);
  if(buttonState != lastButtonState)
  {
    if(buttonState == HIGH)
    {
      buttonPushCounter++;
      delay(50);
    }
  }
  lastButtonState = buttonState;
  int bC = buttonPushCounter % 3;
//  digitalWrite(outPin, state);


  previous = reading;
  
  if (bC == 0) {
    delayVal = 1900;
  } else if (bC == 1) {
    delayVal = 1500;
  } else if (bC == 2) {
    delayVal = 1100;
  }

  servo.attach(9);
  servo.writeMicroseconds(delayVal);
  Serial.println(delayVal);
  delayMicroseconds(10000-delayVal);
  delay(10);
}
