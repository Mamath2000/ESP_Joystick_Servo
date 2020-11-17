/*
   Blink
   Turns on an LED on for one second,
   then off for one second, repeatedly.
*/

#include "Arduino.h"
#include "Servo.h"

const int Joy1_x = 13;
const int Joy1_y = 15;

const int Joy2_x = 12;
const int Joy2_y = 16;

int servoPin = D5;
Servo Servo1;
int serValX;

struct joystickValue
{
  int x;
  int y;
};

unsigned long DELAY_TIME = 30; // 1.5 sec
unsigned long delayStart = 0;    // the time the delay started
bool delayRunning = false;       // true if still waiting for delay to finish

bool ledOn = false;

struct joystickValue read_joystick(bool j1)
{

  struct joystickValue values;
  int pin1, pin2;

  if (j1) {
    pin1 = Joy1_x;
    pin2 = Joy1_y;
  } else {
    pin1 = Joy2_x;
    pin2 = Joy2_y;
  }

  digitalWrite(pin1, HIGH);
  values.x = analogRead(A0);
  digitalWrite(pin1, LOW);
  digitalWrite(pin2, HIGH);
  delay(5);
  values.y = analogRead(A0);
  digitalWrite(pin2, LOW);
  return values;
}

void setup()
{
  //Serial.begin(115200);

 digitalWrite(LED_BUILTIN, LOW);
delay(100);
 digitalWrite(LED_BUILTIN, HIGH);
delay(100);
 digitalWrite(LED_BUILTIN, LOW);
delay(100);
 digitalWrite(LED_BUILTIN, HIGH);
delay(100);
 digitalWrite(LED_BUILTIN, LOW);
delay(100);
digitalWrite(LED_BUILTIN, HIGH);
delay(100);

  Servo1.attach(servoPin);


  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(Joy1_x, OUTPUT);
  pinMode(Joy1_y, OUTPUT);
 // pinMode(Joy2_x, OUTPUT);
 // pinMode(Joy2_y, OUTPUT);
  digitalWrite(Joy1_x, LOW);
  digitalWrite(Joy1_y, LOW);
 // digitalWrite(Joy2_x, LOW);
 // digitalWrite(Joy2_y, LOW);

Servo1.write(0);
delay(500);
Servo1.write(180);
delay(100);
Servo1.write(90);
delay(100);
Servo1.write(180);
delay(1000);
Servo1.write(0);
delay(100);
Servo1.write(90);
delay(100);
Servo1.write(0);
delay(1000);
Servo1.write(180);
delay(1000);
Servo1.write(0);
 
  // start delay
  delayStart = millis();
  delayRunning = true;
}

void loop()
{

  struct joystickValue values;

  // check if delay has timed out
  if (delayRunning && ((millis() - delayStart) >= DELAY_TIME))
  {
    delayStart += DELAY_TIME; // this prevents drift in the delays
    // toggle the led

    Serial.print("new Read Joy1 : ");
    values = read_joystick(true); //(13, 15);
    /*Serial.print("[");
    Serial.print(values.x);
    Serial.print(":");
    Serial.print(values.y);
    Serial.print("]");
*/
    serValX = map(values.x, 0 , 1023, 0, 180);
  //  Serial.print(" - ");
  //  Serial.println(serValX);
    Servo1.write(serValX);

    /*
        Serial.print("new Read Joy2 : ");
        values = read_joystick(false); //(13, 15);
        Serial.print("[");
        Serial.print(values.x);
        Serial.print(":");
        Serial.print(values.y);
        Serial.println("]");
    */

    ledOn = !ledOn;
    if (ledOn)
    {
      // turn the LED on (HIGH is the voltage level)
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      // turn the LED off by making the voltage LOW
      digitalWrite(LED_BUILTIN, LOW);
    }
  }
}
