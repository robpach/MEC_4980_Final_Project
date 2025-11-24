#include <Arduino.h>
#include "SparkFun_Qwiic_OLED.h"
#include <Buzzer.h>

// Thermistor variables
const float R = 10000.0;
const float Rtherm = 100000.0;
int sensorPin = A0;
float reading;
float T25 = 298.0;
float beta = 3750.0;
float tempK, voltage, resistance, tempC, tempF;
char disp[40];

// Temperature control variables
int target = 200; // Fahrenheit

// Buzzer Pin
const int buzzerPin = 2;

// Initialize general pins
const int onButton = 4;
const int upButton = 5;
const int downButton = 6;

// initialize objects
Buzzer buzzer(buzzerPin);
QwiicMicroOLED myOLED;

void setup()
{
  delay(500);
  Serial.begin(9600);
  delay(500);

  pinMode(onButton, INPUT_PULLUP);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);

  while (!myOLED.begin())
  {
    Serial.println("OLED failed");
    delay(1000);
  }

  myOLED.text(0, 0, "clearingthescreen");
  myOLED.text(0, 10, "clearingthescreen");
  myOLED.text(0, 20, "clearingthescreen");
  myOLED.text(0, 30, "clearingthescreen");
  myOLED.text(0, 40, "clearingthescreen");
  myOLED.display();
  delay(500);
  myOLED.erase();
  myOLED.display();
}

void updateTemps()
{

}

void increaseTarget()
{
  // pause for 200 ms, increase temp by 5 F, then buzz
}

void decreaseTarget()
{
  // pause for 200 ms, decrease temp by 5 F, then buzz
}

void togglePower()
{
  
}

void loop()
{
  myOLED.erase();
  reading = analogRead(sensorPin);
  voltage = float(reading / 1023) * 5;
  resistance = float(voltage * R) / float(5 - voltage);
  tempK = 1.0 / ((1.0 / T25) + 1.0 / beta * log(resistance / Rtherm));
  tempC = tempK - 273.15;
  char tempStr[10];
  myOLED.text(0,0, "Temp:");
  tempF = tempC*(9.0/5.0) + 32.0;
  dtostrf(tempF, 3, 2, tempStr);
  snprintf(disp, sizeof(disp), "%s F", tempStr);
  myOLED.text(0, 10, disp);
  Serial.println(disp);
  myOLED.text(0,25, "Target:");
  snprintf(disp, sizeof(disp), "%i F", target);
  myOLED.text(0,35,disp);

  myOLED.display();
  delay(1000);
}