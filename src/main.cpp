#include <Arduino.h>
#include "SparkFun_Qwiic_OLED.h"
#include <Buzzer.h>
#include <QuickPID.h>

// Thermistor variables
const float R = 10000.0;
const float Rtherm = 100000.0;
int sensorPin = A0;
float reading;
float T25 = 298.0;
float beta = 3750.0;
float tempK, voltage, resistance, tempC, tempF;
char disp[40];
unsigned long prevTime = 0;

// Temperature control variables
int target = 200; // Fahrenheit

// Buzzer Pin
const int buzzerPin = 2;

// PID Initialization
float Kp = 100, Ki = 1, Kd = 0.0;
float Input, Output, Setpoint;
QuickPID PID(&Input, &Output, &Setpoint);

// Initialize general pins
const int onButton = 3;
const int upButton = 6;
const int downButton = 5;
const int redLED = 10;
const int greenLED = 8;
const int tempSignal = 9;
bool onPressed = true;
bool upPressed = true;
bool downPressed = true;
bool currentOn;

// initialize objects
Buzzer buzzer(buzzerPin);
QwiicMicroOLED myOLED;

enum MachineStates
{
  Off,
  On
};

MachineStates currentState = Off;

void setup()
{
  delay(500);
  Serial.begin(9600);
  delay(500);

  pinMode(onButton, INPUT_PULLUP);
  pinMode(upButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(tempSignal, OUTPUT);

  while (!myOLED.begin())
  {
    Serial.println("OLED failed");
    delay(1000);
  }

  // Manual screen clearing to get rid of the "speckles"
  myOLED.text(0, 0, "clearingthescreen");
  myOLED.text(0, 10, "clearingthescreen");
  myOLED.text(0, 20, "clearingthescreen");
  myOLED.text(0, 30, "clearingthescreen");
  myOLED.text(0, 40, "clearingthescreen");
  myOLED.display();
  delay(500);
  myOLED.erase();
  myOLED.display();

  Input = tempF;
  PID.SetTunings(Kp, Ki, Kd);
  PID.SetMode(1);
  PID.SetAntiWindupMode(1);
  PID.SetOutputLimits(0, 255);
}

void updateTemps()
{
  // calculates temperature in Kelvin, then changes to C and F
  reading = analogRead(sensorPin);
  voltage = float(reading / 1023) * 5;
  resistance = float(voltage * R) / float(5 - voltage);
  tempK = 1.0 / ((1.0 / T25) + 1.0 / beta * log(resistance / Rtherm));
  tempC = tempK - 273.15;
  tempF = tempC * (9.0 / 5.0) + 32.0;
}

void displayTemps()
{
  // Prints both target temperature and actual temperature
  myOLED.erase();
  char tempStr[10];
  myOLED.text(0, 0, "Temp:");
  dtostrf(tempF, 3, 2, tempStr);
  snprintf(disp, sizeof(disp), "%s F", tempStr);
  myOLED.text(0, 10, disp);
  Serial.println(disp);
  myOLED.text(0, 25, "Target:");
  snprintf(disp, sizeof(disp), "%i F", target);
  myOLED.text(0, 35, disp);
  myOLED.display();

  // Serial.print("PWM signal: ");
  // Serial.println(Output);
}

void loop()
{
  updateTemps();
  switch (currentState)
  {
  case Off:

    // Safety Feature: If the plate is still 120 F or higher, red LED is on
    onPressed = currentOn;
    if (tempF > 120)
    {
      digitalWrite(redLED, HIGH);
    }
    else if (tempF < 120)
    {
      digitalWrite(redLED, LOW);
    }
    analogWrite(tempSignal, 0);

    // Reading the On button to switch to the On state
    currentOn = digitalRead(onButton);
    if (currentOn == LOW && onPressed == HIGH)
    {
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, HIGH);
      onPressed = currentOn;
      buzzer.sound(NOTE_E5, 100);
      buzzer.sound(NOTE_F5, 100);
      buzzer.sound(NOTE_G5, 100);
      currentState = On;
    }

    break;

  case On:
    onPressed = currentOn;
    displayTemps();

    // Simple On/Off controll is used until the error is within 5 degrees F
    if ((target - tempF) > 5)
    {
      analogWrite(tempSignal, 255);
    }
    else if ((target - tempF) <= 5)
    {
      Setpoint = target;
      Input = tempF;
      PID.Compute();
      analogWrite(tempSignal, Output);
    }
    else
    {
      analogWrite(tempSignal, 0);
    }

    // Reading both up and down buttons to change target temperature
    bool currentUp = digitalRead(upButton);
    if (currentUp == LOW && upPressed == HIGH)
    {
      target = target + 5;
      buzzer.sound(NOTE_G5, 50);
    }
    upPressed = currentUp;

    bool currentDown = digitalRead(downButton);
    if (currentDown == LOW && downPressed == HIGH)
    {
      target = target - 5;
      buzzer.sound(NOTE_C5, 50);
    }
    downPressed = currentDown;

    // Reading the On button to switch back to the Off State
    currentOn = digitalRead(onButton);
    if (currentOn == LOW && onPressed == HIGH)
    {
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      onPressed = currentOn;
      buzzer.sound(NOTE_G5, 100);
      buzzer.sound(NOTE_F5, 100);
      buzzer.sound(NOTE_E5, 100);
      myOLED.erase();
      myOLED.display();
      currentState = Off;
    }

    break;

    // 1 Hour shut down (when entering the "On" State)
    unsigned long startTime = millis();
    if ((millis() - startTime) >= 3600000000)
    {
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);
      onPressed = currentOn;
      buzzer.sound(NOTE_G5, 100);
      buzzer.sound(NOTE_F5, 100);
      buzzer.sound(NOTE_E5, 100);
      myOLED.erase();
      myOLED.display();
      currentState = Off;
    }
  }

  delay(50);
}