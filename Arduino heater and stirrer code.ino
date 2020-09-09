/*
   This program is modified from the AutoPID BasicTempControl Example Sketch
  Functions of this program
   1.  Read a 10k NTC thermistor temperature probe as input, potentiometer as setpoint, drive an heater bed as PWM output.
   2.  Drive a stepper using a stepper driver module which acts a stirrer
   3.  Display temperature information on a I2C LCD display 
  Libraries used
   1. Arduino wire library
   2. AutoPID library created by Ryan Downning
   3. LCD03 by Arblaster
*/




#define SerialDebugging 1// change to zero to disable serial debugging
#include <AutoPID.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//pins
#define motor_step 8//can be any pin
#define motor_direction 7 //can be any pin
#define Stepper_speed_pot A0
#define I2C_address 0x3F
#define POT_PIN A1
#define OUTPUT_PIN 3
#define TEMP_PROBE_PIN A2
#define LED_PIN 6

//constants
#define StepperDelayMin 3
#define StepperDelayMax 75
#define TempSetpointMax 70
#define TempSetpointMin 35
#define TEMP_READ_DELAY 300 //can only read digital temp sensor every ~750ms
#define DISPLAY_DELAY 500

//pid settings and gains
#define OUTPUT_MIN 0
#define OUTPUT_MAX 255
#define KP 0
#define KI 0
#define KD 0

double temperature, setPoint, outputVal;
int stepper_delay = 75;


float A = 1.009249522e-03, B = 2.378405444e-04, C = 2.019202697e-07; // 10k thermistor coefficients



//input/output variables passed by reference, so they are updated automatically
AutoPID myPID(&temperature, &setPoint, &outputVal, OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
LiquidCrystal_I2C lcd(I2C_address, 16, 2);


unsigned long lastTempUpdate; //tracks clock time of last temp update
unsigned long lastDisplayUpdate;
unsigned long stepperUpdateTime = 0;
//call repeatedly in loop, only updates after a certain time interval
//returns true if update happened
bool updateTemperature() {
  int rawTempValue=0;
  float R2,logR2,Tk;
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    
     rawTempValue = analogRead(TEMP_PROBE_PIN);
      R2 = 9800 * (1023.0 / (float)rawTempValue - 1.0);
    logR2 = log(R2);
    Tk = (1.0 / (A + B * logR2 + C * logR2 * logR2 * logR2)); // We get the temperature value in Kelvin from this Stein-Hart equation
    temperature = (double)Tk - 273.15;  // Convert Kelvin to Celsius
  
    lastTempUpdate = millis();
    

   
    return true;
  }
  return false;
}

void displayUpdate() // updates I2C display and Serial monitor display if enabled in "SerialDebugging"
{
  if ((millis() - lastDisplayUpdate) > DISPLAY_DELAY) {
    lastDisplayUpdate = millis();
    lcd.clear();
    lcd.print("Setpoint:");
    lcd.print(setPoint);
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Actual temp:");
    lcd.print(temperature);
    lcd.print("C");

    if (SerialDebugging == true)
    {
      Serial.print(millis());
      Serial.print(",");
      Serial.print(temperature);
      Serial.print(",");
      Serial.print(setPoint);
      Serial.print(",");
      Serial.println(outputVal);


    }
  }

}






void setup() {
 
  pinMode(motor_step, OUTPUT);
  pinMode(motor_direction, OUTPUT);
  digitalWrite(motor_direction, HIGH);// move stepper in reverse


  if (SerialDebugging == true)
  {
    Serial.begin(115200);
    Serial.println("millis,Actual_temp,Setpoint,PWM"); // prints debug information in CSV format can be visualised in excel


  }
  pinMode(POT_PIN, INPUT);
  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);


  while (!updateTemperature())
   {  //wait until temp sensor updated
    Serial.println("waiting for temp update");
    delay(100);
    } 

  //if temperature is more than 0.5 degrees below or above setpoint, OUTPUT will be set to min or max respectively
 myPID.setBangBang(0.5);
  //set PID update interval to 500ms
  myPID.setTimeStep(500);

  lcd.begin();

  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.print("Heater +");
  lcd.setCursor(0, 1);
  lcd.print("Stirrer");
  //delay(2000);
 


//setPoint = 38;

}//void setup


void loop() {
  updateTemperature();
  displayUpdate();
 //  Serial.println("loop test");
   //delay(30);
  setPoint = map(analogRead(POT_PIN), 0, 1023, TempSetpointMin, TempSetpointMax);
  stepper_delay = map(analogRead(Stepper_speed_pot),0,1023,StepperDelayMin,StepperDelayMax); // smaller the delay faster the stepper rotates
  myPID.run(); //call every loop, updates automatically at certain time interval
  analogWrite(OUTPUT_PIN, outputVal);
  digitalWrite(LED_PIN, myPID.atSetPoint(1)); //light up LED when we're at setpoint +-1 degree

  if (millis() - stepperUpdateTime >= stepper_delay) // a stepper is used as stirrer and a pulse is sent to the stepper driver every "stepper_delay" ms.
  {
    digitalWrite(motor_step, HIGH);
    delayMicroseconds(20);
    digitalWrite(motor_step, LOW);
    stepperUpdateTime = millis();
   // Serial.println("Stepper pulse");
  }



}//void loop
