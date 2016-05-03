

/*
 * This file is part of the first Serbian BioHack Academy Code. It is a modified version of Waag Society's BioHack Academy Code adjusted for diferrent hardware and temperature/timing control.
 * BioHack Academy Code is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
 * BioHack Academy Code is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License along with this Code. If not, see <http://www.gnu.org/licenses/>.
 * The thermocycler is perhaps the most complex device in the BioHack Academy thus far. The logic of the code is explained in the lecture.
 * Please make sure you understand the logic, before changing the code.
 */

/*
 * SPECIFIC HARDWARE:
 * Temperature Sensor: The TMP36 is an easy-to-use temperature sensor that outputs a voltage that's proportional to the ambient temperature. More information on the sensor is available in the datasheet: http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Sensors/Temp/TMP35_36_37.pdf
 */

//  Libraries
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//  LCD
// Set the LCD address to 0x3F for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);

// Temperature read
int val;               // Create an integer variable to temporarily store infrormation
double currentTemp;    // Variable to hold the current temperature value

//  PCR Settings
String stageNames[3] = { "Denat", "Anneal", "Elon" };   // Names of Stages
int tempSettings[3] = { 55, 35, 45};                       // Temperatures of each stage
//int tempSettings[3] = { 96, 55, 75};                       // Temperatures of each stage
int timeSettings[3] = { 30, 40, 50};                       // Duration of each stage
int cycleSetting = 20;                                   // Max number of cycles

// Pins
#define coolPin 6                                       // Pin for the mosfet that controls the Peltier element
#define heatPin 5                                       // Pin for the mosfet that controls the PCB heat pad

// PCR cycling variables
int stageTemp = 0;                                      // Target temperature of the current stage
int stageTime = 0;                                      // Duration of current stage
int cycleCounter = 1;                                   // Counter of number of cycles completed
int currentStage = 0;                                   // Current stage
unsigned long currentStageStartTime = 0;                // Beginning of the current Stage
int currentState = 1;                                   // In each stage, go through 3 states: Ramping, Steady, Cooling
int toggleCooling = 0;                                  // Toggle to skip or execute Stage 3: Cooling

//Power at which heatbed will be heated max is 255 min 0
int heatPower = 240;

// Pins
const int temperaturePin = 0;                           // TMP36 sensor attached to analog 0 pin

//  Set the initial STATE of the machine
//  In this code we will switch operation modes, from (programming time, to programming temp) x3, to cycling, to stopping/slowing down
#define STATE_CYCLING 1
#define STATE_STOP 2
byte state = STATE_CYCLING;

// Variables needed for keeping track of time
uint32_t lastTick = 0;                                  // Global Clock variable
int LCDTime = 0;                                        // Time tracker for LCD update
// Useful Constants
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24L)
// Useful Macros for getting elapsed time
#define numberOfSeconds(_time_) (_time_ % SECS_PER_MIN)
#define numberOfMinutes(_time_) ((_time_ / SECS_PER_MIN) % SECS_PER_MIN)
#define numberOfHours(_time_) (( _time_% SECS_PER_DAY) / SECS_PER_HOUR)
#define elapsedDays(_time_) ( _time_ / SECS_PER_DAY)

// Setup function, this code is only executed once
void setup() {
  // Update clock
  lastTick = millis();

  // Initialize I2C connection with the LCD screen
  Wire.begin();

  // Open serial connection with the computer and print a message
  Serial.begin(9600);
  Serial.println(F("srpski PCR"));

  // cooling and heating turned off
  pinMode(coolPin, OUTPUT);
  pinMode(heatPin, OUTPUT);
  digitalWrite(coolPin, LOW);
  //digitalWrite(heatPin, LOW);
  analogWrite(heatPin, heatPower);

  // Initialize the LCD and print a message
  lcd.begin(16,2);
  lcd.setCursor(0,0);
  lcd.print(F("BioHacking"));
  lcd.setCursor(0,1);
  lcd.print(F("srpski PCR"));
  delay(2000);

}

// Loop function, this code is constantly repeated
void loop() {
  // Update clock
  uint32_t time = millis();                               // current time since start of sketch
  uint16_t dt = time-lastTick;                            // difference between current and previous time
  lastTick = time;

  // Read temperature
  currentTemp = getTemperature(temperaturePin);                     // read the temperature sensorand convert to Celcius

  // Print temperature to computer via Serial
  Serial.print("Current Temperature: ");
  Serial.println(currentTemp);

    // Do machine logic
  machineUpdate(dt);

  // Wait 200 microsconds before restarting the loop
  delay(2000);
}

//  machineUpdate, this function checks in which STATE the device is and executes the code that belongs to that STATE
//  starting with STATEs to allow the user to set the PCR paramters of the device, such as temperate and time of each stage and the number of cycles
//  the next STATE is to execute the PCR protocol
//  final STATE to stop the machine and reset the settings

void machineUpdate(uint16_t dt) {

  // State Cyling is the state in which the thermocycler is running
  if(state == STATE_CYCLING) {
    LCDTime += dt; // Update LCD once every second
    if(LCDTime > 1000) {
      LCDTime = 0;

      // Print to LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Ciklus "));
      lcd.print(cycleCounter);
      lcd.print("/");
      lcd.print(cycleSetting);
      lcd.setCursor(0,1);
      lcd.print(F("Temp "));
      lcd.print(currentTemp);
    }

    if(cycleCounter < cycleSetting) { // Check whether we have not completed all cycles
    // If not, go through 3 PCR stages: Denat, Anneal and Elon
      Serial.print("Current Stage: ");
      Serial.println(currentStage);

      if(currentStage == 3) {
        currentStage = 0;
        cycleCounter++; // After completion of all three PCR stage, add 1 to the cycleCounter
      }
      stageTemp = tempSettings[currentStage]; // set the PCR stage target temperature
      stageTime = timeSettings[currentStage]; // set the PCR stage time
      Serial.print("Target Temp: ");
      Serial.println(stageTemp);
      Serial.print("Holding Time: ");
      Serial.println(stageTime);
      lcd.setCursor(14,0);
      if(currentStage==0){
        lcd.print(F("DE"));
      }
      if(currentStage==1){
        lcd.print(F("AN"));
      }
      if(currentStage==2){
        lcd.print(F("EL"));
      }

    }
    else { // all cycles are done!

      // Print a message to the LCD
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print(F("Gotovo!"));
      lcd.setCursor(0,1);
      lcd.print(F("Restartujte PCR"));
      stateChange(STATE_STOP);
    }
  }

  // StateStop stops the cycling
  if(state == STATE_STOP) {

    // Reset PCR cycling variables
    stageTemp = 0;       // Target temperature of the current stage
    stageTime = 0;       // Duration of current stage
    cycleCounter = 0;    // Counter of number of cycles completed
    currentStage = 0;    // Go through 3 stages: Denat, Anneal and Elon
    currentStageStartTime = 0; // Beginning of the current Stage
    currentState = 0;    // In each stage, go through 3 states: Ramping, Steady, Cooling

    // Stop heating and cooling
    digitalWrite(coolPin, LOW);
    //digitalWrite(heatPin, LOW);
    analogWrite(heatPin, 0);

  }

/* *******************************************************
/  The actual PCR cycles
/  The code above manages the 3 stages: Denat, Anneal, Elongate
/  Now we need to go through 3 states: Ramping the temperature up, maintain a Steady State, and Cooling if necessary
*/
  Serial.print(" State: "); Serial.println(currentState);
  Serial.print(" Temp: "); Serial.println(stageTemp);
  Serial.print(" Time: "); Serial.println(stageTime);
  Serial.print(" Past: "); Serial.println((millis()-currentStageStartTime)/1000);

  if(currentState == 1) {

    // HEATING OR COOLING
    if(currentTemp < stageTemp ){

      Serial.println(F("Heating..."));
      lcd.setCursor(15,1);
      lcd.print(F("H"));

      //digitalWrite(heatPin, HIGH);
      analogWrite(heatPin, heatPower);
      digitalWrite(coolPin, LOW);
    }

    if(currentTemp > stageTemp ){
      Serial.println(F("Cooling..."));
      lcd.setCursor(15,1);
      lcd.print(F("C"));

      //digitalWrite(heatPin, HIGH);
      analogWrite(heatPin, 0);
      digitalWrite(coolPin, HIGH);
    }

    if(abs(currentTemp - stageTemp) < 2){
      Serial.println(F("Reached Steady State"));
      currentStageStartTime = millis(); // Set timer
      currentState = 2; // Continue STEADY STATE stage
    }
  }

  if(currentState == 2) {
    //STEADY STATE
    if((millis()-currentStageStartTime)/1000 < stageTime){ // Check whether we completed the state
      if(currentTemp > stageTemp){ // Temperature too high, so switch off heater
        //digitalWrite(heatPin, LOW);
        analogWrite(heatPin, 0);
      }else{ // Temperature too low, so turn on heater
        //digitalWrite(heatPin, HIGH);
        analogWrite(heatPin, heatPower);
        Serial.println(F("Heating..."));
        lcd.setCursor(15,1);
        lcd.print(F("H"));
      }
    }
    else {
      Serial.print(F("Steady state finished. Temp: "));
      Serial.println(currentTemp);
      //digitalWrite(heatPin, LOW);
      analogWrite(heatPin, 0);
      currentState = 1; // Back to HEATING/COOLING state
      currentStage++; // Go to the next stage
    }
  }

}

/* *******************************************************
/  stateChange switches the machine logic from one state to another
*/
void stateChange(byte newstate) {
  // set new state
  state = newstate;
}

//  time converts seconds to minutes:seconds format
String time(int val){
  // calculate number of days, hours, minutes and seconds
  int days = elapsedDays(val);
  int hours = numberOfHours(val);
  int minutes = numberOfMinutes(val);
  int seconds = numberOfSeconds(val);

  String returnval = "";

  // digital clock display of current time
  returnval = printDigits(minutes) + ":" + printDigits(seconds) + "   ";

  // return value
  return returnval;
}

//  printDigits adds an extra 0 if the integer is below 10
String printDigits(int digits){
  // utility function for digital clock display: prints colon and leading 0
  String returnval = "";
  if(digits < 10)
    returnval += "0";
  returnval += digits;

  return returnval;
}

float getTemperature(int pin)
{
  // This function has one input parameter, the analog pin number
  // to read. It returns a floating-point value, which is the
  // temperature in Celcius based true voltage on that pin (0 to 5V)
  // and calculation from sensor datasheet
  int reading = analogRead(pin);
  //Serial.println(reading);

  // converting that reading to voltage, for 3.3v arduino use 3.3
  float voltage = reading * 0.004882814;

  // print out the voltage
 //Serial.print(voltage); Serial.println(" volts");

  // now print out the temperature
  float temperatureC = (voltage - 0.5) * 100 ;  //converting from 10 mv per degree wit 500 mV offset
                                                //to degrees ((voltage - 500mV) times 100)
  //Serial.print(temperatureC); Serial.println(" degrees C");

  return (temperatureC);
}
