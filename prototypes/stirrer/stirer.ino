#define THERMISTORPIN A0         //Thermistor pin
#define HEATBED 10               //Heatbed pin
#define FANPIN 9                 //Fan pin
#define BTNPLUS 12               //Plus button pin
#define BTNMINUS 11              //Minus button pin

// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000   
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 21   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000


int samples[NUMSAMPLES];
int motorSpeed = 0;

 
void setup(void) {
  Serial.begin(9600);
  analogReference(EXTERNAL);
  pinMode(FANPIN, OUTPUT);
  pinMode(HEATBED, OUTPUT);
  pinMode(BTNPLUS, INPUT);
  pinMode(BTNMINUS, INPUT);
}
 
void loop(void) {
  uint8_t i;
  float average;
 
  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(THERMISTORPIN);
   delay(10);
  }
 
  // average all the samples out
  average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;
 
  Serial.print("Average analog reading "); 
  Serial.println(average);
 
  // convert the value to resistance
  average = 1023 / average - 1;
  average = SERIESRESISTOR / average;
  Serial.print("Thermistor resistance "); 
  Serial.println(average);
 
  float steinhart;
  steinhart = average / THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= 273.15;                         // convert to C  
  
  //Logic for button press
  if ( (digitalRead(BTNPLUS) == HIGH) && (motorSpeed < 255)) {
      motorSpeed += 5;
      Serial.println("PLUS");
  }
  if ( (digitalRead(BTNMINUS) == HIGH) && (motorSpeed > 5)) {
      motorSpeed -= 5;
      Serial.println("MINUS");
  }
  
  //Stop bead heating if temp is over 40C
  if ( int(steinhart) < 40)
  {
      digitalWrite(HEATBED, LOW);
  }
  else
  {
      digitalWrite(HEATBED, HIGH);
  }
  
  //Set Fan speed to motorSpeed
  analogWrite(FANPIN, motorSpeed);
  
  Serial.print("Temperature "); 
  Serial.print(steinhart);
  Serial.println(" *C");
  Serial.print("Motor speed: ");
  Serial.println(motorSpeed);
  delay(100);
}
