/***************************************************************************
  Copyright 2020 LE CLUB SANDWICH STUDIO SAS
  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at
  http://www.apache.org/licenses/LICENSE-2.0
  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
***************************************************************************/

/***************************************************************************
  !!!                        DISCLAMER                                 !!!
  ALARMS ARE NOT JET WORKING PROPERLY DUE TO TWO REASONS :
  - BME280 library is blocking the loop if the sensor fails
  - I didn't untangle all the program flow...
  BUT :
  It beeps when you exceed peakAlarmLevel and plateauAlarmlevel
  This provides a minimum of over pressure protection...
***************************************************************************/

#include <Servo.h>
#include <Wire.h>
#include <TimerOne.h>      // Inclusion de la librairie Timer2
#include "SparkFunBME280.h"

//pressure control servos
Servo inputValve;
Servo outputValve;
Servo airSourceInputValve;

//BME measured values
BME280 bmePatient;
BME280 bmeAmbient;

// Potentiometer and Servo pins
#define InputValvePin  2             // Pin for input Valve
#define OutputValvePin  3            // Pin for Output Valve
#define AirSourceInputValvePin  4    // Pin for Output Valve

#define   LED 5            // future neopixel Led for user feedback
#define Buzzer   6         // Alarm Buzzer
#define Maintenance   7    // Sets all valves to 0° for maintenance
#define PressureCal   8    // Closes outputValve and opens Input Valve for maximum pressure calibration

// Analog inputs for potentiometers
#define Cycles   A0
#define Ratio   A1
#define Inspiratory   A7
#define Expiratory   A8
#define Peak  A6


//  Please adapt these three values to your Servos and Airsource. We use Futaba S3003 Servos.
int maxPressure = 40;
// treshold values are arbitary set after consulting wikipedia. minimum 600hpa, maximum 1200hpa
float minimumAtmosphericPressure = 600.;
float maximumAtmosphericPressure = 1200.;
// Ring Alarm if differential Pressure goes above that////////////NOT USED IN THIS CODE////////////
float peakAlarmLevel = 40.;
float plateauAlarmLevel = 30.;
float maximumPressure = 1.;
bool peakAlarm = 0;
bool plateauAlarm = 0;

// different timers in our breathing cycle
unsigned long peakTimer = 0;
unsigned long plateauTimer = 0;
unsigned long expirationTime = 0;

// running timers to get asycronous loop
unsigned long cycleZero = 0;
unsigned long bmpZero = 0;

//running cycle flags
//bool peak = false;
bool inspiration = false;
bool expiration = false;
bool pressureMaxCal = false;
bool pressureCal = false;
bool pressureSensorFailure = false;

// Servo positions and others
int ivPos = 0;
int ovPos = 90;
int pvPos = 0;

// position read on potentiometers
int plateauPos = 0;
int baselinePos = 0;
double cycle = 0;
double ratio = 0 ;

//initializing pressure variables
float bmeP = 0.;
float bmeA = 0.;
float differentialP = 0.;
float sensorTare = 0.;

// pressure sensor sample Frequency = 20ms runs smooth on teensy3.2
unsigned long pressureSample = 20;

//PID parameters
double kp = 0 , ki = 0 , kd = 0 ;
double error = 0, I = 0, D = 0;
double lastError = 0;

//time conting variables
unsigned long currentTime = 0, previousTime = 0;
double timeInterval = 0;  // currentTime-previousTime
int peak = 0 ;

void readPot() {
  // Read peaks potentiometer value //////NOT USED IN THIS CODE//////
  int tempI = analogRead(Peak);
  peak = map(tempI, 0, 1023, 20, 50);

  // Calculate total breath cycle length
  cycle = map(analogRead(Cycles), 0, 1023, 6000, 2000);
  // Calculate the time of the inspiration cycle including peak + inspiratory
  ratio = map(analogRead(Ratio), 0, 1023, 0, 1000);
  ratio = ratio / 1000;


  // set inspiratory pressure, can only be opend until a certain point
  plateauPos = map(analogRead(Inspiratory), 0, 1023, 0, (maxPressure));
  // set baseline pressure, can only be opend until a certain point
  baselinePos = map(analogRead(Expiratory), 0, 1023, 5, (maxPressure / 2 ));

}
void initBME() {
  bmePatient.setI2CAddress(0x77);
  if (!bmePatient.isMeasuring()) {
    // Serial.println("Patient side Sensor HS !");
    if (bmePatient.beginI2C() == false) {
      pressureSensorFailure = 1;
      //   Serial.println("Could not find a valid Patient BME280 sensor, check wiring, address, sensor ID!");
    }
    bmePatient.setFilter(1);                //0 to 4 is valid. Filter coefficient. See 3.4.4
    bmePatient.setStandbyTime(0);           //0 to 7 valid. Time between readings. See table 27.
    bmePatient.setTempOverSample(0);        //0 to 16 are valid. 0 disables temp sensing. See table 24.
    bmePatient.setPressureOverSample(4);    //0 to 16 are valid. 0 disables pressure sensing. See table 23.
    bmePatient.setHumidityOverSample(0);    //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    bmePatient.setMode(MODE_NORMAL);
  }

  bmeAmbient.setI2CAddress(0x76);
  if (!bmeAmbient.isMeasuring()) {
    //  Serial.println("Ambient side Sensor HS !");
    if (bmeAmbient.beginI2C() == false) {
      pressureSensorFailure = 1;
      Serial.println("Could not find a valid Ambient BME280 sensor, check wiring, address, sensor ID!");
    }
    bmeAmbient.setFilter(1);              //0 to 4 is valid. Filter coefficient. See 3.4.4
    bmeAmbient.setStandbyTime(0);         //0 to 7 valid. Time between readings. See table 27.
    bmeAmbient.setTempOverSample(0);      //0 to 16 are valid. 0 disables temp sensing. See table 24.
    bmeAmbient.setPressureOverSample(4);  //0 to 16 are valid. 0 disables pressure sensing. See table 23.
    bmeAmbient.setHumidityOverSample(0);  //0 to 16 are valid. 0 disables humidity sensing. See table 19.
    bmeAmbient.setMode(MODE_NORMAL);
  }
  // whait a bit to ensure sensor startup
  delay(20);
}
void updateSensors() {
  // reset timer first for regular intervals!
  bmpZero = millis();

  // read Sensors, get differential
  //bmePatient.takeForcedMeasurement();
  bmeP = bmePatient.readFloatPressure() / 100.;     //(float)bmePatient.readPressure() / 100.;

  //bmeAmbient.takeForcedMeasurement();
  bmeA = bmeAmbient.readFloatPressure() / 100.;     //(float)bmeAmbient.readPressure() / 100.;

  differentialP = bmeP - bmeA;
  if ((differentialP < -30.) || (differentialP > 150.)) {
    differentialP = 0.;
    pressureSensorFailure = 1;
    digitalWrite(Buzzer, HIGH);
  }
  if (!pressureCal) {
    differentialP -= sensorTare;
  }

  // check for sensor failiure, this alarm doesn't care if another alarm is active.
  // this is due to the fact that if one sensor fails the other alarms won't work anymore...
  if ((bmeP <= minimumAtmosphericPressure) || (bmeP >= maximumAtmosphericPressure) || (bmeA <= minimumAtmosphericPressure) || (bmeA >= maximumAtmosphericPressure)) {
    pressureSensorFailure = 1;
    digitalWrite(Buzzer, HIGH);
  }
  // check for realistic pressures if pressureSensorFailure is active
  if (pressureSensorFailure == 1) {
    // check if sensor was reconnected
    initBME();
    if ((bmeP > minimumAtmosphericPressure) && (bmeP < maximumAtmosphericPressure) && (bmeA > minimumAtmosphericPressure) && (bmeA < maximumAtmosphericPressure)) {
      pressureSensorFailure = 0;
      digitalWrite(Buzzer, LOW);
    }
  }
  // This is a more userfriendly graph
  // Inspiratory -- Expiratory -- Pressure
  /////// Generates CSV files //////////
  /*Serial.print(plateauPos); Serial.print(",");
    Serial.print(baselinePos); Serial.print(",");
    Serial.print(differentialP); Serial.print("  ");*/

  // Normal dispaying mode
  Serial.print("Inspiratory: "); Serial.print(plateauPos); Serial.print("  ");
  Serial.print("Expiratory: "); Serial.print(baselinePos); Serial.print("  ");
  Serial.print("differentialP "); Serial.print(differentialP); Serial.print("  ");
    Serial.print("peak "); Serial.print(peak); Serial.print("  ");

  Serial.println();
}

void startupTare() {
  // close input valve and open outputvalve, reset sensorTare
  ivPos = 0;
  inputValve.write(ivPos);
  ovPos = 90;
  outputValve.write(ovPos);
  pressureCal = 1;
  unsigned long timer = millis();
  //read sensors for one second to fill filters, warmup, etc
  while (millis() <= (timer + 1000)) {
    updateSensors();
  }
  timer = millis();
  // calibrate sensorTare for 3 seconds
  while (millis() <= (timer + 3000)) {
    integrateTare();
  }
  pressureCal = 0;
  // close output Valves
  ovPos = 0;
  outputValve.write(ovPos);
}

void integrateTare() {
  // get differential between the two sensors

  if (pressureSample < (millis() - bmpZero)) {
    // sample time before reading the sensor for a regular interval
    updateSensors();
    sensorTare = (0.5 * differentialP + ((1 - 0.5) * sensorTare));
  }
}
// interruption that occurs every 20ms
void updateData() {
  updateSensors(); // Reads measured values
  readPot();       // Reads the configuration of the potentiometers

  if (inspiration) {// if the lung is in inspiration phase
    init_PIDParameters(0.1, 0.0 , 0);
    // PIDs command is efficient for every setPoint in a specific servo opening angle range
    //
    ovPos = computePID(differentialP, plateauPos  , 0, (peak - error) ); //31 is the best max angle to reach maximum inspiration pressure
    // -error allow us to regulate de range of the PID command using a simple proportional
    outputValve.write((peak - error) - ovPos);
    inputValve.write( ovPos + (error));
    airSourceInputValve.write((peak - error) - ovPos);

  }
  else if (expiration) {// if the lung is in expiration phase
    init_PIDParameters(0.5, 0.21 , 0);
    ovPos = computePID(differentialP, baselinePos   , 0, (38 - error));  //44 is the best max angle to reach minimum inspiration pressure
    // -error allow us to regulate de range of the PID command using a simple proportional
    outputValve.write((38 - error) - ovPos);
    inputValve.write(error + ovPos);
    airSourceInputValve.write((38 - error) - ovPos);

  }

}
//Initialize parameters
void init_PIDParameters(double kprop, double kint , double kdiff) {
  kp = kprop , ki = kint , kd = kdiff ;
}

// output calibration
double computePID(double input_, double setPoint, double outMin, double outMax) {
  double output_ = 0 ;
  currentTime = millis();
  timeInterval = (double)(currentTime - previousTime);        //time interval

  error = setPoint - input_;                                  // compute proportional
  I += error * timeInterval;                                  // compute integral
  if (I >= outMax ) I = outMax;                               // condition to limitate the integrals output
  else if (I <= outMin) I = outMin;
  D = (error - lastError) / timeInterval;                     // compute derivative
  output_ = (kp * error + ki * I + kd * D);                   //PID output
  lastError = error;                                          //remember current error
  previousTime = currentTime;                                 //remember current time
  output_ = map(output_ , -100, 100 , outMin, outMax );
  return output_;                                             //have function return the PID output
}



void setup() {
  // put your setup code here, to run once:
  pinMode(Maintenance, INPUT_PULLUP);
  pinMode(PressureCal, INPUT_PULLUP);
  pinMode(Buzzer, OUTPUT);
  digitalWrite(Buzzer, LOW);

  Serial.begin(115200);
  delay(100);
  Wire.begin();
  Wire.setClock(400000);
  initBME();

  inputValve.attach(InputValvePin);
  outputValve.attach(OutputValvePin);
  airSourceInputValve.attach(AirSourceInputValvePin);
  startupTare();

  // initialisation interruption Timer 1
  Timer1.initialize(pressureSample * 1000); //20ms         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(updateData);
  //init servos to release all the pressure
  inputValve.write(90);
  outputValve.write(0);
  airSourceInputValve.write(0);



}


void loop() {
  int time_ = millis();
  while (millis() - time_ <= cycle * (1 - ratio)) {// loop that maintain inspiration time in cycle
    expiration = false;
    inspiration = true;
  }
  D=0;I = 0; // integral reinistialization so it doesnt affect the expiration PID
  time_ = millis();
  while (millis() - time_ <= cycle * (  ratio)) {// loop that maintain expiration time in cycle
    expiration = true;
    inspiration = false;

  }
  D=0; I = 0; // integral reinistialization so it doesnt affect the inspitation PID



}
