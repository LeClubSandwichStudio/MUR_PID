#include <Servo.h>
//pressure control servos
Servo inputValve;
Servo outputValve;
Servo airSourceInputValve;

// Potentiometer and Servo pins
#define InputValvePin  22            // Pin for input Valve
#define OutputValvePin  23           // Pin for Output Valve
#define AirSourceInputValvePin  9    // Pin for Output Valve

void setup() {
  // put your setup code here, to run once:
  inputValve.attach(InputValvePin);
  outputValve.attach(OutputValvePin);
  airSourceInputValve.attach(AirSourceInputValvePin);
  inputValve.write(0);
  outputValve.write(0);
  airSourceInputValve.write(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  inputValve.write(0);
  outputValve.write(0);
  airSourceInputValve.write(0);
  delay(100);
}
