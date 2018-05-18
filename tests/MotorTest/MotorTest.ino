#include <DCMotor.h>

// #define RIGHT_WHEEL 1

#ifdef RIGHT_WHEEL
  const int EN = 12;
  const int D0 = A3;
  const int D1 = A4;
#else
  const int EN = 11;
  const int D0 = A0;
  const int D1 = A1;
#endif

DCMotor dcMotor(EN, D0, D1);

void setup(){
  Serial.begin(9600);

#ifdef RIGHT_WHEEL
  dcMotor.setClockwise(false);
#else
#endif
}

void loop(){
  if (Serial.available() > 0) {
    long vel = Serial.parseInt();
    dcMotor.setSpeed(vel);
#ifdef RIGHT_WHEEL
    Serial.print("[RIGHT] ");
#else
    Serial.print("[LEFT] ");
#endif    
    Serial.println(vel);
  }
}
