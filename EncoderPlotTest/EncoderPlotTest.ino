#include <AS5048A.h>

/**
   Tools > Serial Plotter
*/

AS5048A angleSensor(7);

void setup()
{
  Serial.begin(9600);
  angleSensor.init();
}

void loop()
{
  delay(50);

  //	float val = angleSensor.getRotation();
  uint16_t val = angleSensor.getRawRotation();
  Serial.println(val);
}
