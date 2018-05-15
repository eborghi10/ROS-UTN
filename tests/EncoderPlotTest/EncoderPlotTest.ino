#include <AS5048A.h>

/**
 * Tools > Serial Plotter
 */

AS5048A angleSensor(7);

void setup()
{
  Serial.begin(9600);
  angleSensor.begin();
}

void loop()
{
  delay(50);

  // Select one of these possibilities
  double val = angleSensor.getRotationInRadians();
//  double val = angleSensor.getRotationInDegrees();
//  uint16_t val = angleSensor.getRawRotation();
  
  Serial.println(val);
}
