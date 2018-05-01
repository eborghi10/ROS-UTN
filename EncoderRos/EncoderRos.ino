#include <ros.h>
#include <std_msgs/UInt16.h> // Usar sensor_msgs/JointState

const int PIN = A2;

ros::NodeHandle  nh;
std_msgs::UInt16 msg;
ros::Publisher pub("encoder", &msg);

uint16_t oldState;
uint16_t newState;

void setup() {
  pinMode(PIN, INPUT);
  nh.initNode();
  nh.advertise(pub);
  oldState = digitalRead(PIN);
}

void loop() {
  // Envia cada flanco creciente
  newState = digitalRead(PIN);
  if(oldState != newState)
  {
    if(newState)
    {
      // Agregar timestamp
      msg.data = newState;
      pub.publish( &msg );
    }
    oldState = newState;
  }
  nh.spinOnce();
  delay(200);
}
