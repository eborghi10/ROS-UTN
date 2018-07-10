#include <ros.h>
#include <AS5048A.h>
#include <DCMotor.h>
#include <spline.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>

/**
 * Code used for generating the Spline data
 * 
 * 1) Gather REAL information of the encoders
 * 
 * $ roslaunch robot_control serial_node.launch
 * $ rostopic pub <MOTOR_TOPIC> std_msgs/UInt16 "data: 170"
 * $ rostopic echo <ENCODER_TOPIC>
 * 
 * NOTE: FOR THE RIGHT MOTOR, USE NEGATIVE VELOCITY IN ORDER TO GET A POSITIVE SLOPE!
 * 
 * 2) Copy one sequence of values from 0 to 2PI and past them into the real array
 * 
 * 3) Obtain the expected values using this command in Octave:
 * 
 * $ linspace(0, 2*pi, XX) where XX is the number of points obtained on the step #1
 * 
 * 4) Copy them into the ideal array
 * 
 * 5) Comment the FETCH_DATA macro in order to test the values
 * 
 */

// Comment this line to gather data from the left wheel encoder
// #define RIGHT_WHEEL 1
// #define FETCH_DATA 1

#ifdef RIGHT_WHEEL
  AS5048A angleSensor(7);
  DCMotor dcMotor(12, A3, A4);
#else
  AS5048A angleSensor(6);
  DCMotor dcMotor(11, A0, A1);
#endif

void dcmotor_cb(const std_msgs::Int16& cmd_msg){
    dcMotor.setSpeed(cmd_msg.data);
}

ros::NodeHandle  nh;
std_msgs::Float32 encoder_msg;

#ifdef RIGHT_WHEEL
  ros::Publisher encoder_pub("encoder/right", &encoder_msg);
  ros::Subscriber<std_msgs::Int16> subMotor("output/right", dcmotor_cb);
  const double real[] = {0.000767084071413, 0.137308046222, 0.270780652761, 0.399267226458, 0.516247570515, 0.634378492832, 0.746756315231, 0.857983529568, 0.958471536636, 1.05320632458, 1.13950335979, 1.22349905968, 1.30442643166, 1.38842213154, 1.47510266304, 1.56024897099, 1.6496142149, 1.7393630743, 1.83678281307, 1.94110620022, 2.0569357872, 2.17698454857, 2.30048513412, 2.41976666451, 2.53482937813, 2.6479742527, 2.76955699921, 2.89267396927, 3.01732516289, 3.14005875587, 3.26164150238, 3.37900519371, 3.49675250053, 3.61181521416, 3.73032975197, 3.84155702591, 3.95240068436, 4.05902528763, 4.16104745865, 4.26153564453, 4.36317396164, 4.45982694626, 4.55456161499, 4.6443104744, 4.73444271088, 4.82495880127, 4.91739225388, 5.01097679138, 5.10494422913, 5.19852876663, 5.2947974205, 5.39260101318, 5.49117136002, 5.59319353104, 5.7086391449, 5.83022260666, 5.95333909988, 6.07760715485, 6.20494270325};
  const double ideal[] = {0.00000, 0.10833, 0.21666, 0.32499, 0.43332, 0.54165, 0.64998, 0.75832, 0.86665, 0.97498, 1.08331, 1.19164, 1.29997, 1.40830, 1.51663, 1.62496, 1.73329, 1.84162, 1.94995, 2.05828, 2.16662, 2.27495, 2.38328, 2.49161, 2.59994, 2.70827, 2.81660, 2.92493, 3.03326, 3.14159, 3.24992, 3.35825, 3.46658, 3.57492, 3.68325, 3.79158, 3.89991, 4.00824, 4.11657, 4.22490, 4.33323, 4.44156, 4.54989, 4.65822, 4.76655, 4.87489, 4.98322, 5.09155, 5.19988, 5.30821, 5.41654, 5.52487, 5.63320, 5.74153, 5.84986, 5.95819, 6.06652, 6.17485, 6.28319};
#else
  ros::Publisher encoder_pub("encoder/left", &encoder_msg);
  ros::Subscriber<std_msgs::Int16> subMotor("output/left", dcmotor_cb);
  const double real[] = {0.0617502629757, 0.163772448897, 0.293793171644, 0.482495874166, 0.770919501781, 1.1652007103, 1.59553480148, 1.9292165041, 2.13863039017, 2.27478790283, 2.37681007385, 2.44738173485, 2.50721430779, 2.5643620491, 2.62150979042, 2.68210935593, 2.75229763985, 2.83399200439, 2.92987751961, 3.04378938675, 3.17227602005, 3.3118853569, 3.46223402023, 3.62063670158, 3.76830029488, 3.91519713402, 4.0594086647, 4.19748401642, 4.33824396133, 4.48130512238, 4.6159286499, 4.75668811798, 4.90243434906, 5.04971456528, 5.19392633438, 5.33698701859, 5.46624088287, 5.58283758163, 5.69214725494, 5.78726577759, 5.87241172791, 5.94796991348, 6.01278829575, 6.07722330093, 6.13858985901, 6.20455932617, 6.27321338654};
  const double ideal[] = {0.00000, 0.13659, 0.27318, 0.40977, 0.54636, 0.68295, 0.81955, 0.95614, 1.09273, 1.22932, 1.36591, 1.50250, 1.63909, 1.77568, 1.91227, 2.04886, 2.18546, 2.32205, 2.45864, 2.59523, 2.73182, 2.86841, 3.00500, 3.14159, 3.27818, 3.41477, 3.55137, 3.68796, 3.82455, 3.96114, 4.09773, 4.23432, 4.37091, 4.50750, 4.64409, 4.78068, 4.91728, 5.05387, 5.19046, 5.32705, 5.46364, 5.60023, 5.73682, 5.87341, 6.01000, 6.14659, 6.28319};
#endif
// Error checking
static_assert(sizeof(real) == sizeof(ideal), "Malformed data. They must have the same amount of data.");
// Generate the Spline
Spline<double> spline(real, ideal, sizeof(ideal)/sizeof(double));

void setup()
{
#ifdef RIGHT_WHEEL
  dcMotor.setClockwise(false);
#else
#endif
  angleSensor.begin();
  nh.initNode();
  nh.subscribe(subMotor);
  nh.advertise(encoder_pub);
}

void loop()
{
  double radians = angleSensor.getRotationInRadians();
#ifdef FETCH_DATA
  encoder_msg.data = radians;
  delay(15);
#else
  encoder_msg.data = spline.value(radians);
#endif
  encoder_pub.publish( &encoder_msg );

  nh.spinOnce();
}
