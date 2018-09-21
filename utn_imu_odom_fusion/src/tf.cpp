#include <utn_imu_odom_fusion/imu_tf.hpp>
#include <utn_imu_odom_fusion/odom_tf.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    ImuTf* imu_tf = new ImuTf(&nh);
    OdomTf* odom_tf = new OdomTf(&nh);

    ros::Rate r(60); // 60 hz
    while (ros::ok()) {
      odom_tf->publishWheels();
      r.sleep();
    }

    return 0;
}
