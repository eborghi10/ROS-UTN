#include <utn_imu_odom_fusion/imu_tf.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, ros::this_node::getName());
    ros::NodeHandle nh;

    ImuTf* imu_tf = new ImuTf(&nh);
    imu_tf->publishTf();

    return 0;
}