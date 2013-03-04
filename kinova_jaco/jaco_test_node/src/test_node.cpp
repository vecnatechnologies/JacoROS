#include <jaco_api/jaco_api.hpp>
#include <ros/ros.h>
#include <string>

int main(int argc, char** argv)
{
    using namespace jaco_api;

    ros::init(argc, argv, "jaco_test_node");
    ros::NodeHandle n("~");

    std::string key;
    n.getParam("jaco_key", key);

    JacoArm* arm;
    try
    {
        arm = new JacoArm(key.c_str());
    } 
    catch (JacoArmException e)
    {
        ROS_ERROR("Jaco API exception: %s", e.what());
        return -1;
    }

    for (int i = 0; i < 3; ++i)
        ROS_INFO("Joint angle %i: %f", i, arm->state().joints_[i].angle_);

    ros::spin();
}

