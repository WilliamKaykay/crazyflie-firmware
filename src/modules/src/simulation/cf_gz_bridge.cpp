#include "cf_gz_bridge.hpp"
extern "C"
{
    #include "estimator.h"
    #include "stabilizer_types.h"
}

cfGzBridge::cfGzBridge(const char *world_name, const char *model_name, const char *model_type, const char *init_pose_str) :
    world_name_(world_name),
    model_name_(model_name),
    model_type_(model_type),
    init_pose_str_(init_pose_str)
{

}

void cfGzBridge::init()
{
    gz::msgs::EntityFactory ef;

    ef.set_sdf_filename(model_type_ + "/model.sdf");
    ef.set_name(model_name_);
    ef.set_allow_renaming(false);

    std::string create_service = "/world/" + world_name_ + "/create";

    bool gz_success = false;
    gz::msgs::Boolean gz_reply;
    bool result;

    while (gz_success == false)
    {
        if (_node.Request(create_service, ef, 1000, gz_reply, result)) {
            if (!gz_reply.data() || !result) {
                // TODO: Add error handling
                std::cout << "Failed to create model" << std::endl;
                return;
            } else {
                gz_success = true;
            }
        } else {
            std::cout << "Gazebo create entity factory timed out. Retrying in 2 seconds." << std::endl;
            // TODO: Add sleep function
        }
        
    }


    // odom subscriber
    std::string odom_topic = "/model/" + model_name_ + "/odom";
    if (!_node.Subscribe(odom_topic, &cfGzBridge::odomCallback, this)) {
        // TODO: Error handling
        return;
    }

	std::string imu_topic = "/world/" + world_name_ + "/model/" + model_name_ + "/link/base_link/sensor/imu_sensor/imu";
    if (!_node.Subscribe(imu_topic, &cfGzBridge::imuCallback, this)) {
        // TODO: Error handling
        return;
    }
}

void cfGzBridge::odomCallback(const gz::msgs::Odometry &odom_msg)
{
    poseMeasurement_t ext_pose;
    float extPosStdDev = 0.01;
    float extQuatStdDev = 4.5e-3;

    ext_pose.x = odom_msg.pose().position().x();
    ext_pose.y = odom_msg.pose().position().y();
    ext_pose.z = odom_msg.pose().position().z();
    ext_pose.quat.w = odom_msg.pose().orientation().w();
    ext_pose.quat.x = odom_msg.pose().orientation().x();
    ext_pose.quat.y = odom_msg.pose().orientation().y();
    ext_pose.quat.z = odom_msg.pose().orientation().z();
    ext_pose.stdDevPos = extPosStdDev;
    ext_pose.stdDevQuat = extQuatStdDev;

    estimatorEnqueuePose(&ext_pose);
}

void cfGzBridge::imuCallback(const gz::msgs::IMU &imu_msg)
{
    poseMeasurement_t ext_pose;
    float extPosStdDev = 0.01;
    float extQuatStdDev = 4.5e-3;

    estimatorEnqueuePose(&ext_pose);
}