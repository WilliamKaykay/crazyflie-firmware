// File: cf_gz_bridge.cpp
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Created on 04-13-2024

#include "cf_gz_bridge.h"
extern "C"
{
    #include "estimator.h"
    #include "stabilizer_types.h"
    #include "imu_types.h"
    #include "physicalConstants.h"
    #include "debug.h"
    #include "system.h"
    #include "sensors_gz.h"
    #include "static_mem.h"
}

#define SENSORS_DEG_PER_LSB_CFG                           (float)((2 * 2000.0) / 65536.0)
#define SENSORS_G_PER_LSB_CFG                             (float)((2 * 16) / 65536.0)
#define DEG_TO_RAD_CF (M_PI/180.0)
#define RAD_TO_DEG_CF (180.0/M_PI)



cfGzBridge::cfGzBridge(const char *world_name, const char *model_name, const char *model_type, const char *init_pose_str) :
    world_name_(world_name),
    model_name_(model_name),
    model_type_(model_type),
    init_pose_str_(init_pose_str)
{
    // pthread_mutex_init(&node_mutex_, nullptr);
    pthread_mutex_init(&pose_mutex_, nullptr);
    pthread_mutex_init(&sensors_mutex_, nullptr);
}

cfGzBridge::~cfGzBridge()
{
    for (auto &subscribed_topic : node_.SubscribedTopics()) {
		node_.Unsubscribe(subscribed_topic);
	}
}

void cfGzBridge::init()
{
    // gz::msgs::EntityFactory ef;

    // ef.set_sdf_filename(model_type_ + "/model.sdf");
    // ef.set_name(model_name_);
    // ef.set_allow_renaming(false);

    // std::string create_service = "/world/" + world_name_ + "/create";

    // bool gz_success = false;
    // gz::msgs::Boolean gz_reply;
    // bool result;

    // while (gz_success == false)
    // {
    //     if (node_.Request(create_service, ef, 1000, gz_reply, result)) {
    //         if (!gz_reply.data() || !result) {
    //             // TODO: Add error handling
    //             std::cout << "Failed to create model" << std::endl;
    //             return;
    //         } else {
    //             gz_success = true;
    //         }
    //     } else {
    //         std::cout << "Gazebo create entity factory timed out. Retrying in 2 seconds." << std::endl;
    //         // TODO: Add sleep function
    //     }
        
    // }

    // odom subscriber
    std::string odom_topic = "/model/" + model_name_ + "/odometry";
    if (!node_.Subscribe(odom_topic, &cfGzBridge::odomCallback, this)) {
        // TODO: Error handling
        return;
    }

	std::string imu_topic = "/world/" + world_name_ + "/model/" + model_name_ + "/link/base_link/sensor/imu_sensor/imu";
    if (!node_.Subscribe(imu_topic, &cfGzBridge::imuCallback, this)) {
        // TODO: Error handling
        return;
    }

    DEBUG_PRINT("Initialized cfGzBridgeObj \n");
}

void cfGzBridge::odomCallback(const gz::msgs::Odometry &odom_msg)
{
    pthread_mutex_lock(&pose_mutex_);
    // poseMeasurement_t ext_pose;
    float extPosStdDev = 0.01;
    float extQuatStdDev = 4.5e-3;

    poseData_.x = odom_msg.pose().position().x();
    poseData_.y = odom_msg.pose().position().y();
    poseData_.z = odom_msg.pose().position().z();
    poseData_.quat.w = odom_msg.pose().orientation().w();
    poseData_.quat.x = odom_msg.pose().orientation().x();
    poseData_.quat.y = odom_msg.pose().orientation().y();
    poseData_.quat.z = odom_msg.pose().orientation().z();
    poseData_.stdDevPos = extPosStdDev;
    poseData_.stdDevQuat = extQuatStdDev;
    // std::cout << "Ext pose: " << ext_pose.x << " " << ext_pose.y << " " << ext_pose.z << std::endl;
    // estimatorEnqueuePose(&ext_pose);
    pthread_mutex_unlock(&pose_mutex_);
    poseDataAvailableCallback();
}

void cfGzBridge::imuCallback(const gz::msgs::IMU &imu_msg)
{
    pthread_mutex_lock(&sensors_mutex_);
    gyroData_ = {
        .x = static_cast<int16_t>(imu_msg.angular_velocity().x() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE),
        .y = static_cast<int16_t>(imu_msg.angular_velocity().y() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE),
        .z = static_cast<int16_t>(imu_msg.angular_velocity().z() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE)
    };

    accData_ = {
        .x = static_cast<int16_t>(imu_msg.linear_acceleration().x() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF),
        .y = static_cast<int16_t>(imu_msg.linear_acceleration().y() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF),
        .z = static_cast<int16_t>(imu_msg.linear_acceleration().z() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF)
    };
    // std::cout << "Acc: " << accData_.x << " " << accData_.y << " " << accData_.z << std::endl;
    pthread_mutex_unlock(&sensors_mutex_);
    gz_DataCallback();
}

void cfGzBridge::imuAcquireData(Axis3i16 *accData, Axis3i16 *gyroData)
{
    pthread_mutex_lock(&sensors_mutex_);
    *accData = accData_;
    *gyroData = gyroData_;
    pthread_mutex_unlock(&sensors_mutex_);
}

void cfGzBridge::poseAcquireData(poseMeasurement_t *poseData)
{
    pthread_mutex_lock(&pose_mutex_);
    *poseData = poseData_;
    pthread_mutex_unlock(&pose_mutex_);
}

#ifdef __cplusplus
extern "C" {
#endif

static bool isInit = false;

static cfGzBridge cfGzBridgeObj("crazysim_default", "cf_1", "crazyflie", "pose");

STATIC_MEM_TASK_ALLOC(cfGzBridgeTask, SENSORS_TASK_STACKSIZE);

void acquireImuData(Axis3i16 *accData, Axis3i16 *gyroData)
{
    cfGzBridgeObj.imuAcquireData(accData, gyroData);
    // DEBUG_PRINT("Acquired IMU data: Acc: %d %d %d, Gyro: %d %d %d \n", accData->x, accData->y, accData->z, gyroData->x, gyroData->y, gyroData->z);
}

void acquirePoseData(poseMeasurement_t *poseData)
{
    cfGzBridgeObj.poseAcquireData(poseData);
}

static void cfGzBridgeTask(void* param)
{
    // systemWaitStart();

    cfGzBridgeObj.init();

    TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        vTaskDelayUntil(&lastWakeTime, M2T(1));
    }
    
    // sleep(5000);
    // DEBUG_PRINT("Shutting down Gazebo Bridge \n");
}

void cfGzBridgeInit(void)
{
    if (isInit)
    {
        return;
    }

    STATIC_MEM_TASK_CREATE(cfGzBridgeTask, cfGzBridgeTask, "GAZEBO BRIDGE", NULL, SENSORS_TASK_PRI);

    isInit = true;
}

void __attribute__((weak)) gz_DataCallback(void) {}

#ifdef __cplusplus
}
#endif