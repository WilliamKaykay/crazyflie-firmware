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
#include <thread>

#define SENSORS_DEG_PER_LSB_CFG                           (float)((2 * 2000.0) / 65536.0)
#define SENSORS_G_PER_LSB_CFG                             (float)((2 * 16) / 65536.0)
#define DEG_TO_RAD_CF (M_PI/180.0)
#define RAD_TO_DEG_CF (180.0/M_PI)



// cfGzBridge::cfGzBridge(const char *world_name, const char *model_name, const char *model_type, const char *init_pose_str) :
//     world_name_(world_name),
//     model_name_(model_name),
//     model_type_(model_type),
//     init_pose_str_(init_pose_str)
// {
//     // pthread_mutex_init(&node_mutex_, nullptr);
//     pthread_mutex_init(&pose_mutex_, nullptr);
//     pthread_mutex_init(&sensors_mutex_, nullptr);
// }

// cfGzBridge::~cfGzBridge()
// {
//     for (auto &subscribed_topic : node_.SubscribedTopics()) {
// 		node_.Unsubscribe(subscribed_topic);
// 	}
// }

// void cfGzBridge::init()
// {
//     // gz::msgs::EntityFactory ef;

//     // ef.set_sdf_filename(model_type_ + "/model.sdf");
//     // ef.set_name(model_name_);
//     // ef.set_allow_renaming(false);

//     // std::string create_service = "/world/" + world_name_ + "/create";

//     // bool gz_success = false;
//     // gz::msgs::Boolean gz_reply;
//     // bool result;

//     // while (gz_success == false)
//     // {
//     //     if (node_.Request(create_service, ef, 1000, gz_reply, result)) {
//     //         if (!gz_reply.data() || !result) {
//     //             // TODO: Add error handling
//     //             std::cout << "Failed to create model" << std::endl;
//     //             return;
//     //         } else {
//     //             gz_success = true;
//     //         }
//     //     } else {
//     //         std::cout << "Gazebo create entity factory timed out. Retrying in 2 seconds." << std::endl;
//     //         // TODO: Add sleep function
//     //     }
        
//     // }

//     // odom subscriber
//     std::string odom_topic = "/model/" + model_name_ + "/odometry";
//     if (!node_.Subscribe(odom_topic, &cfGzBridge::odomCallback, this)) {
//         // TODO: Error handling
//         return;
//     }

// 	std::string imu_topic = "/world/" + world_name_ + "/model/" + model_name_ + "/link/base_link/sensor/imu_sensor/imu";
//     if (!node_.Subscribe(imu_topic, &cfGzBridge::imuCallback, this)) {
//         // TODO: Error handling
//         return;
//     }

//     // DEBUG_PRINT("Initialized cfGzBridgeObj \n");
// }

// void cfGzBridge::odomCallback(const gz::msgs::Odometry &odom_msg)
// {
//     pthread_mutex_lock(&pose_mutex_);
//     // poseMeasurement_t ext_pose;
//     float extPosStdDev = 0.01;
//     float extQuatStdDev = 4.5e-3;

//     poseData_.x = odom_msg.pose().position().x();
//     poseData_.y = odom_msg.pose().position().y();
//     poseData_.z = odom_msg.pose().position().z();
//     poseData_.quat.w = odom_msg.pose().orientation().w();
//     poseData_.quat.x = odom_msg.pose().orientation().x();
//     poseData_.quat.y = odom_msg.pose().orientation().y();
//     poseData_.quat.z = odom_msg.pose().orientation().z();
//     poseData_.stdDevPos = extPosStdDev;
//     poseData_.stdDevQuat = extQuatStdDev;
//     // std::cout << "Ext pose: " << poseData_.x << " " << poseData_.y << " " << poseData_.z << std::endl;
//     // estimatorEnqueuePose(&ext_pose);
//     pthread_mutex_unlock(&pose_mutex_);
//     poseDataAvailableCallback();
// }

// void cfGzBridge::imuCallback(const gz::msgs::IMU &imu_msg)
// {
//     pthread_mutex_lock(&sensors_mutex_);
//     gyroData_ = {
//         .x = static_cast<int16_t>(imu_msg.angular_velocity().x() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE),
//         .y = static_cast<int16_t>(imu_msg.angular_velocity().y() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE),
//         .z = static_cast<int16_t>(imu_msg.angular_velocity().z() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE)
//     };

//     accData_ = {
//         .x = static_cast<int16_t>(imu_msg.linear_acceleration().x() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF),
//         .y = static_cast<int16_t>(imu_msg.linear_acceleration().y() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF),
//         .z = static_cast<int16_t>(imu_msg.linear_acceleration().z() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF)
//     };
//     // std::cout << "Acc: " << accData_.x << " " << accData_.y << " " << accData_.z << std::endl;
//     pthread_mutex_unlock(&sensors_mutex_);
//     gz_DataCallback();
// }

// void cfGzBridge::imuAcquireData(Axis3i16 *accData, Axis3i16 *gyroData)
// {
//     pthread_mutex_lock(&sensors_mutex_);
//     *accData = accData_;
//     *gyroData = gyroData_;
//     pthread_mutex_unlock(&sensors_mutex_);
// }

// void cfGzBridge::poseAcquireData(poseMeasurement_t *poseData)
// {
//     pthread_mutex_lock(&pose_mutex_);
//     *poseData = poseData_;
//     pthread_mutex_unlock(&pose_mutex_);
// }

// #include <gz/transport/CIface.h>
// #include <gz/msgs/imu.pb.h>
// #include <gz/msgs/model.pb.h>
// #include <gz/msgs/odometry.pb.h>
// #include <gz/math.hh>
// #include <gz/msgs.hh>
// #include <gz/transport.hh>

#ifdef __cplusplus
extern "C" {
#endif

#define DEBUG_MODULE "GZ_BRIDGE"

static bool isInit = false;

// static cfGzBridge cfGzBridgeObj("crazysim_default", "cf_1", "crazyflie", "pose");
static xQueueHandle imuDataQueue;
STATIC_MEM_QUEUE_ALLOC(imuDataQueue, 1, 2 * sizeof(Axis3i16));
static xQueueHandle poseDataQueue;
STATIC_MEM_QUEUE_ALLOC(poseDataQueue, 1, sizeof(poseMeasurement_t));
SemaphoreHandle_t xMutexImu;
SemaphoreHandle_t xMutexPose;

// STATIC_MEM_TASK_ALLOC(cfGzBridgeTask, SENSORS_TASK_STACKSIZE);

BaseType_t acquireImuData(Axis3i16 *accData, Axis3i16 *gyroData)
{
    // cfGzBridgeObj.imuAcquireData(accData, gyroData);
    // DEBUG_PRINT("Acquired IMU data: Acc: %d %d %d, Gyro: %d %d %d \n", accData->x, accData->y, accData->z, gyroData->x, gyroData->y, gyroData->z);
    Axis3i16 imuData[2];
    if (pdTRUE == xQueueReceive(imuDataQueue, imuData, 0))
    {
        *accData = imuData[1];
        *gyroData = imuData[0];
        return pdTRUE;
    }
    return pdFALSE;
}

BaseType_t acquirePoseData(poseMeasurement_t *poseData)
{
    poseMeasurement_t poseDataBuffer;
    if (pdTRUE == xQueueReceive(poseDataQueue, &poseDataBuffer, 0))
    {
        *poseData = poseDataBuffer;
        return pdTRUE;
    }
    return pdFALSE;
}

void imu_callback(const char *_data, const size_t _size, const char *_msgType,
                  void *_userData)
{
    gz::msgs::IMU imu_msg;
    imu_msg.ParseFromArray(_data, _size);
    // const char *partition;

    Axis3i16 imuData[2];

    imuData[0] = {
        .x = static_cast<int16_t>(imu_msg.angular_velocity().x() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE),
        .y = static_cast<int16_t>(imu_msg.angular_velocity().y() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE),
        .z = static_cast<int16_t>(imu_msg.angular_velocity().z() / SENSORS_G_PER_LSB_CFG / GRAVITY_MAGNITUDE)
    };

    imuData[1] = {
        .x = static_cast<int16_t>(imu_msg.linear_acceleration().x() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF),
        .y = static_cast<int16_t>(imu_msg.linear_acceleration().y() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF),
        .z = static_cast<int16_t>(imu_msg.linear_acceleration().z() / SENSORS_DEG_PER_LSB_CFG  / DEG_TO_RAD_CF)
    };

    SafeQueueSend(imuDataQueue, &imuData, xMutexImu);
    // SafeQueueSend(gyroDataQueue, &gyroData);

    // std::cout << "Acc: " << imuData[1].x << " " << imuData[1].y << " " << imuData[1].z << std::endl;
    // gz_DataCallback();
    // portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void odom_callback(const char *_data, const size_t _size, const char *_msgType,
                  void *_userData)
{
    gz::msgs::Odometry odom_msg;
    odom_msg.ParseFromArray(_data, _size);

    poseMeasurement_t poseData;
    float extPosStdDev = 0.01;
    float extQuatStdDev = 4.5e-3;
    
    poseData.x = odom_msg.pose().position().x();
    poseData.y = odom_msg.pose().position().y();
    poseData.z = odom_msg.pose().position().z();
    poseData.quat.w = odom_msg.pose().orientation().w();
    poseData.quat.x = odom_msg.pose().orientation().x();
    poseData.quat.y = odom_msg.pose().orientation().y();
    poseData.quat.z = odom_msg.pose().orientation().z();
    poseData.stdDevPos = extPosStdDev;
    poseData.stdDevQuat = extQuatStdDev;

    SafeQueueSend(poseDataQueue, &poseData, xMutexPose);
    // std::cout << "Ext pose: " << poseData.x << " " << poseData.y << " " << poseData.z << std::endl;

    // printf("Partition[%s] Msg length: %zu bytes\n", partition, _size);
    // printf("Partition[%s] Msg type: %s\n", partition, _msgType);
    // printf("Partition[%s] Odom Msg x: %s, y: %s, z: %s\n", partition, odom_msg.pose().position().x().c_str(), odom_msg.pose().position().y().c_str(), odom_msg.pose().position().z().c_str());
}

static void cfGzBridgeTask(void* param)
{

    // systemWaitStart();

    GzTransportNode *node = gzTransportNodeCreate(nullptr);
    // DEBUG_PRINT("Initialized gz transport node. \n");

    // Subscribe to IMU topic
    // const char *imu_topic = "/world/crazysim_default/model/cf_1/link/crazyflie/body/sensor/imu_sensor/imu";
    const char *imu_topic = "/cf_1/imu";

    if (gzTransportSubscribe(node, imu_topic, imu_callback, nullptr) != 0)
    {
        printf("Error subscribing to topic %s.\n", imu_topic);
        return;
    }
    
    // const char *odom_topic = "/model/cf_1/odometry";
    const char *odom_topic = "/cf_1/odom";
    if (gzTransportSubscribe(node, odom_topic, odom_callback, nullptr) != 0)
    {
        printf("Error subscribing to topic %s.\n", odom_topic);
        return;
    }
    // TickType_t lastWakeTime = xTaskGetTickCount();
    while(1)
    {
        // vTaskDelayUntil(&lastWakeTime, M2T(2));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    gzTransportWaitForShutdown();
    gzTransportNodeDestroy(&node);
    
    // sleep(5000);
    // DEBUG_PRINT("Shutting down Gazebo Bridge \n");
}

void SafeQueueSend(QueueHandle_t queue, const void *item, SemaphoreHandle_t xMutex) {
    if (xSemaphoreTake(xMutex, portMAX_DELAY) == pdTRUE) {
        if (xQueueSend(queue, item, portMAX_DELAY) != pdPASS) {
            // Handle queue send failure
        }
        xSemaphoreGive(xMutex);
    }
}

void cfGzBridgeInit(void)
{
    if (isInit)
    {
        return;
    }

    imuDataQueue = STATIC_MEM_QUEUE_CREATE(imuDataQueue);
    poseDataQueue = STATIC_MEM_QUEUE_CREATE(poseDataQueue);
    // gyroDataQueue = STATIC_MEM_QUEUE_CREATE(gyroDataQueue);
    xMutexImu = xSemaphoreCreateMutex();
    xMutexPose = xSemaphoreCreateMutex();
    // STATIC_MEM_TASK_CREATE(cfGzBridgeTask, cfGzBridgeTask, "GAZEBO BRIDGE", NULL, SENSORS_TASK_PRI);
    std::thread gazeboThread(cfGzBridgeTask, nullptr);
    gazeboThread.detach();
    

    isInit = true;
}

void __attribute__((weak)) gz_DataCallback(void) {}

#ifdef __cplusplus
}
#endif