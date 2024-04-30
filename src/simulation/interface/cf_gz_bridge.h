// File: cf_gz_bridge.h
// Author: Christian Llanes <christian.llanes@gatech.edu>
// Created on 04-13-2024
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "imu_types.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "stabilizer_types.h"

void cfGzBridgeInit(void);
void acquireImuData(Axis3i16 *accData, Axis3i16 *gyroData);
void acquirePoseData(poseMeasurement_t *poseData);
void gz_DataCallback(void);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include <gz/math.hh>
#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/model.pb.h>

class cfGzBridge
{
public:
    cfGzBridge(const char *world, const char *name, const char *model, const char *pose_str);
    ~cfGzBridge();

    void* operator new(size_t size) {
        return pvPortMalloc(size);
    }

    void operator delete(void* pointer) {
        vPortFree(pointer);
    }

    void init();
    BaseType_t waitForImuDataReady();
    void imuAcquireData(Axis3i16 *accData, Axis3i16 *gyroData);
    void poseAcquireData(poseMeasurement_t *poseData);
    
private:
    void odomCallback(const gz::msgs::Odometry &odom_msg);
    void imuCallback(const gz::msgs::IMU &imu);

    const std::string world_name_;
    const std::string model_name_;
    const std::string model_type_;
    const std::string init_pose_str_;

    pthread_mutex_t pose_mutex_;
    pthread_mutex_t sensors_mutex_;

    gz::transport::Node node_;

    Axis3i16 gyroData_;
    Axis3i16 accData_;
    poseMeasurement_t poseData_;
};
#else
typedef struct cfGzBridge cfGzBridge;
#endif

