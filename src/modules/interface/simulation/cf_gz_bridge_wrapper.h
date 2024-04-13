#ifndef __CF_GZ_BRIDGE_WRAPPER_H__
#define __CF_GZ_BRIDGE_WRAPPER_H__

#ifdef __cplusplus
extern "C" {
#endif


typedef void (*sensorCallbackFunction)(void);

void* createCfGzBridge(const char *world_name, const char *model_name, const char *model_type, const char *init_pose_str, sensorCallbackFunction sensor_callback);
void initCfGzBridge(void* cfGzBridge);

    
#ifdef __cplusplus
}
#endif

#endif // __CF_GZ_BRIDGE_WRAPPER_H__