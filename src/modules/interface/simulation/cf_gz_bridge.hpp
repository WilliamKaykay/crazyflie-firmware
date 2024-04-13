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

    void init();
    
private:
    void odomCallback(const gz::msgs::Odometry &odom_msg);
    void imuCallback(const gz::msgs::IMU &imu);

    const std::string world_name_;
    const std::string model_name_;
    const std::string model_type_;
    const std::string init_pose_str_;

    gz::transport::Node _node;
};