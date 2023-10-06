/*
  * Copyright 2018 Eric Goubault, Cosynus, LIX, France
  * Copyright 2018 Sylve Putot, Cosynus, LIX, France
  * Copyright 2018 Franck Djeumou, Cosynus, LIX, France
  */

#include <iostream>
#include <mutex>
#include <atomic>
#include <math.h>
#include <deque>
#include <stdio.h>
#include <sdf/sdf.hh>

#include <boost/shared_ptr.hpp>
#include <boost/bind/bind.hpp>
#include <Eigen/Eigen>


#include <gz/math/Vector3.hh>

#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/transport.hh>
#include <gz/msgs.hh>

#include <sdf/sdf.hh>

#include <gz/common.hh>

#include <CrtpUtils.h>
// #include <crazyflie_cpp/ITransport.h>

#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>

/* Fast library for a reader writer queue */
#include "blockingconcurrentqueue.h"

static const std::string kDefaultMotorVelocityReferencePubTopic = "/command/motor_speed";
static const std::string kDefaultImuTopic = "/imu";
static const std::string kDefaultNamespace = "";
static const std::string kDefaultMagneticFieldTopic = "/mag";
static const std::string kDefaultFluidPressureTopic = "/baro";
static const std::string kDefaultOdomTopic = "/odom";
static const std::string kDefaultCfPrefix = "cf";
static const int kDefaultCfNbQuads = 1;
// Constants
static constexpr double kGasConstantNmPerKmolKelvin = 8314.32;
static constexpr double kMeanMolecularAirWeightKgPerKmol = 28.9644;
static constexpr double kGravityMagnitude = 9.80665;
static constexpr double kEarthRadiusMeters = 6356766.0;
static constexpr double kPressureOneAtmospherePascals = 101325.0;
static constexpr double kSeaLevelTempKelvin = 288.15;
static constexpr double kTempLapseKelvinPerMeter = 0.0065;
static constexpr double kAirConstantDimensionless = kGravityMagnitude *
    kMeanMolecularAirWeightKgPerKmol /
        (kGasConstantNmPerKmolKelvin * -kTempLapseKelvinPerMeter);
typedef struct _SensorsData {
	uint8_t data[sizeof(struct imu_s)];
} SensorsData;
namespace gz{
	namespace sim{
		namespace systems{
		class GzCrazyflieInterface : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPostUpdate {
		public:
			GzCrazyflieInterface() :
				namespace_(kDefaultNamespace),
				motor_velocity_reference_pub_topic_(kDefaultMotorVelocityReferencePubTopic),
				imu_sub_topic_(kDefaultImuTopic),
				magnetic_field_sub_topic_(kDefaultMagneticFieldTopic),
				barometer_sub_topic_(kDefaultFluidPressureTopic),
				odom_sub_topic_(kDefaultOdomTopic),
				cf_prefix(kDefaultCfPrefix),
				isPluginOn(true)
				{}
			~GzCrazyflieInterface();

		protected:
			virtual void Configure(const Entity &_entity,
									const std::shared_ptr<const sdf::Element> &_sdf,
									EntityComponentManager &_ecm,
									EventManager &_eventMgr) override;
			virtual void PostUpdate(const UpdateInfo &_info,
								const EntityComponentManager &_ecm) override;

		private:
			Entity linkEntity;
			Model model_;

			int cf_id_;
			std::string namespace_;

			std::string motor_velocity_reference_pub_topic_;
			std::string imu_sub_topic_;
			std::string magnetic_field_sub_topic_;
			std::string barometer_sub_topic_;
			std::string odom_sub_topic_;

			std::string cffirm_addr;
			std::string cffirm_port;
			std::string cflib_addr;
			std::string cflib_port;


			bool enable_logging;
			bool enable_logging_imu;
			bool enable_logging_magnetic_field;
			bool enable_logging_temperature;
			bool enable_parameters;
			bool enable_logging_pressure;
			bool enable_logging_battery;
			bool enable_logging_packets;
			bool use_ros_time;
			bool is_hitl;
			std::string cf_prefix;

			transport::Node node_;

			void ImuCallback(const gz::msgs::IMU& imu_msg);
			void BarometerCallback(const gz::msgs::FluidPressure& air_pressure_msg);
			void OdomCallback(const gz::msgs::Odometry& odom_msg);

			// send and recv functions
			bool sendCf(const uint8_t* data, uint32_t length);
			bool sendCfLib(const uint8_t* data, uint32_t length);
			void recvCfLib(uint8_t* data , int length);
			
			// Threads main function
			void handleMotorsMessage(const uint8_t* data);

			// server port and remote address for the crazyflies
			int port;
			int port_cfLib;
			int fd;
			int fd_cfLib;
			struct sockaddr_in myaddr;
			struct sockaddr_in myaddr_CfLib;

			struct sockaddr_in remaddr_rcv;
			struct sockaddr_in remaddr_rcv_cfLib;
			socklen_t addrlen_rcv;
			socklen_t addrlen_rcv_cfLib;

			struct sockaddr_in remaddr;
			socklen_t addrlen;
			bool socketInit;

			struct sockaddr_in remaddr_cfLib;
			socklen_t addrlen_cfLib;
			bool socketInit_cfLib;

			// void initializeCf(ros::NodeHandle &n);
			void initializeSubsAndPub();

			bool isInit;
			bool isPluginOn;

			// Queue for exchanging messages between recv and sender threads
			moodycamel::BlockingConcurrentQueue<crtpPacket_t> m_queueSendCfFirm;

			// mutex and messages for motors command
			gz::msgs::Actuators m_motor_speed;
			std::mutex motors_mutex;

			struct MotorsCommand {
				float m1;
				float m2;
				float m3;
				float m4;
			} m_motor_command_;
			transport::Node::Publisher motor_velocity_reference_pub_;
			void writeMotors();

			// recv thread
			std::thread receiverThread;
			void recvThread();

			// Crazyflie library receiver thread
			std::thread receiverCfLibThread;
			void recvCfLibThread();

			// Sensors data sender thread
			std::thread senderThread;
			void sendThread();
			
		};
		}
	}
}

