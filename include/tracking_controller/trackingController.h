/*
	FILE: trackingController.h
	-------------------------------
	function definition of px4 tracking controller
*/
#ifndef TRACKING_CONTROLLER_H
#define TRACKING_CONTROLLER_H
#include <ros/ros.h>
#include <Eigen/Dense>
#include <queue>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/PositionTarget.h>
#include <tracking_controller/Target.h>
#include <tracking_controller/PositionCommand.h>
#include <tracking_controller/utils.h>

using std::cout;
using std::endl;
namespace controller
{
	class trackingController
	{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber odomSub_;			  // Subscribe to odometry
		ros::Subscriber imuSub_;			  // IMU data subscriber
		ros::Subscriber targetSub_;			  // subscriber for the tracking target states
		ros::Publisher cmdPub_;				  // command publisher
		ros::Publisher accCmdPub_;			  // acceleration command publisher
		ros::Publisher poseVisPub_;			  // current pose publisher
		ros::Publisher targetVisPub_;		  // target pose publisher
		ros::Publisher histTrajVisPub_;		  // history trajectory publisher
		ros::Publisher targetHistTrajVisPub_; // target trajectory publisher
		ros::Publisher velAndAccVisPub_;	  // velocity and acceleration visualization publisher
		ros::Timer cmdTimer_;				  // command timer
		ros::Timer thrustEstimatorTimer_;	  // thrust estimator timer
		ros::Timer visTimer_;				  // visualization timer

		// parameters
		bool bodyRateControl_ = false;
		bool attitudeControl_ = false;
		bool accControl_ = true;
		Eigen::Vector3d pPos_, iPos_, dPos_;
		Eigen::Vector3d pVel_, iVel_, dVel_;
		double attitudeControl_Roll_Pitch_;
		double attitudeControl_Yaw_;
		double hoverThrust_;
		bool verbose_;
		std::string odom_topic, imu_topic, target_topic;
		std::string attitude_target_topic, position_target_topic;
		bool IGNORE_ACC_VEL, IGNORE_ACC;
		bool init_odom = false;
		bool use_fcu_ctrl;
		ros::Time last_Received;

		// controller data
		bool odomReceived_ = false;
		bool imuReceived_ = false;
		bool thrustReady_ = false;
		bool firstTargetReceived_ = false;
		bool targetReceived_ = false;
		bool firstTime_ = true;
		nav_msgs::Odometry odom_, start_odom;
		sensor_msgs::Imu imuData_;
		tracking_controller::PositionCommand target_;
		ros::Time prevTime_;
		double deltaTime_;
		Eigen::Vector3d posErrorInt_;				   // integral of position error
		Eigen::Vector3d velErrorInt_;				   // integral of velocity error
		Eigen::Vector3d deltaPosError_, prevPosError_; // delta of position error
		Eigen::Vector3d deltaVelError_, prevVelError_; // delta of velocity error
		double cmdThrust_;
		ros::Time cmdThrustTime_;

		double Kp_yaw, Ki_yaw, Kd_yaw;
        double prevYawError_;
        double yawErrorInt_;
		double deltaYawError_;
		double yaw_rate;

		// kalman filter
		bool kfFirstTime_ = true;
		ros::Time kfStartTime_;
		double stateVar_ = 0.01;
		double processNoiseVar_ = 0.01;
		double measureNoiseVar_ = 0.02;
		const Eigen::Vector3d kGravity_ = Eigen::Vector3d(0.0, 0.0, -9.81);
		double pxy_error_max, vxy_error_max;
		double pz_error_max, vz_error_max;
		std::deque<double> prevEstimateThrusts_;

		// visualization
		geometry_msgs::PoseStamped poseVis_;
		std::deque<geometry_msgs::PoseStamped> histTraj_;
		geometry_msgs::PoseStamped targetPoseVis_;
		std::deque<geometry_msgs::PoseStamped> targetHistTraj_;
		bool velFirstTime_ = true;
		Eigen::Vector3d prevVel_;
		ros::Time velPrevTime_;

	public:
		trackingController(const ros::NodeHandle &nh);
		void initParam();
		void registerPub();
		void registerCallback();

		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr &odom);
		void imuCB(const sensor_msgs::ImuConstPtr &imu);
		void targetCB(const tracking_controller::PositionCommandConstPtr &target);
		void cmdCB(const ros::TimerEvent &);
		void thrustEstimateCB(const ros::TimerEvent &);
		void visCB(const ros::TimerEvent &);

		void publishCommand(const Eigen::Vector4d &cmd);
		void publishCommand(const Eigen::Vector4d &cmd, const Eigen::Vector3d &accRef);
		void publishCommand(const Eigen::Vector3d &accRef);
		void computeAttitudeAndAccRef(Eigen::Vector4d &attitudeRefQuat, Eigen::Vector3d &accRef);
		void computeBodyRate(const Eigen::Vector4d &attitudeRefQuat, const Eigen::Vector3d &accRef, Eigen::Vector4d &cmd);

		Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
			const Eigen::Vector3d& y_C,
      		const Eigen::Quaterniond& attitude_estimate) const;

		Eigen::Quaterniond computeDesiredAttitude(const Eigen::Vector3d& desired_acceleration, const double reference_heading,
			const Eigen::Quaterniond& attitude_estimate) const;

		Eigen::Vector3d computeNominalReferenceInputs(const Eigen::Quaterniond& attitude_estimate) const;
		
		bool almostZero(const double value) const;

		bool almostZeroThrust(const double thrust_value) const;

		void limit(Eigen::Vector3d* error, double xy_max, double z_max);

		void publishTakeOff(const nav_msgs::Odometry &odom);
		void publishHover();
		double normalize_yaw(double yaw);
		// visualization
		void publishPoseVis();
		void publishHistTraj();
		void publishTargetVis();
		void publishTargetHistTraj();
		void publishVelAndAccVis();
	};
}

#endif