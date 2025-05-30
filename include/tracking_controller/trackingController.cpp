/*
FILE: trackingController.cpp
-------------------------------
function implementation of px4 tracking controller
*/

#include <tracking_controller/trackingController.h>

namespace controller
{
	trackingController::trackingController(const ros::NodeHandle &nh) : nh_(nh)
	{
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	// Init Param
	void trackingController::initParam()
	{

		// ROS Topic
		this->nh_.getParam("controller/odom_topic", this->odom_topic);
		this->nh_.getParam("controller/imu_topic", this->imu_topic);
		this->nh_.getParam("controller/target_topic", this->target_topic);

		this->nh_.getParam("controller/attitude_target_topic", this->attitude_target_topic);
		this->nh_.getParam("controller/position_arget_topic", this->position_target_topic);

		// Chose control mode, default: ACC Control
		this->nh_.getParam("controller/body_rate_control", this->bodyRateControl_);
		this->nh_.getParam("controller/attitude_control", this->attitudeControl_);
		this->nh_.getParam("controller/acceleration_control", this->accControl_);
		this->nh_.getParam("controller/use_fcu_ctrl", this->use_fcu_ctrl);

		// PID param
		std::vector<double> pPosTemp, iPosTemp, dPosTemp, pVelTemp, iVelTemp, dVelTemp;
		this->nh_.getParam("controller/position_p", pPosTemp);
		this->pPos_(0) = pPosTemp[0];
		this->pPos_(1) = pPosTemp[1];
		this->pPos_(2) = pPosTemp[2];
		this->nh_.getParam("controller/position_i", iPosTemp);
		this->iPos_(0) = iPosTemp[0];
		this->iPos_(1) = iPosTemp[1];
		this->iPos_(2) = iPosTemp[2];
		this->nh_.getParam("controller/position_d", dPosTemp);
		this->dPos_(0) = dPosTemp[0];
		this->dPos_(1) = dPosTemp[1];
		this->dPos_(2) = dPosTemp[2];
		this->nh_.getParam("controller/velocity_p", pVelTemp);
		this->pVel_(0) = pVelTemp[0];
		this->pVel_(1) = pVelTemp[1];
		this->pVel_(2) = pVelTemp[2];
		this->nh_.getParam("controller/velocity_i", iVelTemp);
		this->iVel_(0) = iVelTemp[0];
		this->iVel_(1) = iVelTemp[1];
		this->iVel_(2) = iVelTemp[2];
		this->nh_.getParam("controller/velocity_d", dVelTemp);
		this->dVel_(0) = dVelTemp[0];
		this->dVel_(1) = dVelTemp[1];
		this->dVel_(2) = dVelTemp[2];

		this->nh_.getParam("controller/yaw_p", this->Kp_yaw);
		this->nh_.getParam("controller/yaw_i", this->Ki_yaw);
		this->nh_.getParam("controller/yaw_d", this->Kd_yaw);

		this->nh_.getParam("controller/attitude_control_tau", this->attitudeControlTau_);
		this->nh_.getParam("controller/hover_thrust", this->hoverThrust_);
		this->nh_.getParam("controller/verbose", this->verbose_);

		// Ignore VEL or ACC
		this->nh_.getParam("controller/IGNORE_ACC", this->IGNORE_ACC);
		this->nh_.getParam("controller/IGNORE_ACC_VEL", this->IGNORE_ACC_VEL);

		// Target Time
		this->last_Received = ros::Time::now();

		this->prevYawError_ = 0.0;
		this->yawErrorInt_ = 0.0;
	}

	void trackingController::registerPub()
	{
		// command publisher
		this->cmdPub_ = this->nh_.advertise<mavros_msgs::AttitudeTarget>(this->attitude_target_topic, 100);

		// acc comman publisher
		this->accCmdPub_ = this->nh_.advertise<mavros_msgs::PositionTarget>(this->position_target_topic, 100);

		// current pose visualization publisher
		this->poseVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/robot_pose", 1);

		// trajectory history visualization publisher
		this->histTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/trajectory_history", 1);

		// target pose visualization publisher
		this->targetVisPub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/tracking_controller/target_pose", 1);

		// target trajectory history publisher
		this->targetHistTrajVisPub_ = this->nh_.advertise<nav_msgs::Path>("/tracking_controller/target_trajectory_history", 1);

		// velocity and acceleration visualization publisher
		this->velAndAccVisPub_ = this->nh_.advertise<visualization_msgs::Marker>("/tracking_controller/vel_and_acc_info", 1);
	}

	void trackingController::registerCallback()
	{
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odom_topic, 1, &trackingController::odomCB, this);

		// imu subscriber
		this->imuSub_ = this->nh_.subscribe(this->imu_topic, 1, &trackingController::imuCB, this);

		// target setpoint subscriber
		this->targetSub_ = this->nh_.subscribe(this->target_topic, 1, &trackingController::targetCB, this);

		// controller publisher timer
		this->cmdTimer_ = this->nh_.createTimer(ros::Duration(0.01), &trackingController::cmdCB, this);

		if (not this->accControl_)
		{
			// auto thrust esimator timer
			this->thrustEstimatorTimer_ = this->nh_.createTimer(ros::Duration(0.01), &trackingController::thrustEstimateCB, this);
		}
		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.033), &trackingController::visCB, this);
	}

	void trackingController::odomCB(const nav_msgs::OdometryConstPtr &odom)
	{
		this->odom_ = *odom;
		this->odomReceived_ = true;
	}

	void trackingController::imuCB(const sensor_msgs::ImuConstPtr &imu)
	{
		this->imuData_ = *imu;
		this->imuReceived_ = true;
	}

	void trackingController::targetCB(const tracking_controller::PositionCommandConstPtr &target)
	{
		this->target_ = *target;
		this->firstTargetReceived_ = true;
		this->targetReceived_ = true;
		this->last_Received = ros::Time::now();
	}

	void trackingController::cmdCB(const ros::TimerEvent &)
	{

		/*​​
		 * 状态机流程：
		 *
		 * [Start]
		 *  |
		 *  |--> [检查里程计] --(未收到odom)--> 返回
		 *  |        |
		 *  |       (收到odom)
		 *  |        |
		 *  |        V
		 *  |--> [检查是否开始规划] --(未开始规划)--> [发布起飞指令]
		 *  |        |                              |
		 *  |        |                              |--(把持起飞位置)-->
		 *  |       (已经开始规划)
		 *  |        |
		 *  |        V
		 *  |--> [检查是否超时] ----(超时)----> [发布悬停指令]
		 *  |        |                            |
		 *  |       (未超时)                       |--(维持悬停)-->
		 *  |        |
		 *  |        V
		 *  +--> [计算控制量] --(发布新控制量)--> [结束]
		 *
		 */

		if (not this->odomReceived_)
		{
			ROS_ERROR("No Odom!, Please check the location part");
			return;
		}

		// store the frist odom as start position
		if (not init_odom)
		{
			start_odom = this->odom_;
			init_odom = true;
		}
		//
		if (not this->targetReceived_)
		{
			// Never Sub Target,  Not Start Plan
			if (not this->firstTargetReceived_)
			{
				// ROS_WARN("Ready to TakeOFF, Be Careful");
				this->publishTakeOff(start_odom);
			}
			else if ((ros::Time::now() - this->last_Received).toSec() > 0.5) // Maybe Plan End or Something Else, Hover
			{
				// ROS_WARN("There is no Target, is the Plan End?");
				this->publishHover();
			}
			return;
		}
		// Test for Open loop control
		if (this->use_fcu_ctrl)
		{
			mavros_msgs::PositionTarget cmdMsg;
			cmdMsg.coordinate_frame = cmdMsg.FRAME_LOCAL_NED;
			cmdMsg.header.stamp = ros::Time::now();
			cmdMsg.header.frame_id = "map";
			cmdMsg.position.x = this->target_.position.x;
			cmdMsg.position.y = this->target_.position.y;
			cmdMsg.position.z = this->target_.position.z;
			cmdMsg.velocity.x = this->target_.velocity.x;
			cmdMsg.velocity.y = this->target_.velocity.y;
			cmdMsg.velocity.z = this->target_.velocity.z;
			cmdMsg.acceleration_or_force.x = this->target_.acceleration.x;
			cmdMsg.acceleration_or_force.y = this->target_.acceleration.y;
			cmdMsg.acceleration_or_force.z = this->target_.acceleration.z - 9.8;
			cmdMsg.yaw = this->target_.yaw;
			cmdMsg.yaw_rate = this->target_.yaw_dot;
			cmdMsg.type_mask = 0;
			this->accCmdPub_.publish(cmdMsg);
			ROS_WARN("Only use FCU, No PID");
			return;
		}

		Eigen::Vector4d cmd;

		// NOTE 目前仅使用ACC Control
		// 1. Find target reference attitude from the desired acceleration
		Eigen::Vector4d attitudeRefQuat;
		Eigen::Vector3d accRef;
		// PID main function
		this->computeAttitudeAndAccRef(attitudeRefQuat, accRef);

		if (this->bodyRateControl_)
		{
			// 2. Compute the body rate from the reference attitude
			this->computeBodyRate(attitudeRefQuat, accRef, cmd);

			// 3. publish body rate as control input
			this->publishCommand(cmd);
		}

		if (this->attitudeControl_)
		{
			// direct attitude control
			cmd = attitudeRefQuat;
			this->publishCommand(cmd, accRef);
		}

		if (this->accControl_)
		{
			this->publishCommand(accRef);
		}

		this->targetReceived_ = false;
	}

	void trackingController::thrustEstimateCB(const ros::TimerEvent &)
	{
		if (not this->thrustReady_ or not this->imuReceived_)
		{
			return;
		}
		if (this->kfFirstTime_)
		{
			this->kfFirstTime_ = false;
			this->kfStartTime_ = ros::Time::now();
			return;
		}

		// run estimator when the command thrust is available
		// sync IMU and command thrust (?)
		double hoverThrust = this->hoverThrust_;
		double cmdThrust = this->cmdThrust_;
		Eigen::Vector3d currAccBody(this->imuData_.linear_acceleration.x, this->imuData_.linear_acceleration.y, this->imuData_.linear_acceleration.z);
		Eigen::Vector4d currQuat(this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d currRot = controller::quat2RotMatrix(currQuat);
		Eigen::Vector3d currAcc = currRot * currAccBody;

		// states: Hover thrust
		double states = hoverThrust;
		double A = 1;
		double H = -(cmdThrust * 9.8) * (1.0 / pow(hoverThrust, 2));
		double z = (currAcc(2) - 9.8); // acceleratoin

		// Kalman filter predict (predict states and propagate var)
		states = A * states;
		this->stateVar_ += this->processNoiseVar_;

		// Kalman filter correction
		double Ivar = std::max(H * this->stateVar_ * H + this->measureNoiseVar_, this->measureNoiseVar_);
		double K = this->stateVar_ * H / Ivar;
		double I = z - (cmdThrust / hoverThrust - 1.0) * 9.8;

		// double residual;
		// residual = I;

		// // whether this iteration passes the test
		// double ItestRatio = (I*I/(this->innovGateSize_ * Ivar));
		// if (ItestRatio < 1.0){
		// 	hoverThrust = hoverThrust + K * I;
		// 	this->stateVar_ = (1.0 - K * H) * this->stateVar_;
		// 	residual = z - (cmdThrust/hoverThrust - 1.0) * 9.8;
		// 	this->hoverThrust_ = hoverThrust;
		// }

		// update hoverThrust
		double newHoverThrust = hoverThrust + K * I;
		this->stateVar_ = (1.0 - K * H) * this->stateVar_;

		if (this->verbose_)
		{
			cout << "[trackingController]: Estimation variance: " << this->stateVar_ << endl;
			// cout << "test ratio: " << I*I/Ivar << endl;
			// cout << "new estimate is: " << newHoverThrust << endl;
		}

		double prevMinThrust = 0.0;
		double prevMaxThrust = 1.0;
		if (this->prevEstimateThrusts_.size() < 10)
		{
			this->prevEstimateThrusts_.push_back(newHoverThrust);
		}
		else
		{
			this->prevEstimateThrusts_.pop_front();
			this->prevEstimateThrusts_.push_back(newHoverThrust);
			std::deque<double>::iterator itMin = std::min_element(this->prevEstimateThrusts_.begin(), this->prevEstimateThrusts_.end());
			std::deque<double>::iterator itMax = std::max_element(this->prevEstimateThrusts_.begin(), this->prevEstimateThrusts_.end());
			prevMinThrust = *itMin;
			prevMaxThrust = *itMax;
		}

		// if the state variance is smaller enough, update the hover thrust
		if (std::abs(prevMinThrust - prevMaxThrust) < 0.005)
		{
			if (newHoverThrust > 0 and newHoverThrust < 1.0)
			{
				this->hoverThrust_ = newHoverThrust;
				ros::Time currTime = ros::Time::now();
				double estimatedTime = (currTime - this->kfStartTime_).toSec();
				if (this->verbose_)
				{
					cout << "[trackingController]: New estimate at " << estimatedTime << "s, and Estimated thrust is: " << newHoverThrust << ". Variance: " << this->stateVar_ << endl;
				}
			}
			else
			{
				cout << "[trackingController]: !!!!!!!!!!AUTO TRHUST ESTIMATION FAILS!!!!!!!!!" << endl;
			}
		}
		// cout << "current commnd thrust is: " << cmdThrust << endl;
		// cout << "current z: " << z << endl;
		// cout << "world acc: " << currAcc.transpose() << endl;
		// cout << "[trackingController]: Estimated thrust is: " << newHoverThrust << endl;
		// cout << "variance: " << this->stateVar_ << endl;
		// cout << "hover thrust from acc: " << (9.8 * cmdThrust)/currAcc(2) << endl;
		// cout << "Current hover thrust is set to: " << this->hoverThrust_ << endl;
	}

	void trackingController::visCB(const ros::TimerEvent &)
	{
		this->publishPoseVis();
		this->publishHistTraj();
		this->publishTargetVis();
		this->publishTargetHistTraj();
		this->publishVelAndAccVis();
	}

	void trackingController::publishCommand(const Eigen::Vector4d &cmd)
	{
		mavros_msgs::AttitudeTarget cmdMsg;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.body_rate.x = cmd(0);
		cmdMsg.body_rate.y = cmd(1);
		cmdMsg.body_rate.z = cmd(2);
		cmdMsg.thrust = cmd(3);
		cmdMsg.type_mask = cmdMsg.IGNORE_ATTITUDE;
		this->cmdPub_.publish(cmdMsg);
	}

	void trackingController::publishCommand(const Eigen::Vector4d &cmd, const Eigen::Vector3d &accRef)
	{
		mavros_msgs::AttitudeTarget cmdMsg;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.orientation.w = cmd(0);
		cmdMsg.orientation.x = cmd(1);
		cmdMsg.orientation.y = cmd(2);
		cmdMsg.orientation.z = cmd(3);
		double thrust = accRef.norm();
		double thrustPercent = std::max(0.0, std::min(1.0, 1.0 * thrust / (9.8 * 1.0 / this->hoverThrust_))); // percent
		this->cmdThrust_ = thrustPercent;
		this->cmdThrustTime_ = ros::Time::now();
		this->thrustReady_ = true;
		cmdMsg.thrust = thrustPercent;
		cmdMsg.type_mask = cmdMsg.IGNORE_ROLL_RATE + cmdMsg.IGNORE_PITCH_RATE + cmdMsg.IGNORE_YAW_RATE;
		this->cmdPub_.publish(cmdMsg);
	}

	void trackingController::publishCommand(const Eigen::Vector3d &accRef)
	{
		mavros_msgs::PositionTarget cmdMsg;
		cmdMsg.coordinate_frame = cmdMsg.FRAME_LOCAL_NED;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.acceleration_or_force.x = accRef(0);
		cmdMsg.acceleration_or_force.y = accRef(1);
		cmdMsg.acceleration_or_force.z = accRef(2) - 9.8;
		cmdMsg.yaw = this->target_.yaw;
		cmdMsg.yaw_rate = this->yaw_rate;
		cmdMsg.type_mask = cmdMsg.IGNORE_PX + cmdMsg.IGNORE_PY + cmdMsg.IGNORE_PZ + cmdMsg.IGNORE_VX + cmdMsg.IGNORE_VY + cmdMsg.IGNORE_VZ + cmdMsg.IGNORE_YAW;
		// cout << "acc: " << accRef(0) << " " << accRef(1) << " " << accRef(2) - 9.8 << " " << endl;
		this->accCmdPub_.publish(cmdMsg);
	}

	void trackingController::publishTakeOff(const nav_msgs::Odometry &odom)
	{
		mavros_msgs::PositionTarget cmdMsg;
		cmdMsg.coordinate_frame = cmdMsg.FRAME_LOCAL_NED;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.position.x = odom.pose.pose.position.x;
		cmdMsg.position.y = odom.pose.pose.position.y;
		cmdMsg.position.z = odom.pose.pose.position.z + 1.0;
		cmdMsg.type_mask = cmdMsg.IGNORE_VX + cmdMsg.IGNORE_VY + cmdMsg.IGNORE_VZ + cmdMsg.IGNORE_AFX + cmdMsg.IGNORE_AFY + cmdMsg.IGNORE_AFZ + cmdMsg.IGNORE_YAW + cmdMsg.IGNORE_YAW_RATE;
		this->accCmdPub_.publish(cmdMsg);
	}

	void trackingController::publishHover()
	{
		mavros_msgs::PositionTarget cmdMsg;
		cmdMsg.coordinate_frame = cmdMsg.FRAME_LOCAL_NED;
		cmdMsg.header.stamp = ros::Time::now();
		cmdMsg.header.frame_id = "map";
		cmdMsg.position.x = this->odom_.pose.pose.position.x;
		cmdMsg.position.y = this->odom_.pose.pose.position.y;
		cmdMsg.position.z = this->odom_.pose.pose.position.z;
		cmdMsg.type_mask = cmdMsg.IGNORE_VX + cmdMsg.IGNORE_VY + cmdMsg.IGNORE_VZ + cmdMsg.IGNORE_AFX + cmdMsg.IGNORE_AFY + cmdMsg.IGNORE_AFZ + cmdMsg.IGNORE_YAW + cmdMsg.IGNORE_YAW_RATE;
		this->accCmdPub_.publish(cmdMsg);
	}

	void trackingController::computeAttitudeAndAccRef(Eigen::Vector4d &attitudeRefQuat, Eigen::Vector3d &accRef)
	{
		// Find the reference acceleration for motors, then convert the acceleration into attitude

		/*
			There are four components of reference acceleration:
			1. target acceleration (from setpoint)
			2. acceleration from feedback of position and velocity (P control)
			3. air drag (not consider this now)
			4. gravity
		*/

		if (this->firstTime_)
		{
			this->prevTime_ = ros::Time::now();
			this->deltaTime_ = 0.0;
			this->posErrorInt_ = Eigen::Vector3d(0.0, 0.0, 0.0);
			this->velErrorInt_ = Eigen::Vector3d(0.0, 0.0, 0.0);
			this->yawErrorInt_ = 0.0;
			// this->firstTime_ = false;
		}
		else
		{
			ros::Time currTime = ros::Time::now();
			this->deltaTime_ = (currTime - this->prevTime_).toSec();
			this->prevTime_ = currTime;
		}

		// 1. target acceleration
		Eigen::Vector3d accTarget(this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);

		// 2. position & velocity feedback control (PID control for both position and velocity)
		Eigen::Vector3d currPos(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d currVelBody(this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		// Eigen::Vector4d currQuat(this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		// Eigen::Matrix3d currRot = controller::quat2RotMatrix(currQuat);
		// Eigen::Vector3d currVel = currRot * currVelBody;
		// maybe not need in LIO
		Eigen::Vector3d currVel = currVelBody;
		Eigen::Vector3d targetPos(this->target_.position.x, this->target_.position.y, this->target_.position.z);
		Eigen::Vector3d targetVel(this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);
		Eigen::Vector3d positionError = targetPos - currPos;
		Eigen::Vector3d velocityError = targetVel - currVel;
		this->posErrorInt_ += this->deltaTime_ * positionError;
		this->velErrorInt_ += this->deltaTime_ * velocityError;

		double current_yaw = controller::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		double yaw_error = normalize_yaw(this->target_.yaw - current_yaw);
		this->yawErrorInt_ += this->deltaTime_ * yaw_error;

		if (this->firstTime_)
		{
			this->deltaPosError_ = Eigen::Vector3d(0.0, 0.0, 0.0);
			this->prevPosError_ = positionError;
			this->deltaVelError_ = Eigen::Vector3d(0.0, 0.0, 0.0);
			this->prevVelError_ = velocityError;
			this->deltaYawError_ = 0.0;
			this->prevYawError_ = yaw_error;

			this->firstTime_ = false;
		}
		else
		{
			this->deltaPosError_ = (positionError - this->prevPosError_) / this->deltaTime_;
			this->prevPosError_ = positionError;
			this->deltaVelError_ = (velocityError - this->prevVelError_) / this->deltaTime_;
			this->prevVelError_ = velocityError;

			this->deltaYawError_ = (yaw_error - this->prevYawError_) / this->deltaTime_;
			this->prevYawError_ = yaw_error;
		}

		this->yaw_rate = this->Kp_yaw * yaw_error + this->Ki_yaw * this->yawErrorInt_ + this->Kd_yaw * deltaYawError_;
		// mask out the velocity input if needed
		if (this->IGNORE_ACC_VEL)
		{
			velocityError *= 0.0;
			this->velErrorInt_ *= 0.0;
			this->dVel_ *= 0.0;
		}

		Eigen::Vector3d accFeedback = this->pPos_.asDiagonal() * positionError + this->iPos_.asDiagonal() * this->posErrorInt_ + this->dPos_.asDiagonal() * this->deltaPosError_ +
									  this->pVel_.asDiagonal() * velocityError + this->iVel_.asDiagonal() * this->velErrorInt_ + this->dVel_.asDiagonal() * this->deltaVelError_;

		// 3. air drag
		Eigen::Vector3d accAirdrag(0.0, 0.0, 0.0);

		// 4. gravity
		Eigen::Vector3d gravity{0.0, 0.0, -9.8};

		// Final reference acceleration for motors
		if (this->IGNORE_ACC_VEL or this->IGNORE_ACC)
		{
			accTarget *= 0.0;
		}
		accRef = accTarget + accFeedback - accAirdrag - gravity;

		// Convert the reference acceleration into the reference attitude
		// double yaw = this->target_.yaw;  // todo: the original implementation uses the current yaw or velocity yaw
		double yaw = controller::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		Eigen::Vector3d direction(cos(yaw), sin(yaw), 0.0);
		Eigen::Vector3d zDirection = accRef / accRef.norm();
		Eigen::Vector3d yDirection = zDirection.cross(direction) / (zDirection.cross(direction)).norm();
		Eigen::Vector3d xDirection = yDirection.cross(zDirection) / (yDirection.cross(zDirection)).norm();

		// with three axis vector, we can construct the rotation matrix
		Eigen::Matrix3d attitudeRefRot;
		attitudeRefRot << xDirection(0), yDirection(0), zDirection(0),
			xDirection(1), yDirection(1), zDirection(1),
			xDirection(2), yDirection(2), zDirection(2);
		attitudeRefQuat = controller::rot2Quaternion(attitudeRefRot);

		// cout << "Position Error: " << positionError(0) << " " << positionError(1) << " " << positionError(2) << endl;
		// 	cout << "Target Velocity: " << targetVel(0) << " " << targetVel(1) << " " << targetVel(2) << endl;
		// cout << "Current Velocity: " << currVel(0) << " " << currVel(1) << " " << currVel(2) << endl;
		// cout << "Velocity Error: " << velocityError(0) << " " << velocityError(1) << " " << velocityError(2) << endl;
		// 	cout << "Feedback acceleration: " << accFeedback(0) << " " << accFeedback(1) << " " << accFeedback(2) << endl;
		// 	cout << "Desired Acceleration: " << accRef(0) << " " << accRef(1) << " " << accRef(2) << endl;
	}

	void trackingController::computeBodyRate(const Eigen::Vector4d &attitudeRefQuat, const Eigen::Vector3d &accRef, Eigen::Vector4d &cmd)
	{
		// body rate
		Eigen::Vector4d currAttitudeQuat(this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Vector4d inverseQuat(1.0, -1.0, -1.0, -1.0);
		Eigen::Vector4d currAttitudeQuatInv = inverseQuat.asDiagonal() * currAttitudeQuat;
		Eigen::Vector4d attitudeErrorQuat = quatMultiplication(currAttitudeQuatInv, attitudeRefQuat);
		cmd(0) = (2.0 / this->attitudeControlTau_) * std::copysign(1.0, attitudeErrorQuat(0)) * attitudeErrorQuat(1);
		cmd(1) = (2.0 / this->attitudeControlTau_) * std::copysign(1.0, attitudeErrorQuat(0)) * attitudeErrorQuat(2);
		cmd(2) = (2.0 / this->attitudeControlTau_) * std::copysign(1.0, attitudeErrorQuat(0)) * attitudeErrorQuat(3);

		// thrust
		// Eigen::Matrix3d currAttitudeRot = quat2RotMatrix(currAttitudeQuat);
		// Eigen::Vector3d zDirection = currAttitudeRot.col(2); // body z axis
		// double thrust = accRef.dot(zDirection); // thrust in acceleration
		double thrust = accRef.norm();
		double thrustPercent = std::max(0.0, std::min(1.0, 1.0 * thrust / (9.8 * 1.0 / this->hoverThrust_))); // percent
		this->cmdThrust_ = thrustPercent;
		this->cmdThrustTime_ = ros::Time::now();
		this->thrustReady_ = true;
		cmd(3) = thrustPercent;

		// cout << "body rate: " << cmd(0) << " " << cmd(1) << " " << cmd(2) << endl;
		if (this->verbose_)
		{
			cout << "[trackingController]: Thrust percent: " << thrustPercent << endl;
		}
	}

	double trackingController::normalize_yaw(double yaw)
	{
		yaw = fmod(yaw, 2 * M_PI);
		if (yaw > M_PI)
			yaw -= 2 * M_PI;
		else if (yaw < -M_PI)
			yaw += 2 * M_PI;
		return yaw;
	}

	void trackingController::publishPoseVis()
	{
		if (not this->odomReceived_)
			return;
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = this->odom_.pose.pose.position.x;
		ps.pose.position.y = this->odom_.pose.pose.position.y;
		ps.pose.position.z = this->odom_.pose.pose.position.z;
		ps.pose.orientation = this->odom_.pose.pose.orientation;
		if (this->histTraj_.size() <= 100)
		{
			this->histTraj_.push_back(ps);
		}
		else
		{
			this->histTraj_.push_back(ps);
			this->histTraj_.pop_front();
		}
		this->poseVis_ = ps;
		this->poseVisPub_.publish(ps);
	}

	void trackingController::publishHistTraj()
	{
		if (not this->odomReceived_)
			return;
		nav_msgs::Path histTrajMsg;
		histTrajMsg.header.frame_id = "map";
		histTrajMsg.header.stamp = ros::Time::now();
		for (size_t i = 0; i < this->histTraj_.size(); ++i)
		{
			histTrajMsg.poses.push_back(this->histTraj_[i]);
		}

		this->histTrajVisPub_.publish(histTrajMsg);
	}

	void trackingController::publishTargetVis()
	{
		if (not this->firstTargetReceived_)
			return;
		geometry_msgs::PoseStamped ps;
		ps.header.frame_id = "map";
		ps.header.stamp = ros::Time::now();
		ps.pose.position.x = this->target_.position.x;
		ps.pose.position.y = this->target_.position.y;
		ps.pose.position.z = this->target_.position.z;
		ps.pose.orientation = controller::quaternion_from_rpy(0, 0, this->target_.yaw);
		if (this->targetHistTraj_.size() <= 100)
		{
			this->targetHistTraj_.push_back(ps);
		}
		else
		{
			this->targetHistTraj_.push_back(ps);
			this->targetHistTraj_.pop_front();
		}

		this->targetPoseVis_ = ps;
		this->targetVisPub_.publish(ps);
	}

	void trackingController::publishTargetHistTraj()
	{
		if (not this->firstTargetReceived_)
			return;
		nav_msgs::Path targetHistTrajMsg;
		targetHistTrajMsg.header.frame_id = "map";
		targetHistTrajMsg.header.stamp = ros::Time::now();
		for (size_t i = 0; i < this->targetHistTraj_.size(); ++i)
		{
			targetHistTrajMsg.poses.push_back(this->targetHistTraj_[i]);
		}

		this->targetHistTrajVisPub_.publish(targetHistTrajMsg);
	}

	void trackingController::publishVelAndAccVis()
	{
		if (not this->odomReceived_)
			return;
		// current velocity
		Eigen::Vector3d currPos(this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		Eigen::Vector3d currVelBody(this->odom_.twist.twist.linear.x, this->odom_.twist.twist.linear.y, this->odom_.twist.twist.linear.z);
		Eigen::Vector4d currQuat(this->odom_.pose.pose.orientation.w, this->odom_.pose.pose.orientation.x, this->odom_.pose.pose.orientation.y, this->odom_.pose.pose.orientation.z);
		Eigen::Matrix3d currRot = controller::quat2RotMatrix(currQuat);
		Eigen::Vector3d currVel = currRot * currVelBody;

		// current acceleration
		Eigen::Vector3d currAcc;
		ros::Time currTime = ros::Time::now();
		if (this->velFirstTime_)
		{
			this->velPrevTime_ = ros::Time::now();
			currAcc = Eigen::Vector3d(0.0, 0.0, 0.0);
			this->velFirstTime_ = false;
		}
		else
		{
			double dt = (currTime - this->velPrevTime_).toSec();
			currAcc = (currVel - this->prevVel_) / dt;
			// cout << "dt: " << dt << endl;
			// cout << "current velocity: " << currVel(0) << " " << currVel(1) << " " << currVel(2) << endl;
			// cout << "prev velocity: " << this->prevVel_(0) << " " << this->prevVel_(1) << " " << this->prevVel_(2) << endl;
		}
		this->prevVel_ = currVel;
		this->velPrevTime_ = currTime;

		// target position
		Eigen::Vector3d targetPos(this->target_.position.x, this->target_.position.y, this->target_.position.z);

		// target velocity
		Eigen::Vector3d targetVel(this->target_.velocity.x, this->target_.velocity.y, this->target_.velocity.z);

		// target acceleration
		Eigen::Vector3d targetAcc(this->target_.acceleration.x, this->target_.acceleration.y, this->target_.acceleration.z);

		visualization_msgs::Marker velAndAccVisMsg;
		velAndAccVisMsg.header.frame_id = "map";
		velAndAccVisMsg.header.stamp = ros::Time::now();
		velAndAccVisMsg.ns = "tracking_controller";
		// velAndAccVisMsg.id = 0;
		velAndAccVisMsg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		velAndAccVisMsg.pose.position.x = this->odom_.pose.pose.position.x;
		velAndAccVisMsg.pose.position.y = this->odom_.pose.pose.position.y;
		velAndAccVisMsg.pose.position.z = this->odom_.pose.pose.position.z + 0.4;
		velAndAccVisMsg.scale.x = 0.15;
		velAndAccVisMsg.scale.y = 0.15;
		velAndAccVisMsg.scale.z = 0.15;
		velAndAccVisMsg.color.a = 1.0;
		velAndAccVisMsg.color.r = 1.0;
		velAndAccVisMsg.color.g = 1.0;
		velAndAccVisMsg.color.b = 1.0;
		velAndAccVisMsg.lifetime = ros::Duration(0.05);

		double vNorm = currVel.norm();
		double aNorm = currAcc.norm();
		double vNormTgt = targetVel.norm();
		double aNormTgt = targetAcc.norm();
		double posError = (currPos - targetPos).norm();

		std::string velText = "|V|=" + std::to_string(vNorm) + ", |VT|=" + std::to_string(vNormTgt) + "\n|A|=" + std::to_string(aNorm) + ", |AT|=" + std::to_string(aNormTgt) + "\n|PosError|=" + std::to_string(posError);
		velAndAccVisMsg.text = velText;
		this->velAndAccVisPub_.publish(velAndAccVisMsg);
	}
}