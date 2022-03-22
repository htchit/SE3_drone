#ifndef __CONTROLLER_H
#define __CONTROLLER_H

#include <sensor_msgs/Imu.h>
#include <mavros_msgs/AttitudeTarget.h>

#include "input.h"

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d jerk;

	double yaw;
	double head_rate;
};

struct Odom_Data_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d w;
	Eigen::Quaterniond q;
};

struct Controller_Output_t
{
	double thrust;
	double roll_rate;
	double pitch_rate;
	double yaw_rate;

	double normalized_thrust;

	Eigen::Quaterniond orientation;

	Eigen::Vector3d des_v_real;
};

class Controller
{
public:
	Parameter_t param;

	ros::Publisher ctrl_FCU_pub;
	ros::Publisher debug_roll_pub;
	ros::Publisher debug_pitch_pub;
	ros::ServiceClient set_FCU_mode;

	Eigen::Matrix3d Kp;
	Eigen::Matrix3d Kv;
	Eigen::Matrix3d Kvi;
	Eigen::Matrix3d Ka;
	double Kyaw;

	Eigen::Vector3d int_e_v;

	Controller(Parameter_t &);
	void config_gain(Parameter_t &);
	void update(const Desired_State_t &des, const Odom_Data_t &odom, Controller_Output_t &u);

	Controller_Output_t computeNominalReferenceInputs(const Desired_State_t &reference_state, const Odom_Data_t &attitude_estimate) const;
	Eigen::Quaterniond computeDesiredAttitude(const Eigen::Vector3d &desired_acceleration, const double reference_heading, const Eigen::Quaterniond &attitude_estimate) const;
	Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d &x_B_prototype, const Eigen::Vector3d &x_C, const Eigen::Vector3d &y_C, const Eigen::Quaterniond &attitude_estimate) const;
	Eigen::Vector3d computeFeedBackControlBodyrates(const Eigen::Quaterniond &desired_attitude, const Eigen::Quaterniond &attitude_estimate);
	bool almostZero(const double value) const;
	bool almostZeroThrust(const double thrust_value) const;
	void publish_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
	void publish_zero_ctrl(const ros::Time &stamp);

private:
};

#endif
