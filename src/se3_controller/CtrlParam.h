#ifndef __CtrlPARAM_H
#define __CtrlPARAM_H

#include <ros/ros.h>

class Parameter_t
{
public:
	Parameter_t(const ros::NodeHandle &nh);
	double Kp0, Kp1, Kp2;
	double Kv0, Kv1, Kv2;
	double Kvi0, Kvi1, Kvi2;
	double Ka0, Ka1, Ka2;
	double Kyaw;
	double Krp;

	double mass;
	double gra;
	double hov_percent;
	double full_thrust;

	double ctrl_rate;
};

#endif