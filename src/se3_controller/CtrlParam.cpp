#include "CtrlParam.h"

Parameter_t::Parameter_t(const ros::NodeHandle &nh)
{
    nh.getParam("Kp0", Kp0);
    nh.getParam("Kp1", Kp1);
    nh.getParam("Kp2", Kp2);
    nh.getParam("Kv0", Kv0);
    nh.getParam("Kv1", Kv1);
    nh.getParam("Kv2", Kv2);
    nh.getParam("Kvi0", Kvi0);
    nh.getParam("Kvi1", Kvi1);
    nh.getParam("Kvi2", Kvi2);
    nh.getParam("Ka0", Ka0);
    nh.getParam("Ka1", Ka1);
    nh.getParam("Ka2", Ka2);
    nh.getParam("Kyaw", Kyaw);
    nh.getParam("mass", mass);
    nh.getParam("gra", gra);
    nh.getParam("hov_percent", hov_percent);
    nh.getParam("full_thrust", full_thrust);
    nh.getParam("ctrl_rate", ctrl_rate);
}
