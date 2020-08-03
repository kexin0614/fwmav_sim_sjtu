#ifndef FWMAV_SERVO_D1302_H_
#define FWMAV_SERVO_D1302_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

namespace FWMAV{

class ServoD1302{
    public:
        ServoD1302(double internal_friction);
        void setPoint(double r);
        void updateStatesAndTorque(double y, double ydot);
        double getTorque(); //default operating frequency: 500Hz

    protected:
        double m_kp, m_ki, m_kd;
        double m_y, m_ydot;
        double m_u;
        double m_r;
        double m_int_friction;
        const double MAX_OUT_TORQUE = 0.006;
    
    private:
        double m_int_value;

        //DEBUG_CODE
        ros::NodeHandle nh;
        ros::Publisher dbg_pub;
};

}

#endif