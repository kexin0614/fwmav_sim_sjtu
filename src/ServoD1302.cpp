#include "flappy_dart/ServoD1302.h"

namespace FWMAV{


ServoD1302::ServoD1302(double internal_friction){
    m_kp = 0.5;
    m_kd = 4.0e-3;     //0.016Nm/(35rad/s~2000deg/s)
    m_ki = 0.0;
    m_u = 0.0;
    m_y = 0.0;
    m_ydot = 0.0;
    m_int_friction = internal_friction;

    //DEBUG_CODE
    dbg_pub = nh.advertise<geometry_msgs::Vector3>("/servo_pid",10);
}

void ServoD1302::setPoint(double r){
    m_r = r;
}

void ServoD1302::updateStatesAndTorque(double y, double ydot){
    m_y = y;
    m_ydot = ydot;

    m_u = m_kp * (m_r - m_y) - m_kd * m_ydot;
    
    if(m_u > MAX_OUT_TORQUE)
        m_u = MAX_OUT_TORQUE + m_int_friction;
    else{
        if(m_u < - MAX_OUT_TORQUE)
            m_u = - MAX_OUT_TORQUE - m_int_friction;
        else
            m_u = m_u > 0 ? m_u + m_int_friction : m_u - m_int_friction;
    }
    
    //DEBUG_CODE
    geometry_msgs::Vector3 msg;
    msg.x = m_u;
    dbg_pub.publish(msg);

    return;
}

double ServoD1302::getTorque(){ //default operating frequency: 500Hz
    return m_u;
}


}