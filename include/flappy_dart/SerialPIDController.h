#ifndef FWMAV_PID_CONTROLLER_H_
#define FWMAV_PID_CONTROLLER_H_

#include <math.h>

namespace FWMAV{

class SerialPIDController{
    public:
        SerialPIDController(int freq = 500){
            m_dt = 1.0 / freq;

            m_ext_kp = 3.0;
            m_ext_kd = 0.0;
            m_ext_ki = 0.0;
            m_int_kp = 0.05;
            m_int_kd = 0.02;
            m_int_ki = 0.0;

            reset();
        }
        
        void setPoint(double r){
            m_r = r * 180.0 / M_PI;
        }

        void reset(){
            m_r = 0.0;
            m_int_i = 0.0;
            m_ext_i = 0.0;
            m_u = 0.0;
            m_u_ext = 0.0;
        }

        void updateStatesAndCalculate(double y, double ydot){
            m_y = y * 180.0 / M_PI;
            m_ydot = ydot * 180.0 / M_PI;
            m_ext_err_ll = m_ext_err_l;
            m_ext_err_l = m_ext_err;
            m_ext_err = m_r - m_y;

            m_u_ext = m_ext_kp * m_ext_err + m_ext_kd * (m_ext_err - m_ext_err_l);
            if(m_u_ext + m_ext_i + m_dt * m_ext_ki < MAX_OUT_ANGLE_RATE &&
               m_u_ext + m_ext_i + m_dt * m_ext_ki > - MAX_OUT_ANGLE_RATE)
               m_ext_i += m_dt * m_ext_i;
            m_u_ext += m_ext_i;

            m_int_err_ll = m_int_err_l;
            m_int_err_l = m_int_err;
            m_int_err = m_u_ext - m_ydot;
            m_u = m_int_kp * m_int_err + m_int_kd * (m_int_err - m_int_err_l);
            if(m_u + m_int_i + m_dt * m_int_ki < MAX_OUT_ANGLE &&
               m_u + m_int_i + m_dt * m_int_ki > - MAX_OUT_ANGLE)
               m_int_i += m_dt * m_int_i;
            m_u += m_int_i;
        }

        double getOutput(){
            if(m_u > MAX_OUT_ANGLE)
                return MAX_OUT_ANGLE * M_PI / 180.0;
            else{
                if(m_u < - MAX_OUT_ANGLE)
                    return - MAX_OUT_ANGLE * M_PI / 180.0;
                else
                    return m_u * M_PI / 180.0;
            }
        }
    public:
        double m_dt;

        double m_int_kp, m_int_ki, m_int_kd;
        double m_ext_kp, m_ext_ki, m_ext_kd;
        double m_y, m_ydot;
        double m_u;
        double m_r;
        const double MAX_OUT_ANGLE_RATE = 800.0;
        const double MAX_OUT_ANGLE = 30.0;
    
    // private:
        double m_ext_err, m_ext_err_l, m_ext_err_ll;
        double m_int_err, m_int_err_l, m_int_err_ll;
        double m_ext_i, m_int_i;
        double m_u_ext;
};

}

#endif