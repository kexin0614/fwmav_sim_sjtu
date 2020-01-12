#ifndef FWMAV_SERVO_D1302_H_
#define FWMAV_SERVO_D1302_H_


namespace FWMAV{

class ServoD1302{
    public:
        ServoD1302(double internal_friction){
            m_kp = 0.183;
            m_kd = 1.5e-3;     //0.016Nm/(35rad/s~2000deg/s)
            m_ki = 0.0;
            m_u = 0.0;
            m_y = 0.0;
            m_ydot = 0.0;
            m_int_friction = internal_friction;
        }
        
        void setPoint(double r){
            m_r = r;
        }

        void updateStatesAndTorque(double y, double ydot){
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
            return;
        }

        double getTorque(){ //default operating frequency: 500Hz
            return m_u;
        }
    protected:
        double m_kp, m_ki, m_kd;
        double m_y, m_ydot;
        double m_u;
        double m_r;
        double m_int_friction;
        const double MAX_OUT_TORQUE = 0.006;
    
    private:
        double m_int_value;
};

}

#endif