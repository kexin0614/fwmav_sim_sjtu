/*************************  FWMAV Simulation  *************************
* Version 0.3
* Fan Fei		Jan 2018
* FWMAV simulation with dual motor driven robotic flapper
* PID controller using split cycle mechanism
***********************************************************************
*/

#ifndef FWMAV_ACTUATOR_H_
#define FWMAV_ACTUATOR_H_

#include <vector>
#include <iostream>
#include <Eigen/Dense>

namespace FWMAV{

class Actuator {
public:
	Actuator(double k_resistance, double k_torque_constant, double k_gear_ratio,
			 double k_mechanical_efficiency, double k_friction_coefficient,
			 double k_damping_coefficient, double k_inertia);
	virtual ~Actuator();

	void reset();
	void doNothing();
	void updateDriverVoltage(double voltage);
	void UpdateTorque(double stroke_velocity, double stroke_acceleration);

	double GetTorque();
	double GetMotorTorque();
	double GetMagTorque();
	double GetInerTorque();
	double GetDampTorque();
	double GetFricTorque();
	double GetBEMF();
	double GetCurrent();

protected:
	double* stroke_velocity_;
	double* stroke_acceleration_;

	const double k_resistance_;
	const double k_torque_constant_;
	const double k_gear_ratio_;
	const double k_mechanical_efficiency_;
	const double k_friction_coefficient_;
	const double k_damping_coefficient_;
	const double k_inertia_;

	double psi_dot_;
	double psi_ddot_;
	double motor_vel_;
	double motor_accel_;
	double sign_;
	double voltage_;
	double current_;
	double back_EMF_;
	double variable_resistance_;

	double inertia_torque_;
	double damping_torque_;
	double friction_torque_;
	double magnetic_torque_;
	double motor_torque_;
	double output_torque_;
};

}

#endif