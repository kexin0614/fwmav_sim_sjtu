/*************************  FWMAV Simulation  *************************
* Version 0.3
* Fan Fei		Jan 2018
* FWMAV simulation with dual motor driven robotic flapper
* PID controller using split cycle mechanism
***********************************************************************
*/

#include "flappy_dart/Actuator.h"

namespace FWMAV{

Actuator::Actuator(double k_resistance, double k_torque_constant, double k_gear_ratio,
				   double k_mechanical_efficiency, double k_friction_coefficient,
				   double k_damping_coefficient, double k_inertia):
		k_resistance_(k_resistance),
		k_torque_constant_(k_torque_constant),
		k_gear_ratio_(k_gear_ratio),
		k_mechanical_efficiency_(k_mechanical_efficiency),
		k_friction_coefficient_(k_friction_coefficient),
		k_damping_coefficient_(k_damping_coefficient),
		k_inertia_(k_inertia)
{
	reset();
	doNothing();
}


Actuator::~Actuator()
{

}

void Actuator::reset(){

	inertia_torque_ = 0;
	damping_torque_ = 0;
	friction_torque_ = 0;
	magnetic_torque_ = 0;
	motor_torque_ = 0;
	voltage_ = 0;
	current_ = 0;
	back_EMF_ = 0;
	output_torque_ = 0;
}

void Actuator::doNothing()
{
	output_torque_ = 0;
}

void Actuator::updateDriverVoltage(double voltage)
{
	voltage_ = voltage;
}

void Actuator::UpdateTorque(double stroke_velocity, double stroke_acceleration)
{
	psi_dot_ = stroke_velocity;
	psi_ddot_ = stroke_acceleration;
	motor_vel_ = psi_dot_*k_gear_ratio_;
	motor_accel_ = psi_ddot_*k_gear_ratio_;
	if (psi_dot_>0)
		sign_ = 1;
	else if (psi_dot_<0)
		sign_ = -1;
	else
		sign_ = 0;
	
	// if (variable_resistance_ == 0)
	// 	resistance_ = k_resistance_;
	// else
	// 	resistance_ = variable_resistance_;

	back_EMF_ = k_torque_constant_*motor_vel_;
	current_ = (voltage_-back_EMF_)/k_resistance_;

	inertia_torque_ = k_inertia_*motor_accel_;
	damping_torque_ = k_damping_coefficient_*motor_vel_;
	friction_torque_ = k_friction_coefficient_*sign_;
	magnetic_torque_ = k_torque_constant_*current_;

	motor_torque_ = magnetic_torque_-inertia_torque_-damping_torque_-friction_torque_;

	output_torque_ = motor_torque_*k_gear_ratio_*k_mechanical_efficiency_;
	//std::cout << "inertia_torque_=" << inertia_torque_ << " damping_torque_=" << damping_torque_ << " friction_torque_=" << friction_torque_ << " magnetic_torque_=" << magnetic_torque_ << " motor_torque_=" << motor_torque_ << " output_torque_=" << output_torque_ << std::endl;
}

double Actuator::GetTorque()
{
	return output_torque_;
}

// for debugging

double Actuator::GetMotorTorque()
{
	return motor_torque_;
}

double Actuator::GetMagTorque()
{
	return magnetic_torque_;
}

double Actuator::GetInerTorque()
{
	return inertia_torque_;
}

double Actuator::GetDampTorque()
{
	return damping_torque_;
}

double Actuator::GetFricTorque()
{
	return friction_torque_;
}

double Actuator::GetBEMF()
{
	return back_EMF_;
}

double Actuator::GetCurrent()
{
	return current_;
}

/*
I_a = (V-K_a psi_dot)/(R_a)

T_motor = T_load/(eta*N_g)
*/

}