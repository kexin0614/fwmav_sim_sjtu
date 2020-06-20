#ifndef FWMAV_FLAPPY_H_
#define FWMAV_FLAPPY_H_

#include <dart/dart.hpp>
#include <string>
#include "flappy_dart/Wing.h"
#include "flappy_dart/Actuator.h"
#include "flappy_dart/ServoD1302.h"
#include "cjson/CJsonObject.hpp"
#include <Eigen/Dense>

namespace FWMAV{

class Flappy{
    public:
        Flappy(const dart::simulation::WorldPtr& world,
               const std::string& urdf_file,
               const std::string& config_file);
        void init();
        void config();
        void preDartStep(const double pitch, const double roll, const double yaw,
                         const double throttle, const double time);
        void updateStates();
        void reset();

    private:
        double generateControlSignal(const double amp, const double delta,
                                     const double bias, const double split_cycle,
                                     const double time, const double phase);
    
    public:
        struct{
            Eigen::Isometry3d rotationMatrix;       //rotation matrix from world to torso
            Eigen::Vector6d positions;              //body positions: roll pitch yaw x y z
            Eigen::Vector6d velocities;             //body velocities: roll pitch yaw x y z
            Eigen::Vector6d accelerations;
            Eigen::Vector3d spatialVelocities;
            Eigen::Vector3d spatialAccelerations;
            double leftStrokeAngle;
            double leftStrokeVelocity;
            double leftStrokeAcceleration;
            double leftRotateAngle;
            double leftRotateVelocity;
            double rightStrokeAngle;
            double rightStrokeVelocity;
            double rightStrokeAcceleration;
            double rightRotateAngle;
            double rightRotateVelocity;
            double pitchServoAngle;
            double pitchServoVelocity;
        }mStates;

    protected:
        dart::simulation::WorldPtr mWorld;
        dart::dynamics::SkeletonPtr mFlappy;
        std::shared_ptr<Wing> mLeftWing;
        std::shared_ptr<Wing> mRightWing;
        std::shared_ptr<Actuator> mLeftMotor;
        std::shared_ptr<Actuator> mRightMotor;
        std::shared_ptr<ServoD1302> mPitchServo;
        
        neb::CJsonObject mFlappyConfigJson;

    protected:
        int mFrequency;
        //Wing Aero Config
        double mWingLength;
        double mMeanChord;
        double m_r33;
        double m_r22;
        double m_r11;
        double m_r00;
        double m_z_cp2;
        double m_z_cp1;
        double m_z_cp0;
        double m_z_rd;
        double mLeftShoulderWidth;
        double mRightShoulderWidth;
        double mStrokePlaneOffset;
        //Wing Dynamics Config
        double mLeftSpringStiffness;
        double mRightSpringStiffness;
        double mLeftStrokeLower;
        double mLeftStrokeUpper;
        double mLeftRotateLower;
        double mLeftRotateUpper;
        double mRightStrokeLower;
        double mRightStrokeUpper;
        double mRightRotateLower;
        double mRightRotateUpper;
        double mPitchServoUpper;
        double mPitchServoLower;
        double mServoInternalFriction;
        double mLeftStrokeMid;
        double mRightStrokeMid;
        //Actuator(Motor) Config
        struct motorProperties{
            double resistance;
            double torqueConstant;
            double gearRatio;
            double mechanicalEfficiency;
			double frictionCoefficient;
			double dampingCoefficient;
            double inertia;
        }mLeftMotorProperties, mRightMotorProperties;
        //Sim Configs
        int mDriverFrequency;
        double mDriverDt;
        int mServoFrequency;
        double mServoDt;

    private:
        std::string mJsonString;
        double mNextDriveTime;
        double mNextServoTime;
};

}

#endif