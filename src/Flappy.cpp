#include "flappy_dart/Flappy.h"
#include <dart/common/Console.hpp>
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/common/Uri.hpp>
#include <fstream>

using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::common;
using namespace dart::utils;

namespace FWMAV{

Flappy::Flappy(const WorldPtr& world, const std::string& urdf_file, const std::string& config_file){

    mWorld = world;
    if(mWorld == nullptr){
        dterr << "World is not specified, FWMAV will not be added." << std::endl;
        return;
    }

    Uri flappy_urdf(urdf_file);
    DartLoader urdf_loader;
    mFlappy = urdf_loader.parseSkeleton(flappy_urdf);
    if(mFlappy == nullptr){
        dterr << "URDF File not correct, FWMAV will not be added." << std::endl;
        return;
    }

    std::ifstream fin(config_file); 
    std::istreambuf_iterator<char> beg(fin), end;
    mJsonString.append(beg,end);
    fin.close();
    if(!mFlappyConfigJson.Parse(mJsonString)){
        dterr << "Json Config File not correct, FWMAV will not be added." << std::endl;
        return;
    }

    world -> addSkeleton(mFlappy);
    return;
}

void Flappy::reset(){

    mLeftMotor -> reset();
    mRightMotor -> reset();
    mNextDriveTime = 0.0;
    mNextServoTime = 0.0;
    config();
}

void Flappy::init(){

    mFlappyConfigJson[0].Get("wing_length",mWingLength);
    mFlappyConfigJson[0].Get("mean_chord",mMeanChord);
    mFlappyConfigJson[0].Get("r33",m_r33);
    mFlappyConfigJson[0].Get("r22",m_r22);
    mFlappyConfigJson[0].Get("r11",m_r11);
    mFlappyConfigJson[0].Get("r00",m_r00);
    mFlappyConfigJson[0].Get("z_cp2",m_z_cp2);
    mFlappyConfigJson[0].Get("z_cp1",m_z_cp1);
    mFlappyConfigJson[0].Get("z_cp0",m_z_cp0);
    mFlappyConfigJson[0].Get("z_rd",m_z_rd);
    mFlappyConfigJson[0].Get("left_shoulder_width",mLeftShoulderWidth);
    mFlappyConfigJson[0].Get("right_shoulder_width",mRightShoulderWidth);
    mFlappyConfigJson[0].Get("stroke_plane_offset",mStrokePlaneOffset);

    mLeftWing = std::shared_ptr<Wing>(
        new Wing(0, mWingLength, mMeanChord, m_r33, m_r22, m_r11, m_r00,
                m_z_cp2, m_z_cp1, m_z_cp0, m_z_rd, mLeftShoulderWidth, mStrokePlaneOffset
            )
    );
    mRightWing = std::shared_ptr<Wing>(
        new Wing(1, mWingLength, mMeanChord, m_r33, m_r22, m_r11, m_r00,
                m_z_cp2, m_z_cp1, m_z_cp0, m_z_rd, mRightShoulderWidth, mStrokePlaneOffset
            )
    );

    mFlappyConfigJson[0].Get("left_spring_stiffness",mLeftSpringStiffness);
    mFlappyConfigJson[0].Get("right_spring_stiffness",mRightSpringStiffness);
    mFlappyConfigJson[0].Get("left_stroke_lower",mLeftStrokeLower);
    mFlappyConfigJson[0].Get("left_stroke_upper",mLeftStrokeUpper);
    mFlappyConfigJson[0].Get("left_rotate_lower",mLeftRotateLower);
    mFlappyConfigJson[0].Get("left_rotate_upper",mLeftRotateUpper);
    mFlappyConfigJson[0].Get("right_stroke_lower",mRightStrokeLower);
    mFlappyConfigJson[0].Get("right_stroke_upper",mRightStrokeUpper);
    mFlappyConfigJson[0].Get("right_rotate_lower",mRightRotateLower);
    mFlappyConfigJson[0].Get("right_rotate_upper",mRightRotateUpper);
    mFlappyConfigJson[0].Get("pitch_servo_lower",mPitchServoLower);
    mFlappyConfigJson[0].Get("pitch_servo_upper",mPitchServoUpper);
    mFlappyConfigJson[0].Get("servo_internal_friction",mServoInternalFriction);
    mFlappyConfigJson[0].Get("left_stroke_mid",mLeftStrokeMid);
    mFlappyConfigJson[0].Get("right_stroke_mid",mRightStrokeMid);

    mFlappyConfigJson[0]["left_motor_properties"].Get("resistance", mLeftMotorProperties.resistance);
    mFlappyConfigJson[0]["left_motor_properties"].Get("torque_constant", mLeftMotorProperties.torqueConstant);
    mFlappyConfigJson[0]["left_motor_properties"].Get("gear_ratio", mLeftMotorProperties.gearRatio);
    mFlappyConfigJson[0]["left_motor_properties"].Get("mechanical_efficiency", mLeftMotorProperties.mechanicalEfficiency);
    mFlappyConfigJson[0]["left_motor_properties"].Get("friction_coefficient", mLeftMotorProperties.frictionCoefficient);
    mFlappyConfigJson[0]["left_motor_properties"].Get("damping_coefficient", mLeftMotorProperties.dampingCoefficient);
    mFlappyConfigJson[0]["left_motor_properties"].Get("inertia", mLeftMotorProperties.inertia);

    mFlappyConfigJson[0]["right_motor_properties"].Get("resistance", mRightMotorProperties.resistance);
    mFlappyConfigJson[0]["right_motor_properties"].Get("torque_constant", mRightMotorProperties.torqueConstant);
    mFlappyConfigJson[0]["right_motor_properties"].Get("gear_ratio", mRightMotorProperties.gearRatio);
    mFlappyConfigJson[0]["right_motor_properties"].Get("mechanical_efficiency", mRightMotorProperties.mechanicalEfficiency);
    mFlappyConfigJson[0]["right_motor_properties"].Get("friction_coefficient", mRightMotorProperties.frictionCoefficient);
    mFlappyConfigJson[0]["right_motor_properties"].Get("damping_coefficient", mRightMotorProperties.dampingCoefficient);
    mFlappyConfigJson[0]["right_motor_properties"].Get("inertia", mRightMotorProperties.inertia);

    mLeftMotor = std::shared_ptr<Actuator>(
        new Actuator(
            mLeftMotorProperties.resistance,
            mLeftMotorProperties.torqueConstant,
            mLeftMotorProperties.gearRatio,
            mLeftMotorProperties.mechanicalEfficiency,
            mLeftMotorProperties.frictionCoefficient,
            mLeftMotorProperties.dampingCoefficient,
            mLeftMotorProperties.inertia
        )
    );

    mRightMotor = std::shared_ptr<Actuator>(
        new Actuator(
            mRightMotorProperties.resistance,
            mRightMotorProperties.torqueConstant,
            mRightMotorProperties.gearRatio,
            mRightMotorProperties.mechanicalEfficiency,
            mRightMotorProperties.frictionCoefficient,
            mRightMotorProperties.dampingCoefficient,
            mRightMotorProperties.inertia
        )
    );

    mFlappyConfigJson[0].Get("frequency", mFrequency);
    mFlappyConfigJson[0].Get("driver_frequency", mDriverFrequency);
    mFlappyConfigJson[0].Get("servo_frequency", mServoFrequency);
    mDriverDt = 1.0 / mDriverFrequency;
    mServoDt = 1.0 / mServoFrequency;

    mPitchServo = std::shared_ptr<ServoD1302>(
        new ServoD1302(mServoInternalFriction)
    );

    config();
    return;
}

void Flappy::config(){

    mFlappy -> getJoint("left_stroke") -> setPositionLowerLimit(0, mLeftStrokeLower);
    mFlappy -> getJoint("left_stroke") -> setPositionUpperLimit(0, mLeftStrokeUpper);
    mFlappy -> getJoint("left_rotate") -> setPositionLowerLimit(0, mLeftRotateLower);
    mFlappy -> getJoint("left_rotate") -> setPositionUpperLimit(0, mLeftRotateUpper);
    mFlappy -> getJoint("right_stroke") -> setPositionLowerLimit(0, mRightStrokeLower);
    mFlappy -> getJoint("right_stroke") -> setPositionUpperLimit(0, mRightStrokeUpper);
    mFlappy -> getJoint("right_rotate") -> setPositionLowerLimit(0, mRightRotateLower);
    mFlappy -> getJoint("right_rotate") -> setPositionUpperLimit(0, mRightRotateUpper);
    mFlappy -> getJoint("servo_control") -> setPositionLowerLimit(0, mPitchServoLower);
    mFlappy -> getJoint("servo_control") -> setPositionUpperLimit(0, mPitchServoUpper);

    mFlappy -> getJoint("left_stroke") -> setPositionLimitEnforced(true);
    mFlappy -> getJoint("left_rotate") -> setPositionLimitEnforced(true);
    mFlappy -> getJoint("right_stroke") -> setPositionLimitEnforced(true);
    mFlappy -> getJoint("right_rotate") -> setPositionLimitEnforced(true);
    mFlappy -> getJoint("servo_control") -> setPositionLimitEnforced(true);

    mFlappy -> getJoint("left_stroke") -> setSpringStiffness(0, mLeftSpringStiffness * 1.8);
    mFlappy -> getJoint("right_stroke") -> setSpringStiffness(0, mRightSpringStiffness * 1.8);

    mFlappy -> getJoint("left_stroke") -> setRestPosition(0, mLeftStrokeMid);
    mFlappy -> getJoint("right_stroke") -> setRestPosition(0, mRightStrokeMid);

    //simulate the internal friction of servo
    mFlappy -> getJoint("servo_control") -> setCoulombFriction(0, mServoInternalFriction);

    //set init position/velocity
    mFlappy -> getDof("torso_to_world_rot_x") -> setPosition(0.0);  //roll
    mFlappy -> getDof("torso_to_world_rot_x") -> setVelocity(0.0);  //roll rate
    mFlappy -> getDof("torso_to_world_rot_y") -> setPosition(0.0);  //pitch
    mFlappy -> getDof("torso_to_world_rot_y") -> setVelocity(0.0);  //pitch rate
    mFlappy -> getDof("torso_to_world_rot_z") -> setPosition(0.0);  //yaw
    mFlappy -> getDof("torso_to_world_rot_z") -> setVelocity(0.0);  //yaw rate
    mFlappy -> getDof("torso_to_world_pos_x") -> setPosition(0.0);  //x
    mFlappy -> getDof("torso_to_world_pos_x") -> setVelocity(0.0);  //x vel
    mFlappy -> getDof("torso_to_world_pos_y") -> setPosition(0.0);  //y
    mFlappy -> getDof("torso_to_world_pos_y") -> setVelocity(0.0);  //y vel
    mFlappy -> getDof("torso_to_world_pos_z") -> setPosition(0.0);  //z
    mFlappy -> getDof("torso_to_world_pos_z") -> setVelocity(0.0);  //z vel
    // mFlappy -> getDof("torso_to_world") -> setPosition(0.0);
    // mFlappy -> getDof("torso_to_world") -> setVelocity(0.0);

    mFlappy -> getDof("left_stroke") -> setPosition(0.0);   //left stroke
    mFlappy -> getDof("left_stroke") -> setVelocity(0.0);   //left stroke vel
    mFlappy -> getDof("left_rotate") -> setPosition(0.0);   //left rotate
    mFlappy -> getDof("left_rotate") -> setVelocity(0.0);   //left rotate vel
    mFlappy -> getDof("right_stroke") -> setPosition(0.0);  //right stroke
    mFlappy -> getDof("right_stroke") -> setVelocity(0.0);  //right stroke vel
    mFlappy -> getDof("right_rotate") -> setPosition(0.0);  //right rotate
    mFlappy -> getDof("right_rotate") -> setVelocity(0.0);  //right rotate vel
    mFlappy -> getDof("servo_control") -> setPosition(0.0); //pitch servo
    mFlappy -> getDof("servo_control") -> setVelocity(0.0); //pitch servo vel

    return;
}

void Flappy::preDartStep(const double pitch, const double roll, const double yaw,
                         const double throttle, const double time){
    
    updateStates();

    //update aero forces
    Eigen::Vector6d v_upperpart = mFlappy -> getBodyNode("flapping_mechanism") -> getCOMSpatialVelocity();

    mLeftWing -> UpdateStates(
        v_upperpart[0],
        v_upperpart[1],
        v_upperpart[2],
        v_upperpart[3],
        v_upperpart[4],
        v_upperpart[5],
        0,
        0,
        mStates.leftStrokeAngle,
        mStates.leftStrokeVelocity,
        0,
        0,
        mStates.leftRotateAngle,
        mStates.leftRotateVelocity
    );

    mRightWing -> UpdateStates(
        v_upperpart[0],
        v_upperpart[1],
        v_upperpart[2],
        v_upperpart[3],
        v_upperpart[4],
        v_upperpart[5],
        0,
        0,
        mStates.rightStrokeAngle,
        mStates.rightStrokeVelocity,
        0,
        0,
        mStates.rightRotateAngle,
        mStates.rightRotateVelocity
    );
    mLeftWing -> UpdateAeroForce();
    mRightWing -> UpdateAeroForce();

    Eigen::Vector3d left_FN(mLeftWing -> GetNormalForce() * 4.0, 0, 0);
    Eigen::Vector3d right_FN(mRightWing -> GetNormalForce() * 4.0, 0, 0);
    Eigen::Vector3d left_CoP(0, mLeftWing -> GetSpanCoP(), - mLeftWing -> GetChordCoP());
    Eigen::Vector3d right_CoP(0, - mRightWing -> GetSpanCoP(), - mRightWing -> GetChordCoP());
    Eigen::Vector3d left_M_rd(0, mLeftWing -> GetM_rd() * 4.0, 0);
    Eigen::Vector3d right_M_rd(0, mRightWing -> GetM_rd() * 4.0, 0);

    mFlappy -> getBodyNode("left_wing") -> addExtForce(left_FN, left_CoP, true, true);
    mFlappy -> getBodyNode("right_wing") -> addExtForce(right_FN, right_CoP, true, true);
    mFlappy -> getBodyNode("left_wing") -> addExtTorque(left_M_rd, true);
    mFlappy -> getBodyNode("right_wing") -> addExtTorque(right_M_rd, true);
    
    //update motor torque
    if(time >= mNextDriveTime){
        mNextDriveTime += mDriverDt;
        mLeftMotor -> updateDriverVoltage(generateControlSignal(
            throttle * 18.0, roll, 0, 0, time, 0
        ));
        mRightMotor -> updateDriverVoltage(generateControlSignal(
            throttle * 18.0, - roll, 0, 0, time, 0
        ));
    }
    mLeftMotor -> UpdateTorque(mStates.leftStrokeVelocity, mStates.leftStrokeAcceleration);
    mRightMotor -> UpdateTorque(mStates.rightStrokeVelocity, mStates.rightStrokeAcceleration);
    mFlappy -> getDof("left_stroke") -> setForce(mLeftMotor -> GetTorque() * 1.8);
    mFlappy -> getDof("right_stroke") -> setForce(mRightMotor -> GetTorque() * 1.8);

    //update servo forces
    if(time >= mNextServoTime){
        mNextServoTime += mServoDt;
        mPitchServo -> setPoint(pitch);
        mPitchServo -> updateStatesAndTorque(mStates.pitchServoAngle, mStates.pitchServoVelocity);   
    }
    mFlappy -> getDof("servo_control") -> setForce(mPitchServo -> getTorque());

    return;
}

void Flappy::updateStates(){
    
    mStates.rotationMatrix = mFlappy -> getBodyNode("torso") -> getWorldTransform();

    //DEBUG CODE
    // dtmsg << mFlappy -> getBodyNode("torso") -> getCOMSpatialAcceleration() << std::endl;

    mStates.positions[0] = atan2(mStates.rotationMatrix(2,1), mStates.rotationMatrix(2,2));
    mStates.positions[1] = asin(- mStates.rotationMatrix(2,0));
    mStates.positions[2] = atan2(mStates.rotationMatrix(1,0), mStates.rotationMatrix(0,0));
    mStates.positions[3] = mFlappy -> getBodyNode("torso") -> getCOM()[0];
    mStates.positions[4] = mFlappy -> getBodyNode("torso") -> getCOM()[1];
    mStates.positions[5] = mFlappy -> getBodyNode("torso") -> getCOM()[2];

    mStates.velocities = mFlappy -> getBodyNode("torso") -> getCOMSpatialVelocity();
    mStates.accelerations = mFlappy -> getBodyNode("torso") -> getCOMSpatialAcceleration();
    mStates.spatialVelocities = mFlappy -> getBodyNode("torso") -> getCOMLinearVelocity();
    mStates.spatialAccelerations = mFlappy -> getBodyNode("torso") -> getCOMLinearAcceleration();
    
    //left wing
    mStates.leftStrokeAngle = mFlappy -> getDof("left_stroke") -> getPosition();
    mStates.leftStrokeVelocity = mFlappy -> getDof("left_stroke") -> getVelocity();
    mStates.leftStrokeAcceleration = mFlappy -> getDof("left_stroke") -> getAcceleration();
    mStates.leftRotateAngle = mFlappy -> getDof("left_rotate") -> getPosition();
    mStates.leftRotateVelocity = mFlappy -> getDof("left_rotate") -> getVelocity();

    //right wing
    mStates.rightStrokeAngle = mFlappy -> getDof("right_stroke") -> getPosition();
    mStates.rightStrokeVelocity = mFlappy -> getDof("right_stroke") -> getVelocity();
    mStates.rightStrokeAcceleration = mFlappy -> getDof("right_stroke") -> getAcceleration();
    mStates.rightRotateAngle = mFlappy -> getDof("right_rotate") -> getPosition();
    mStates.rightRotateVelocity = mFlappy -> getDof("right_rotate") -> getVelocity();

    //pitch servo
    mStates.pitchServoAngle = mFlappy -> getDof("servo_control") -> getPosition();
    mStates.pitchServoVelocity = mFlappy -> getDof("servo_control") -> getVelocity();

    return;
}

/*********************
 * note: control signal bounds refer to origin author
 * lower bound : 0.0    -3.0    -3.5    -0.15
 * upper bound : 18.0   3.0     3.5     0.15
 *********************/
double Flappy::generateControlSignal(const double amp, const double delta,
                                     const double bias, const double split_cycle,
                                     const double time, const double phase){

    double res, t;
    double T = 1.0 / mFrequency;
    t = time + phase / 360.0 * T;
    double period = floor(t / T);
    t -= period * T;

    double sigma = 0.5 + split_cycle;
    if(t >= 0 && t < sigma / mFrequency){
        res = (amp + delta) * cos(2* M_PI * mFrequency * t / (2 * sigma)) + bias;
    }
    else{
        if(t >= sigma / mFrequency && t < 1.0 / mFrequency){
            res = (amp + delta) * cos((2* M_PI * mFrequency * t - 2 * M_PI) / (2 * (1 - sigma))) + bias;
        }
        else{
            res = 0;
        }
    }
    return res;
}

}