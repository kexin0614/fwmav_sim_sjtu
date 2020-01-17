#include "flappy_dart/SimEnv.h"
#include <dart/common/Console.hpp>
#include <fstream>

SimEnv::SimEnv(const dart::simulation::WorldPtr& world, const std::string& config_file)
    : dart::gui::osg::RealTimeWorldNode(world),
      mPitchRateFilter2(200),
      mRollRateFilter2(200)
{
    //read sim config file
    std::ifstream fin(config_file); 
    std::istreambuf_iterator<char> beg(fin), end;
    mConfigJsonString.append(beg,end);
    fin.close();
    if(!mSimConfigJson.Parse(mConfigJsonString)){
        dterr << "Json Config File not correct, environment config failed." << std::endl;
        return;
    }
    mSimConfigJson.Get("f_sim",mSimFrequency);
    mSimConfigJson.Get("f_control",mControlFrequency);
    mSimConfigJson.Get("df_visual",mRealTimeFactor);
    mSimConfigJson.Get("mav_urdf", mMAVUrdfPath);
    mSimConfigJson.Get("mav_config", mMAVConfigPath);
    mControlDt = 1.0 / mControlFrequency;
    
    //add flapper to world
    mWorld = world;
    mFlappy = std::shared_ptr<FWMAV::Flappy>(
        new FWMAV::Flappy(world, mMAVUrdfPath, mMAVConfigPath)
    );
    mFlappy -> init();
    mFlappy -> config();

    //world settings
    mWorld -> setTimeStep(1.0 / mSimFrequency);
    this -> setTargetRealTimeFactor(mRealTimeFactor);

    //pid controllers & filters
    mPitchPIDController = std::shared_ptr<FWMAV::SerialPIDController>(
        new FWMAV::SerialPIDController(
            mControlFrequency
        )
    );
    mRollPIDController = std::shared_ptr<FWMAV::SerialPIDController>(
        new FWMAV::SerialPIDController(
            mControlFrequency
        )
    );
    mPitchFilter.biquadFilterInitLPF(30, mControlDt);
    mPitchRateFilter.biquadFilterInitLPF(30, mControlDt);
    mRollFilter.biquadFilterInitLPF(30, mControlDt);
    mRollRateFilter.biquadFilterInitLPF(30, mControlDt);
    mFilteredPitch = 0.0;
    mFilteredPitchRate = 0.0;
    mFilteredRoll = 0.0;
    mFilteredRollRate = 0.0;

    mFWMAVThrottle = 0.0;
    mFWMAVPitch = 0.0;
    mFWMAVRoll = 0.0;
    mFWMAVYaw = 0.0;

    //DEBUG CODE
    pid_pub = nh.advertise<geometry_msgs::Vector3>("/pid_debug", 10);

}

void SimEnv::reset(){

    mNextControlTime = 0.0;
    mFlappy -> reset();
    mWorld -> reset();
    mPitchPIDController -> reset();
    mRollPIDController -> reset();
    mFilteredPitch = 0.0;
    mFilteredPitchRate = 0.0;
    mFilteredRoll = 0.0;
    mFilteredRollRate = 0.0;
}

void SimEnv::customPreStep()
{
    mFlappy -> preDartStep(mFWMAVPitch, mFWMAVRoll, 0, mFWMAVThrottle, mWorld -> getTime());
    return;
}

void SimEnv::customPostStep()
{   
    //smooth filter
    mFilteredRollRate = mRollRateFilter2.filterApply(mFlappy -> mStates.velocities[0]);
    mFilteredPitchRate = mPitchRateFilter2.filterApply(mFlappy -> mStates.velocities[1]);

    if(mWorld -> getTime() > mNextControlTime){
        
        //DEBUG CODE
        geometry_msgs::Vector3 msg;
        msg.y = mFilteredRollRate * 180.0 / M_PI;

        mNextControlTime += mControlDt;
        
        mFilteredRollRate = mRollRateFilter.biquadFilterApply(mFilteredRollRate);
        mFilteredRoll = mRollFilter.biquadFilterApply(mFlappy -> mStates.positions[0]);
        mFilteredPitchRate = mPitchRateFilter.biquadFilterApply(mFilteredPitchRate);
        mFilteredPitch = mPitchFilter.biquadFilterApply(mFlappy -> mStates.positions[1]);

        mPitchPIDController -> setPoint(0.0);
        mPitchPIDController -> updateStatesAndCalculate(
            mFilteredPitch,
            mFilteredPitchRate
        );
        mFWMAVPitch = mPitchPIDController -> getOutput();

        mRollPIDController -> setPoint(0.0);
        mRollPIDController -> updateStatesAndCalculate(
            mFilteredRoll,
            mFilteredRollRate
        );
        mFWMAVRoll = mRollPIDController -> getOutput() * 5.0;
        
        //DEBUG CODE
        msg.z = mFilteredRollRate * 180.0 / M_PI;
        msg.x = mFWMAVRoll;
        pid_pub.publish(msg);
    }
}

void SimEnv::customPreRefresh()
{
    // Use this function to execute custom code before each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
}

void SimEnv::setFWMAVThrottle(double throttle){
    mFWMAVThrottle = throttle;
}

void SimEnv::setCustomRealTimeFactor(double realtime_factor){
    mRealTimeFactor = realtime_factor;
    setTargetRealTimeFactor(mRealTimeFactor);
}
