#include "flappy_dart/SimEnv.h"
#include <dart/common/Console.hpp>
#include <fstream>

SimEnv::SimEnv(const dart::simulation::WorldPtr& world, const std::string& config_file)
    : dart::gui::osg::RealTimeWorldNode(world),
      mPitchRateFilter2(200),
      mRollRateFilter2(200)
{

    char program_path[1024];
    int i;
    if(i = readlink("/proc/self/exe", program_path, 1024)){
        
        while(program_path[i] != '/' && i--);
        program_path[i + 1] = 0;
        dtmsg << "The program is running at " << program_path << std::endl;

    }
    else{
        dterr << "Couldn't find Program path, check the permission." << std::endl;
        return;
    }
    mProgramPathString = program_path;

    //read sim config file
    std::ifstream fin(mProgramPathString + config_file); 
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
        new FWMAV::Flappy(world, mProgramPathString + mMAVUrdfPath, mProgramPathString + mMAVConfigPath)
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

    //virtual sensors settings
    mSensor.configAndInit(350, 350, 350, 30, 90, 90, 0);
    mSensor.setDataSource(mFlappy);

    //class private var init
    mFWMAVThrottle = 0.0;
    mFWMAVPitch = 0.0;
    mFWMAVRoll = 0.0;
    mFWMAVYaw = 0.0;

    //DEBUG CODE
    pid_pub = nh.advertise<visualization_msgs::Marker>("/imu_debug", 10);

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
    mSensor.configAndInit(350, 350, 350, 30, 90, 90, 0);
}

void SimEnv::customPreStep()
{
    mFlappy -> preDartStep(mFWMAVPitch, mFWMAVRoll, 0, mFWMAVThrottle, mWorld -> getTime());
    mSensor.run(mWorld -> getTime());
    dtmsg << mSensor.mIMUData.acc[2];
    return;
}

void SimEnv::customPostStep()
{   
    //smooth filter
    mFilteredRollRate = mRollRateFilter2.filterApply(mFlappy -> mStates.velocities[0]);
    mFilteredPitchRate = mPitchRateFilter2.filterApply(mFlappy -> mStates.velocities[1]);

    if(mWorld -> getTime() > mNextControlTime){

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
        visualization_msgs::Marker msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.type = visualization_msgs::Marker::ARROW;
        geometry_msgs::Point p;
        p.x = 0; p.y = 0; p.z = 0;
        msg.points.push_back(p);
        p.x = mSensor.mIMUData.acc[0];
        p.y = mSensor.mIMUData.acc[1];
        p.z = mSensor.mIMUData.acc[2];
        msg.points.push_back(p);
        msg.scale.x = 0.5; msg.scale.y = 1; msg.scale.z = 0;
        msg.color.a = 1.0; msg.color.r = 1.0;
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
