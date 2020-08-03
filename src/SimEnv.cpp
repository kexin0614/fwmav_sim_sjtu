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
    mSimConfigJson.Get("sensor_config", mSensorConfigPath);
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

    //Sensor Delay
    mPitchDelay = std::shared_ptr<Sensor::PureDelay<double> >(
        new Sensor::PureDelay<double>(mControlDt, 0.03)
    );
    mPitchRateDelay = std::shared_ptr<Sensor::PureDelay<double> >(
        new Sensor::PureDelay<double>(mControlDt, 0.03)
    );
    mRollDelay = std::shared_ptr<Sensor::PureDelay<double> >(
        new Sensor::PureDelay<double>(mControlDt, 0.03)
    );
    mRollRateDelay = std::shared_ptr<Sensor::PureDelay<double> >(
        new Sensor::PureDelay<double>(mControlDt, 0.03)
    );

    //virtual sensors settings
    mSensor = std::shared_ptr<Sensor::SensorFusion>(
        new Sensor::SensorFusion(mProgramPathString + mSensorConfigPath, mFlappy)
    );

    //class private var init
    mFWMAVThrottle = 0.0;
    mFWMAVPitch = 0.0;
    mFWMAVRoll = 0.0;
    mFWMAVYaw = 0.0;

    //DEBUG CODE
    pid_pub = nh.advertise<geometry_msgs::Vector3>("/imu_debug", 100);
    reference_vicon_pub = nh.advertise<geometry_msgs::Vector3>("/vicon_reference", 100);
    acc_vis_pub = nh.advertise<visualization_msgs::Marker>("/acc_vis", 50);

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

    mSensor -> reset();

    //Sensor Delay
    mPitchDelay -> reset();
    mPitchRateDelay -> reset();
    mRollDelay -> reset();
    mRollRateDelay -> reset();
}

void SimEnv::customPreStep()
{
    mFlappy -> preDartStep(mFWMAVPitch, mFWMAVRoll, 0, mFWMAVThrottle, mWorld -> getTime());
    mSensor -> run(mWorld -> getTime());
}

void SimEnv::customPostStep()
{   
    //smooth filter
    // mFilteredRollRate = mRollRateFilter2.filterApply(mFlappy -> mStates.velocities[0]);
    // mFilteredPitchRate = mPitchRateFilter2.filterApply(mFlappy -> mStates.velocities[1]);
    // mFilteredRollRate = mFlappy -> mStates.velocities[0];
    // mFilteredPitchRate = mFlappy -> mStates.velocities[1];

    if(mWorld -> getTime() > mNextControlTime){

        mNextControlTime += mControlDt;
        
        // mFilteredRollRate = mRollRateDelay -> applyDelay(mRollRateFilter.biquadFilterApply(mFilteredRollRate));
        // mFilteredRoll = mRollDelay -> applyDelay(mRollFilter.biquadFilterApply(mFlappy -> mStates.positions[0]));
        // mFilteredRollRate = mRollRateFilter.biquadFilterApply(mFilteredRollRate);
        // mFilteredRoll = mRollFilter.biquadFilterApply(mFlappy -> mStates.positions[0]);
        // mFilteredPitchRate = mPitchRateDelay -> applyDelay(mPitchRateFilter.biquadFilterApply(mFilteredPitchRate));
        // mFilteredPitch = mPitchDelay -> applyDelay(mPitchFilter.biquadFilterApply(mFlappy -> mStates.positions[1]));

        mPitchPIDController -> setPoint(0.0);
        mPitchPIDController -> updateStatesAndCalculate(
            mSensor -> PoseRPY[1],
            mSensor -> mSensor.mIMUData.gyro[1]
        );
        mFWMAVPitch = mPitchPIDController -> getOutput() /*0.5*/;

        mRollPIDController -> setPoint(0.0);
        mRollPIDController -> updateStatesAndCalculate(
            mSensor -> PoseRPY[0],
            mSensor -> mSensor.mIMUData.gyro[0]
        );
        mFWMAVRoll = mRollPIDController -> getOutput() * 5.0;
        
        //DEBUG CODE
        geometry_msgs::Vector3 msg;
        msg.x = mSensor -> PoseRPY[0] * 180.0 / M_PI;
        msg.y = mSensor -> PoseRPY[1] * 180.0 / M_PI;
        msg.z = mSensor -> PoseRPY[2] * 180.0 / M_PI;
        if(msg.x > 180.0) msg.x -= 360.0;
        if(msg.y > 180.0) msg.y -= 360.0;
        if(msg.z > 180.0) msg.z -= 360.0;
        pid_pub.publish(msg);

        msg.x = mFlappy -> mStates.positions[0] * 180.0 / M_PI;
        msg.y = mFlappy -> mStates.positions[1] * 180.0 / M_PI;
        msg.z = mFlappy -> mStates.positions[2] * 180.0 / M_PI;
        if(msg.x > 180.0) msg.x -= 360.0;
        if(msg.y > 180.0) msg.y -= 360.0;
        if(msg.z > 180.0) msg.z -= 360.0;
        reference_vicon_pub.publish(msg);

        //send tf transform
        tf::Transform transform;
        tf::Quaternion q;
        q.setRPY(mFlappy -> mStates.positions[0] , mFlappy -> mStates.positions[1], mFlappy -> mStates.positions[2]);
        transform.setRotation(q);
        transform.setOrigin(tf::Vector3(0,
                                        0,
                                        0));
        tf_pub.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

        visualization_msgs::Marker marker;
        geometry_msgs::Point pt_tmp;
        marker.header.frame_id = "base_link";
        marker.id = 101;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::MODIFY;
        marker.lifetime = ros::Duration(0.5);
        marker.color.a = 0.8;
        marker.color.r = 1; marker.color.g = 0; marker.color.b = 0;
        marker.scale.x = 0.01; marker.scale.y = 0.02; marker.scale.z = 0.03;
        marker.points.clear();
        pt_tmp.x = pt_tmp.y = pt_tmp.z = 0.0;
        marker.points.push_back(pt_tmp);
        pt_tmp.x = mSensor -> mSensor.mIMUData.acc[0] / 40.0;
        pt_tmp.y = mSensor -> mSensor.mIMUData.acc[1] / 40.0;
        pt_tmp.z = mSensor -> mSensor.mIMUData.acc[2] / 40.0;
        marker.points.push_back(pt_tmp);
        acc_vis_pub.publish(marker);
    }
}

void SimEnv::customPreRefresh()
{
    // Use this function to execute custom code before each time that the
    // window is rendered. This function can be deleted if it does not need
    // to be used.
    // mViewer -> getCameraManipulator() -> setHomePosition(
    //     ::osg::Vec3(2.57f, 3.14f, 1.64f),
    //     ::osg::Vec3(mFlappy -> mStates.positions[3], mFlappy -> mStates.positions[4], mFlappy -> mStates.positions[5]),
    //     ::osg::Vec3(-0.24f, -0.25f, 0.94f), true);
    // mViewer -> setCameraManipulator(mViewer -> getCameraManipulator());
    
}

void SimEnv::setFWMAVThrottle(double throttle){
    mFWMAVThrottle = throttle;
}

void SimEnv::setCustomRealTimeFactor(double realtime_factor){
    mRealTimeFactor = realtime_factor;
    setTargetRealTimeFactor(mRealTimeFactor);
}

void SimEnv::setOsgViewer(osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer){
    mViewer = viewer;
}