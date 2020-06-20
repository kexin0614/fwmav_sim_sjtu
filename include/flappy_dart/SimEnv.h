#ifndef SIMENV_H_
#define SIMENV_H_

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/common/Console.hpp>
#include "flappy_dart/Flappy.h"
#include "cjson/CJsonObject.hpp"
#include <string>
#include "flappy_dart/SerialPIDController.h"
#include "sensor/Filter.h"
#include "sensor/VirtualSensor.h"

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>

class SimEnv : public dart::gui::osg::RealTimeWorldNode
{
  public:
    SimEnv(const dart::simulation::WorldPtr& world, const std::string& config_file);

    void reset();
    void customPreStep();
    void customPostStep();
    void customPreRefresh();

    void customPostRefresh()
    {
      // Use this function to execute custom code after each time that the
      // window is rendered. This function can be deleted if it does not need
      // to be used.
    }

    

  protected:
    std::shared_ptr<FWMAV::Flappy> mFlappy;
    dart::simulation::WorldPtr mWorld;

    std::shared_ptr<FWMAV::SerialPIDController> mPitchPIDController;
    Sensor::BiquadFilter mPitchFilter;
    Sensor::BiquadFilter mPitchRateFilter;
    Sensor::SmoothFilter mPitchRateFilter2;
    std::shared_ptr<FWMAV::SerialPIDController> mRollPIDController;
    Sensor::BiquadFilter mRollFilter;
    Sensor::BiquadFilter mRollRateFilter;
    Sensor::SmoothFilter mRollRateFilter2;
    double mFilteredPitch;
    double mFilteredPitchRate;
    double mFilteredRoll;
    double mFilteredRollRate;

    Sensor::VirtualSensor mSensor;

    neb::CJsonObject mSimConfigJson;
    int mSimFrequency;
    int mControlFrequency;
    int mSensorFrequency;
    double mRealTimeFactor;
    std::string mMAVUrdfPath;
    std::string mMAVConfigPath;
  
  public:
    void setFWMAVThrottle(double throttle);
    void setCustomRealTimeFactor(double realtime_factor);
    void setFWMAVPitch(double pitch);
    void setFWMAVRoll(double roll);
    void setFWMAVYaw(double yaw);

  private:
    double mFWMAVThrottle;
    double mFWMAVPitch;
    double mFWMAVRoll;
    double mFWMAVYaw;
    double mControlDt;
    double mNextControlTime;

    std::string mConfigJsonString;

  //DEBUG CODE
    ros::Publisher pid_pub;
    ros::NodeHandle nh;

};

#endif