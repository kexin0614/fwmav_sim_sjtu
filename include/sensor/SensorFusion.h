#ifndef SENSOR_SENSORFUSION_H_
#define SENSOR_SENSORFUSION_H_

#include "sensor/VirtualSensor.h"
#include "sensor/Filter.h"
#include "sensor/Madgwick.h"
#include "cjson/CJsonObject.hpp"
#include <stdint.h>
#include <string>

namespace Sensor{

class SensorFusion{

    public:
        SensorFusion(const std::string& sensor_config_file, const std::shared_ptr<FWMAV::Flappy> data_source = nullptr);
        void run(double time);
        void reset();
    
    public:
        VirtualSensor mSensor;
        double PoseRPY[3];      // in radians

    private:
        Madgwick mMadgwickFilter;
        neb::CJsonObject mSensorConfigJson;
        std::string mSensorConfigJsonString;

        // sensor configs
        int mAccFreq, mGyroFreq, mMagFreq;
        double mAccLpf, mGyroLpf, mMagLpf;
        double mAccSoftLpf, mGyroSoftLpf, mMagSoftLpf;
        int mFusionFreq;
        double mFusionDelay;
        double mGyroOffset[3];

        // sensor readings
        double mAccAccumulated[3], mGyroAccumulated[3], mMagAccumulated[3];
        int mAccCount, mGyroCount, mMagCount;

        // tmp variables
        double mAccDt, mFusionDt, mGyroDt;
        double mNextFusionTime;
        double mNextAccProcessTime;
        double mNextGyroProcessTime;

};


}   // end of namespace Sensor


#endif