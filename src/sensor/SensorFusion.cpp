#include "sensor/SensorFusion.h"

namespace Sensor{

SensorFusion::SensorFusion(const std::string& sensor_config_file, const std::shared_ptr<FWMAV::Flappy> data_source){
    
    //read sim config file
    std::ifstream fin(sensor_config_file); 
    std::istreambuf_iterator<char> beg(fin), end;
    mSensorConfigJsonString.append(beg,end);
    fin.close();
    if(!mSensorConfigJson.Parse(mSensorConfigJsonString)){
        dterr << "Sensor Json Config File not correct, Sensor cannot be added." << std::endl;
        return;
    }
    mSensorConfigJson.Get("acc_freq",mAccFreq);
    mSensorConfigJson.Get("gyro_freq",mGyroFreq);
    mSensorConfigJson.Get("mag_freq",mMagFreq);
    mSensorConfigJson.Get("acc_lpf", mAccLpf);
    mSensorConfigJson.Get("gyro_lpf", mGyroLpf);
    mSensorConfigJson.Get("mag_lpf", mMagLpf);
    mSensorConfigJson.Get("acc_soft_lpf", mAccSoftLpf);
    mSensorConfigJson.Get("gyro_soft_lpf", mGyroSoftLpf);
    mSensorConfigJson.Get("mag_soft_lpf", mMagSoftLpf);
    mSensorConfigJson.Get("fusion_freq", mFusionFreq);
    mSensorConfigJson.Get("fusion_delay", mFusionDelay);
    mAccDt = 1.0 / mAccFreq;
    mGyroDt = 1.0 / mGyroFreq;
    mFusionDt = 1.0 / mFusionFreq;

    mSensorConfigJson.Get("gyro_offset_x", mGyroOffset[0]);
    mSensorConfigJson.Get("gyro_offset_y", mGyroOffset[1]);
    mSensorConfigJson.Get("gyro_offset_z", mGyroOffset[2]);
    
    mSensor.setDataSource(data_source);
    mSensor.setGyroOffset(mGyroOffset[0], mGyroOffset[1], mGyroOffset[2]);
    reset();
}

void SensorFusion::reset(){
    
    mSensor.configAndInit(mAccFreq, mGyroFreq, mMagFreq, mAccLpf, mGyroLpf, mMagLpf, 0.0);

    mNextAccProcessTime = 0.0;
    mNextGyroProcessTime = 0.0;
    mNextFusionTime = 0.0;
    
    for(int i = 0; i < 3; i ++){
        mAccAccumulated[i] = 0.0;
        mGyroAccumulated[i] = 0.0;
        mMagAccumulated[i] = 0.0;
    }

    mAccCount = 0;
    mGyroCount = 0;
    mMagCount = 0;

    mMadgwickFilter.imuInit();
    PoseRPY[0] = 0.0;
    PoseRPY[1] = 0.0;
    PoseRPY[2] = 0.0;
}

void SensorFusion::run(double time){
    
    mSensor.run(time);
    
    if(time >= mNextAccProcessTime){
        mNextAccProcessTime += mAccDt;
        mAccCount ++;
        for(int i = 0; i < 3; i ++)
            mAccAccumulated[i] += mSensor.mIMUData.acc[i];
    }

    if(time >= mNextGyroProcessTime){
        mNextGyroProcessTime += mGyroDt;
        mGyroCount ++;
        for(int i = 0; i < 3; i ++)
            mGyroAccumulated[i] += mSensor.mIMUData.gyro[i];
    }

    if(time >= mNextFusionTime){
        mNextFusionTime += mFusionDt;

        mMadgwickFilter.imuUpdateAttitude(
            time,
            mGyroAccumulated[0] / mGyroCount, mGyroAccumulated[1] / mGyroCount, mGyroAccumulated[2] / mGyroCount,
            mAccAccumulated[0] / mAccCount, mAccAccumulated[1] / mAccCount, mAccAccumulated[2] / mAccCount,
            0,  0,  0);
        
        mGyroCount = 0; mAccCount = 0;
        for(int i = 0; i < 3; i ++){
            mGyroAccumulated[i] = 0.0;
            mAccAccumulated[i] = 0.0;
        }
            
        PoseRPY[0] = mMadgwickFilter.attitude.roll;
        PoseRPY[1] = mMadgwickFilter.attitude.pitch;
        PoseRPY[2] = mMadgwickFilter.attitude.yaw;
    }

}

}   // end of namespace Sensor