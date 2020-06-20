#include "sensor/VirtualSensor.h"

namespace Sensor
{

VirtualSensor::VirtualSensor(std::shared_ptr<FWMAV::Flappy> data_source){

    mDataSource = data_source;
}

void VirtualSensor::setDataSource(std::shared_ptr<FWMAV::Flappy> data_source){
    mDataSource = data_source;
}

void VirtualSensor::configAndInit(int acc_freq, int gyro_freq, int mag_freq,
                            double acc_lpf, double gyro_lpf, double mag_lpf,
                            double delay){
    
    mAccDt = 1.0 / acc_freq;
    mGyroDt = 1.0 / gyro_freq;
    mMagDt = 1.0 / mag_freq;
    mNextAccTime = 0.0;
    mNextGyroTime = 0.0;
    mNextMagTime = 0.0;

    mAccLpf[0].biquadFilterInitLPF(acc_lpf, mAccDt);
    mAccLpf[1].biquadFilterInitLPF(acc_lpf, mAccDt);
    mAccLpf[2].biquadFilterInitLPF(acc_lpf, mAccDt);
    mGyroLpf[0].biquadFilterInitLPF(gyro_lpf, mGyroDt);
    mGyroLpf[1].biquadFilterInitLPF(gyro_lpf, mGyroDt);
    mGyroLpf[2].biquadFilterInitLPF(gyro_lpf, mGyroDt);
    mMagLpf[0].biquadFilterInitLPF(mag_lpf, mMagDt);
    mMagLpf[1].biquadFilterInitLPF(mag_lpf, mMagDt);
    mMagLpf[2].biquadFilterInitLPF(mag_lpf, mMagDt);

    return;
}

void VirtualSensor::run(double time){

    if(mDataSource == nullptr){
        
    }

    if(time >= mNextAccTime){
        mNextAccTime += mAccDt;
        mIMUData.acc[0] = mAccLpf[0].biquadFilterApply(mDataSource -> mStates.accelerations[3] + mDataSource -> mStates.rotationMatrix(2,0) * 9.8);
        mIMUData.acc[1] = mAccLpf[1].biquadFilterApply(mDataSource -> mStates.accelerations[4] + mDataSource -> mStates.rotationMatrix(2,1) * 9.8);
        mIMUData.acc[2] = mAccLpf[2].biquadFilterApply(mDataSource -> mStates.accelerations[5] + mDataSource -> mStates.rotationMatrix(2,2) * 9.8);
    }
    return;
}


} // namespace Sensor
