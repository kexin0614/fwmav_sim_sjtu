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

void VirtualSensor::setGyroOffset(double off_x, double off_y, double off_z){

    mGyroOffset[0] = off_x;
    mGyroOffset[1] = off_y;
    mGyroOffset[2] = off_z;
}

void VirtualSensor::run(double time){

    if(mDataSource == nullptr){
        dterr << "No DataSource is given for VirtualSensor." << std::endl;
        return;
    }

    if(time >= mNextAccTime){
        mNextAccTime += mAccDt;
        mIMUData.acc[0] = mAccLpf[0].biquadFilterApply(mDataSource -> mStates.accelerations[3] + mDataSource -> mStates.rotationMatrix(2,0) * STANDARD_GRAVITY);
        mIMUData.acc[1] = mAccLpf[1].biquadFilterApply(mDataSource -> mStates.accelerations[4] + mDataSource -> mStates.rotationMatrix(2,1) * STANDARD_GRAVITY);
        mIMUData.acc[2] = mAccLpf[2].biquadFilterApply(mDataSource -> mStates.accelerations[5] + mDataSource -> mStates.rotationMatrix(2,2) * STANDARD_GRAVITY);
    }

    if(time >= mNextGyroTime){
        mNextGyroTime += mGyroDt;
        mIMUData.gyro[0] = mGyroLpf[0].biquadFilterApply(mDataSource -> mStates.velocities[0]) + mGyroOffset[0];
        mIMUData.gyro[1] = mGyroLpf[1].biquadFilterApply(mDataSource -> mStates.velocities[1]) + mGyroOffset[1];
        mIMUData.gyro[2] = mGyroLpf[2].biquadFilterApply(mDataSource -> mStates.velocities[2]) + mGyroOffset[2];
    }

    if(time >= mNextMagTime){
        mNextMagTime += mMagDt;
        mIMUData.mag[0] = mMagLpf[0].biquadFilterApply(mDataSource -> mStates.rotationMatrix(0,0));
        mIMUData.mag[1] = mMagLpf[1].biquadFilterApply(mDataSource -> mStates.rotationMatrix(0,1));
        mIMUData.mag[2] = mMagLpf[2].biquadFilterApply(mDataSource -> mStates.rotationMatrix(0,2));
    }
    return;
}


} // namespace Sensor
