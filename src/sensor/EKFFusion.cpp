#include "sensor/EKFFusion.h"

namespace Sensor{

EKFFusion::EKFFusion(int sensor_frequency){

    mSensorDt = 1.0 / sensor_frequency;
    double k_I_x = 5300e-9, k_I_y = 4900e-9, k_I_z = 3700e-9;
    mI1 = (k_I_y - k_I_z) / k_I_x;
    mI2 = (k_I_x - k_I_z) / k_I_y;
    mI3 = (k_I_x - k_I_y) / k_I_z;

    double RC;
    RC = 1.0 / (2 * M_PI * 5);
    mAlpha = mSensorDt / (RC + mSensorDt);
    RC = 1.0 / (2 * M_PI * 136);
    mAlphaGyro = mSensorDt / (RC + mSensorDt);
    reset();
}

void EKFFusion::reset(){

    mXPo.setZero();
    mXPri.setZero();
    mGyro.setZero();
    mXPoCache.setZero();
    mGyroCache.setZero();
    mIndexNow = mIndexPrev = 0;
    mOutPitch = mOutRoll = mOutYaw = 0.0;
    mIMUGx = mIMUGy = mIMUGz = 0.0;
    mViconRoll = mViconPitch = mViconYaw = 0.0;
}

void EKFFusion::runEKF2(bool is_vicon_available){

    if(!is_vicon_available){

        mXPo[4] = mGyro[0] = mIMUGx;
        mXPo[5] = mGyro[1] = mIMUGy;
        mXPo[6] = mGyro[2] = mIMUGz;
        updatePrediction();
        mXPo = mXPri;
    }
    else{

        mIndexPrev = mIndexNow - cViconDelayStep;
        int i, j;
        double tmp[4];
        j = mIndexPrev < 0 ? mIndexPrev + cEKFCacheSize : mIndexPrev;
        mXPo = mXPoCache.col(j);

        for(j = mIndexPrev + 1; j <= mIndexNow + 1; j ++){
            i = j < 0 ? j + cEKFCacheSize : j;
            if(j == mIndexPrev + 1){
                mGyro = mGyroCache.col(i);
                eulerAngleToQuaternion(tmp, mViconRoll, mViconPitch, mViconYaw);
                mXPo[0] = tmp[0]; mXPo[1] = tmp[1]; mXPo[2] = tmp[2]; mXPo[3] = tmp[3];
                mXPo[4] = mGyro[0];
                mXPo[5] = mGyro[1];
                mXPo[6] = mGyro[2];
                updatePrediction();
                mXPo = mXPri;
            }
            else{
                if(j == mIndexNow + 1){
                    mGyro[0] = mIMUGx;
                    mGyro[1] = mIMUGy;
                    mGyro[2] = mIMUGz;
                }
                else mGyro = mGyroCache.col(i);

                mXPo[4] = mGyro[0];
                mXPo[5] = mGyro[1];
                mXPo[6] = mGyro[2];
                updatePrediction();
                mXPo = mXPri;
            }
            if(j != mIndexNow + 1)
                mXPoCache.col(i) = mXPo;
        }
    }

    mIndexNow ++;
    if(mIndexNow > cEKFCacheSize -1)
        mIndexNow = mIndexNow - cEKFCacheSize;
    
    mXPoCache.col(mIndexNow) = mXPo;
    mGyroCache.col(mIndexNow) = mGyro;

    return;
}

void EKFFusion::updatePrediction(){

        mXPri[0] = mXPo[0] - 0.5 * mXPo[1] * mXPo[4] * mSensorDt - 0.5 * mXPo[2] * mXPo[5] * mSensorDt - 0.5 * mXPo[3] * mXPo[6] * mSensorDt;
		mXPri[1] = mXPo[1] + 0.5 * mXPo[0] * mXPo[4] * mSensorDt - 0.5 * mXPo[3] * mXPo[5] * mSensorDt + 0.5 * mXPo[2] * mXPo[6] * mSensorDt;
		mXPri[2] = mXPo[2] + 0.5 * mXPo[3] * mXPo[4] * mSensorDt + 0.5 * mXPo[0] * mXPo[5] * mSensorDt - 0.5 * mXPo[1] * mXPo[6] * mSensorDt;
		mXPri[3] = mXPo[3] - 0.5 * mXPo[2] * mXPo[4] * mSensorDt + 0.5 * mXPo[1] * mXPo[5] * mSensorDt + 0.5 * mXPo[0] * mXPo[6] * mSensorDt;
		mXPri[4] = mXPo[4] + (mI1 * mXPo[5] * mXPo[6]) * mSensorDt;
		mXPri[5] = mXPo[5] + (mI2 * mXPo[4] * mXPo[6]) * mSensorDt;
		mXPri[6] = mXPo[6] + (mI3 * mXPo[4] * mXPo[5]) * mSensorDt;

}

}