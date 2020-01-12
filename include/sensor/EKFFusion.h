#ifndef SENSOR_EKFFUSION_H_
#define SENSOR_EKFFUSION_H_

#include <Eigen/Dense>
#include <math.h>

namespace Sensor
{

const int cEKFCacheSize = 50;
const int cViconDelayStep = 16;

class EKFFusion{

    public:
        EKFFusion(int sensor_frequency);
        void reset();
        void run();
        struct FusionState{
            double roll, pitch, yaw;
            double rollRate, pitchRate, yawRate;

        } mState;


    protected:
        void quaternionToEulerAngle(double* res, double w, double x, double y, double z);
        void eulerAngleToQuaternion(double* res, double roll, double pitch, double yaw);
        void updatePrediction();
        void runEKF2(bool is_vicon_available);

    protected:
        Eigen::Matrix<double, 7, 1> mXPri;
        Eigen::Matrix<double, 7, 1> mXPo;
        Eigen::Matrix<double, 3, 1> mGyro;
        Eigen::Matrix<double, 4, cEKFCacheSize> mXPoCache;
        Eigen::Matrix<double, 3, cEKFCacheSize> mGyroCache;

        double mSensorDt;
        int mIndexNow;
        int mIndexPrev;
        double mOutRoll, mOutPitch, mOutYaw;

    private:
        double mAlpha;
        double mAlphaGyro;
        double mI1, mI2, mI3;
        double mIMUGx, mIMUGy, mIMUGz;
        double mViconRoll, mViconPitch, mViconYaw;
};


} // namespace Sensor

#endif