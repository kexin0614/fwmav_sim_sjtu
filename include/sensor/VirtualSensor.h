#ifndef SENSOR_VIRTUAL_SENSOR_H_
#define SENSOR_VIRTUAL_SENSOR_H_

#include "flappy_dart/Flappy.h"
#include <dart/common/Console.hpp>
#include "sensor/Filter.h"

namespace Sensor
{

class VirtualSensor{

    public:
        VirtualSensor(std::shared_ptr<FWMAV::Flappy> data_source = nullptr);
        void setDataSource(std::shared_ptr<FWMAV::Flappy> data_source);
        void configAndInit(int acc_freq, int gyro_freq, int mag_freq,
                    double acc_lpf, double gyro_lpf, double mag_lpf,
                    double delay);
        void run(double time);

    public:
        struct IMUData_t
        {
            double acc[3];
            double gyro[3];
            double mag[3];
        } mIMUData;

    private:
        std::shared_ptr<FWMAV::Flappy> mDataSource;
        BiquadFilter mAccLpf[3];
        BiquadFilter mGyroLpf[3];
        BiquadFilter mMagLpf[3];

        double mAccDt, mGyroDt, mMagDt;
        
        double mNextAccTime;
        double mNextGyroTime;
        double mNextMagTime;
    

};


} // namespace Sensor


#endif