#ifndef SENSOR_MADGWICK_H_
#define SENSOR_MADGWICK_H_

#include <stdint.h>
#include <math.h>

namespace Sensor
{

#define RAD (M_PI / 180.0f)
#ifndef sq
#define sq(x) ((x)*(x))
#endif

static const int XYZ_AXIS_COUNT = 3;

typedef int32_t timeDelta_t;
typedef uint64_t timeUs_t;

typedef struct{
    float w,x,y,z;
} quaternion;

typedef struct {
    float ww,wx,wy,wz,xx,xy,xz,yy,yz,zz;
} quaternionProducts;

typedef struct {
    uint16_t dcm_kp;                        // DCM filter proportional gain ( x 10000)
    uint16_t dcm_ki;                        // DCM filter integral gain ( x 10000)
    uint8_t small_angle;
} imuConfig_t;

typedef struct {
    float dcm_ki;
    float dcm_kp;
} imuRuntimeConfig_t;

typedef struct {
    // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
    double roll;
    double pitch;
    double yaw;
} attitudeEulerAngles_t;

typedef enum {
    X = 0,
    Y,
    Z
} axis_e;


class Madgwick{
    
    public:
        Madgwick();
        void imuConfigure(uint16_t, uint8_t);
        void imuInit();
        void imuUpdateAttitude(float time, float gx, float gy, float gz, float ax, float ay, float az,
                                 float mx, float my, float mz);
        
    
    protected:
        void imuQuaternionComputeProducts(quaternion*, quaternionProducts*);
        void imuComputeRotationMatrix();
        float calculateThrottleAngleScale(uint16_t);
        float invSqrt(float);
        void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                bool useAcc, float ax, float ay, float az,
                                bool useMag, float mx, float my, float mz,
                                bool useCOG, float courseOverGround, const float dcmKpGain);
        void imuUpdateEulerAngles();
        bool imuIsAccelerometerHealthy(float ax, float ay, float az);
        float imuCalcKpGain();
        int calculateThrottleAngleCorrection();
        float getCosTiltAngle(void);

    public:
        attitudeEulerAngles_t attitude;     // output angles in radians

        bool canUseGPSHeading;
        imuConfig_t imuConfig;

    private:
        float rMat[3][3];       // rotation matrix
        quaternion q;           // rotation quaternion

        float throttleAngleScale;
        int throttleAngleValue;
        float smallAngleCosZ;
        imuRuntimeConfig_t imuRuntimeConfig;
        bool attitudeIsEstablished;
        quaternionProducts qP;

        float integralFBx, integralFBy, integralFBz;    // integral error terms scaled by Ki

};

static float degreesToRadians(float degrees);


}

#endif