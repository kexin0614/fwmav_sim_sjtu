#include "sensor/Madgwick.h"
#include "sensor/VirtualSensor.h"

#define SPIN_RATE_LIMIT 20

#define ATTITUDE_RESET_QUIET_TIME 250000   // 250ms - gyro quiet period after disarm before attitude reset
#define ATTITUDE_RESET_GYRO_LIMIT 15       // 15 deg/sec - gyro limit for quiet period
#define ATTITUDE_RESET_KP_GAIN    25.0     // dcmKpGain value to use during attitude reset
#define ATTITUDE_RESET_ACTIVE_TIME 500000  // 500ms - Time to wait for attitude to converge at high gain
#define GPS_COG_MIN_GROUNDSPEED 500        // 500cm/s minimum groundspeed for a gps heading to be considered valid

namespace Sensor{

Madgwick::Madgwick(){

    canUseGPSHeading = false;
    smallAngleCosZ = 0;
    attitudeIsEstablished = false;

    q = {.w=1, .x=0, .y=0, .z=0};
    qP = {.ww=1, .wx=0, .wy=0, .wz=0, .xx=0, .xy=0, .xz=0, .yy=0, .yz=0, .zz=0};

    imuConfig = {.dcm_kp = 2500, .dcm_ki = 2500, .small_angle = 25};

    imuRuntimeConfig.dcm_kp = imuConfig.dcm_kp / 10000.0f;
    imuRuntimeConfig.dcm_ki = imuConfig.dcm_ki / 10000.0f;

    imuInit();

}

void Madgwick::imuQuaternionComputeProducts(quaternion *quat, quaternionProducts *quatProd)
{
    quatProd->ww = quat->w * quat->w;
    quatProd->wx = quat->w * quat->x;
    quatProd->wy = quat->w * quat->y;
    quatProd->wz = quat->w * quat->z;
    quatProd->xx = quat->x * quat->x;
    quatProd->xy = quat->x * quat->y;
    quatProd->xz = quat->x * quat->z;
    quatProd->yy = quat->y * quat->y;
    quatProd->yz = quat->y * quat->z;
    quatProd->zz = quat->z * quat->z;
}

void Madgwick::imuComputeRotationMatrix(){

    imuQuaternionComputeProducts(&q, &qP);

    rMat[0][0] = 1.0f - 2.0f * qP.yy - 2.0f * qP.zz;
    rMat[0][1] = 2.0f * (qP.xy + -qP.wz);
    rMat[0][2] = 2.0f * (qP.xz - -qP.wy);

    rMat[1][0] = 2.0f * (qP.xy - -qP.wz);
    rMat[1][1] = 1.0f - 2.0f * qP.xx - 2.0f * qP.zz;
    rMat[1][2] = 2.0f * (qP.yz + -qP.wx);

    rMat[2][0] = 2.0f * (qP.xz + -qP.wy);
    rMat[2][1] = 2.0f * (qP.yz - -qP.wx);
    rMat[2][2] = 1.0f - 2.0f * qP.xx - 2.0f * qP.yy;
}

float Madgwick::calculateThrottleAngleScale(uint16_t throttle_correction_angle)
{
    return (1800.0f / M_PI) * (900.0f / throttle_correction_angle);
}

void Madgwick::imuConfigure(uint16_t throttle_correction_angle, uint8_t throttle_correction_value)
{
    smallAngleCosZ = cos(degreesToRadians(imuConfig.small_angle));

    throttleAngleScale = calculateThrottleAngleScale(throttle_correction_angle);

    throttleAngleValue = throttle_correction_value;
}

void Madgwick::imuInit()
{
    q = {.w=1, .x=0, .y=0, .z=0};
    integralFBx = 0.0; integralFBy = 0.0; integralFBz = 0.0;

    canUseGPSHeading = false;
    attitudeIsEstablished = false;
    imuComputeRotationMatrix();
}

float Madgwick::invSqrt(float x)
{
    return 1.0f / sqrt(x);
}

void Madgwick::imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                        bool useAcc, float ax, float ay, float az,
                                        bool useMag, float mx, float my, float mz,
                                        bool useCOG, float courseOverGround, const float dcmKpGain)
{
    // Calculate general spin rate (rad/s)
    const float spin_rate = sqrt(sq(gx) + sq(gy) + sq(gz));

    // Use raw heading error (from GPS or whatever else)
    float ex = 0, ey = 0, ez = 0;
    if (useCOG) {
        while (courseOverGround >  M_PI) {
            courseOverGround -= (2.0f * M_PI);
        }

        while (courseOverGround < -M_PI) {
            courseOverGround += (2.0f * M_PI);
        }

        const float ez_ef = (- sin(courseOverGround) * rMat[0][0] - cos(courseOverGround) * rMat[1][0]);

        ex = rMat[2][0] * ez_ef;
        ey = rMat[2][1] * ez_ef;
        ez = rMat[2][2] * ez_ef;
    }
    
    // Use measured magnetic vector
    if(useMag){
        float recipMagNorm = sq(mx) + sq(my) + sq(mz);
        if (useMag && recipMagNorm > 0.01f) {
            // Normalise magnetometer measurement
            recipMagNorm = invSqrt(recipMagNorm);
            mx *= recipMagNorm;
            my *= recipMagNorm;
            mz *= recipMagNorm;

            // For magnetometer correction we make an assumption that magnetic field is perpendicular to gravity (ignore Z-component in EF).
            // This way magnetic field will only affect heading and wont mess roll/pitch angles

            // (hx; hy; 0) - measured mag field vector in EF (assuming Z-component is zero)
            // (bx; 0; 0) - reference mag field vector heading due North in EF (assuming Z-component is zero)
            const float hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
            const float hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
            const float bx = sqrt(hx * hx + hy * hy);

            // magnetometer error is cross product between estimated magnetic north and measured magnetic north (calculated in EF)
            const float ez_ef = -(hy * bx);

            // Rotate mag error vector back to BF and accumulate
            ex += rMat[2][0] * ez_ef;
            ey += rMat[2][1] * ez_ef;
            ez += rMat[2][2] * ez_ef;
        }
    }

    // Use measured acceleration vector
    float recipAccNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipAccNorm > 0.01f) {
        // Normalise accelerometer measurement
        recipAccNorm = invSqrt(recipAccNorm);
        ax *= recipAccNorm;
        ay *= recipAccNorm;
        az *= recipAccNorm;

        // Error is sum of cross product between estimated direction and measured direction of gravity
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    // Compute and apply integral feedback if enabled
    if (imuRuntimeConfig.dcm_ki > 0.0f) {
        // Stop integrating if spinning beyond the certain limit
        if (spin_rate < degreesToRadians(SPIN_RATE_LIMIT)) {
            const float dcmKiGain = imuRuntimeConfig.dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    // integral error scaled by Ki
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    } else {
        integralFBx = 0.0f;    // prevent integral windup
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    // Apply proportional and integral feedback
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    // Integrate rate of change of quaternion
    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    quaternion buffer;
    buffer.w = q.w;
    buffer.x = q.x;
    buffer.y = q.y;
    buffer.z = q.z;

    q.w += (-buffer.x * gx - buffer.y * gy - buffer.z * gz);
    q.x += (+buffer.w * gx + buffer.y * gz - buffer.z * gy);
    q.y += (+buffer.w * gy - buffer.x * gz + buffer.z * gx);
    q.z += (+buffer.w * gz + buffer.x * gy - buffer.y * gx);

    // Normalise quaternion
    float recipNorm = invSqrt(sq(q.w) + sq(q.x) + sq(q.y) + sq(q.z));
    q.w *= recipNorm;
    q.x *= recipNorm;
    q.y *= recipNorm;
    q.z *= recipNorm;

    // Pre-compute rotation matrix from quaternion
    imuComputeRotationMatrix();

    attitudeIsEstablished = true;
}

void Madgwick::imuUpdateEulerAngles()
{
    // Disable Headfree mode
    attitude.roll = atan2(rMat[2][1], rMat[2][2]);
    attitude.pitch = (0.5f * M_PI) - acos(-rMat[2][0]);
    attitude.yaw = -atan2(rMat[1][0], rMat[0][0]);

    if (attitude.yaw < 0.0) {
        attitude.yaw += 2 * M_PI;
    }
}

bool Madgwick::imuIsAccelerometerHealthy(float ax, float ay, float az)
{
    float accMagnitudeSq = 0;
    accMagnitudeSq = ax * ax + ay * ay + az * az;

    accMagnitudeSq = accMagnitudeSq / sq(STANDARD_GRAVITY);

    // Accept accel readings only in range 0.9g - 1.1g
    return (0.81f < accMagnitudeSq) && (accMagnitudeSq < 1.21f);
}

float Madgwick::imuCalcKpGain()
{
    // delete a lot of thing, if need, get them from imu.c
    return imuRuntimeConfig.dcm_kp;
}

void Madgwick::imuUpdateAttitude(float time, float gx, float gy, float gz, float ax, float ay, float az,
                                 float mx, float my, float mz)
{
    static float previousIMUUpdateTime;
    bool useAcc = false;
    bool useMag = false;
    bool useCOG = false; // Whether or not correct yaw via imuMahonyAHRSupdate from our ground course
    float courseOverGround = 0; // To be used when useCOG is true.  Stored in Radians

    const float deltaT = time - previousIMUUpdateTime;
    previousIMUUpdateTime = time;

    // if (sensors(SENSOR_MAG) && compassIsHealthy()){
    //     useMag = true;
    // }

    // useAcc = imuIsAccelerometerHealthy(ax, ay, az);
    useAcc = true;

    // all angle (rate) in radians
    imuMahonyAHRSupdate(deltaT, gx, gy, gz, useAcc, ax, ay, az,
                        useMag, 0, 0, 0, useCOG, courseOverGround,  imuCalcKpGain());

    imuUpdateEulerAngles();

    // Update the throttle correction for angle and supply it to the mixer
    int throttleAngleCorrection = 0;
    if (throttleAngleValue) {
        throttleAngleCorrection = calculateThrottleAngleCorrection();
    }
    // mixerSetThrottleAngleCorrection(throttleAngleCorrection);
}

int Madgwick::calculateThrottleAngleCorrection()
{
    /*
    * Use 0 as the throttle angle correction if we are inverted, vertical or with a
    * small angle < 0.86 deg
    * TODO: Define this small angle in config.
    */
    if (getCosTiltAngle() <= 0.015f) {
        return 0;
    }
    int angle = lrintf(acos(getCosTiltAngle()) * throttleAngleScale);
    if (angle > 900)
        angle = 900;
    return lrintf(throttleAngleValue * sin(angle / (900.0f * M_PI / 2.0f)));
}

float Madgwick::getCosTiltAngle(void)
{
    return rMat[2][2];
}


static float degreesToRadians(float degrees)
{
    return degrees * RAD;
}



}   // namespace sensor