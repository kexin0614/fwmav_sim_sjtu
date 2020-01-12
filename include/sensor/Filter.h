#ifndef SENSOR_FILTER_H_
#define SENSOR_FILTER_H_

#include <stdint.h>
#include <math.h>

#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - 2nd order butterworth*/

namespace Sensor
{

class BiquadFilter{

    protected:
        struct mCoeff_t{
            double b0, b1, b2, a1, a2;
            double x1, x2, y1, y2;
        } mCoeff;

    public:
        typedef enum {
            FILTER_LPF,    // 2nd order Butterworth section
            FILTER_NOTCH,
            FILTER_BPF,
        } biquadFilterType;

    public:
        BiquadFilter(){}

        /* sets up a biquad Filter */
        void biquadFilterInitLPF(double filterFreq, double sampleDt)
        {
            biquadFilterInit(filterFreq, sampleDt, BIQUAD_Q, FILTER_LPF);
        }

        void biquadFilterInit(double filterFreq, double sampleDt, double Q, biquadFilterType filterType)
        {
            // setup variables
            const double omega = 2.0f * M_PI * filterFreq * sampleDt;
            const double sn = sin(omega);
            const double cs = cos(omega);
            const double alpha = sn / (2.0f * Q);

            double b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

            switch (filterType) {
            case FILTER_LPF:
                // 2nd order Butterworth (with Q=1/sqrt(2)) / Butterworth biquad section with Q
                // described in http://www.ti.com/lit/an/slaa447/slaa447.pdf
                b0 = (1 - cs) * 0.5f;
                b1 = 1 - cs;
                b2 = (1 - cs) * 0.5f;
                a0 = 1 + alpha;
                a1 = -2 * cs;
                a2 = 1 - alpha;
                break;
            case FILTER_NOTCH:
                b0 =  1;
                b1 = -2 * cs;
                b2 =  1;
                a0 =  1 + alpha;
                a1 = -2 * cs;
                a2 =  1 - alpha;
                break;
            case FILTER_BPF:
                b0 = alpha;
                b1 = 0;
                b2 = -alpha;
                a0 = 1 + alpha;
                a1 = -2 * cs;
                a2 = 1 - alpha;
                break;
            }

            // precompute the coefficients
            mCoeff.b0 = b0 / a0;
            mCoeff.b1 = b1 / a0;
            mCoeff.b2 = b2 / a0;
            mCoeff.a1 = a1 / a0;
            mCoeff.a2 = a2 / a0;

            // zero initial samples
            mCoeff.x1 = mCoeff.x2 = 0;
            mCoeff.y1 = mCoeff.y2 = 0;
        }

        /* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
        double biquadFilterApply(double input)
        {
            const double result = mCoeff.b0 * input + mCoeff.x1;
            mCoeff.x1 = mCoeff.b1 * input - mCoeff.a1 * result + mCoeff.x2;
            mCoeff.x2 = mCoeff.b2 * input - mCoeff.a2 * result;
            return result;
        }

};

class SmoothFilter{

    public:
        SmoothFilter(int smooth_count){
            mSmoothCount = smooth_count;
            mDataFifo = new double[smooth_count];
            int i;
            for(i = 0; i < smooth_count; i ++)
                mDataFifo[i] = 0;
            mFrontPointer = 0;
            mSum = 0.0;
        }

        ~SmoothFilter(){
            delete mDataFifo;
        }

        double filterApply(double input){
            mSum -= mDataFifo[mFrontPointer];
            mDataFifo[mFrontPointer] = input;
            mSum += mDataFifo[mFrontPointer];
            mFrontPointer = mFrontPointer == mSmoothCount - 1 ? 0 : mFrontPointer + 1;
            return mSum / mSmoothCount;
        }
    
    private:
        double* mDataFifo;
        int mFrontPointer;
        int mSmoothCount;
        double mSum;
};


} // namespace Sensor

#endif