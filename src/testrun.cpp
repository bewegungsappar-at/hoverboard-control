
#define M_PI 3.14159265358979323846
#include <math.h>
#include "testrun.h"
#include <stdint.h>

Testrun::State Testrun::getState() {
    return state;
}

void Testrun::setState(Testrun::State newstate) {
    state = newstate;
}

double Testrun::update(int deltaMillis, uint8_t &enable) {
    time += deltaMillis;
    if(time > duration) {
        time = time - duration;
        if      (state == pwmZero)    state = linearRamp;
        else if (state == linearRamp) state = sinus;
        else if (state == sinus)      state = staticLow;
        else if (state == staticLow)  state = staticHigh;
        else if (state == staticHigh) state = disabled;
        else if (state == disabled)   state = testDone;
        else state = testDone;
    }
    switch (state)
    {
    case pwmZero:
        enable = 1;
        return 0.0;
        break;

    case linearRamp:
    {
        enable = 1;

        double ltime = time % (duration/4);
        double lduration = (double)duration/4.0;
        double lphase= (double)time / lduration;
        double lfact = ltime / lduration;

        if     (lphase < 1.0) return lfact * maxPWM;                // ramp up positive
        else if(lphase < 2.0) return (1.0 - lfact) * maxPWM;        // ramp down positive
        else if(lphase < 3.0) return lfact * -maxPWM;               // ramp down negative
        else if(lphase < 4.0) return (1.0 - lfact) * -maxPWM;       // ramp up negative
        else                  return 0;
        break;
    }

    case sinus:
    {
        enable = 1;
        double x = (double)time / (double) duration * 2.0 * M_PI;
        return sin(x) * maxPWM;
        break;
    }

    case staticLow:
    {
        enable = 1;

        double ltime = time % (duration/10);
        double lduration = (double)duration/10.0;
        double lphase= (double)time / lduration;
        double lfact = ltime / lduration;

        if     (lphase <  1.0) return lfact * maxPWM / 5.0;                // ramp up positive
        else if(lphase <  9.0) return maxPWM / 5.0;
        else if(lphase < 10.0) return (1.0 - lfact) * maxPWM / 5.0;        // ramp down positive
        else                  return 0;
        break;
    }

    case staticHigh:
    {
        enable = 1;

        double ltime = time % (duration/10);
        double lduration = (double)duration/10.0;
        double lphase= (double)time / lduration;
        double lfact = ltime / lduration;

        if     (lphase <  1.0) return lfact * maxPWM;                // ramp up positive
        else if(lphase <  9.0) return maxPWM;
        else if(lphase < 10.0) return (1.0 - lfact) * maxPWM;        // ramp down positive
        else                  return 0;
    }

    case disabled:
        enable = 0;
        return 0.0;
        break;


    default:
        enable = 0;
        return 0.0;
        break;

    }
}