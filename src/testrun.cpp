
#define M_PI 3.14159265358979323846
#include <math.h>
#include "testrun.h"

Testrun::State Testrun::getState() {
    return state;
}

void Testrun::setState(Testrun::State newstate) {
    state = newstate;
}

double Testrun::update(int deltaMillis, bool &enable) {
    time += deltaMillis;
    if(time > duration) {
        time = time - duration;
        if(state != testDone) state = State(state + 1);
    }
    switch (state)
    {
    case pwmZero:
        enable = true;
        return 0.0;
        break;

    case linearRamp:
    {
        enable = true;

        double ltime = time % (duration/4);
        double lduration = (double)duration/4.0;
        double lphase= (double)time / lduration;
        double lfact = ltime / lduration;

        if     (lphase <= 1.0) return lfact * maxPWM;                // ramp up positive
        else if(lphase <= 2.0) return (1.0 - lfact) * maxPWM;        // ramp down positive
        else if(lphase <= 3.0) return lfact * -maxPWM;               // ramp down negative
        else                   return (1.0 - lfact) * -maxPWM;       // ramp up negative
        break;
    }

    case sinus:
    {
        enable = true;
        double x = (double)time / (double) duration * 2.0 * M_PI;
        return sin(x) * maxPWM;
        break;
    }

    case staticLow:
    {
        enable = true;

        double ltime = time % (duration/10);
        double lduration = (double)duration/10.0;
        double lphase= (double)time / lduration;
        double lfact = ltime / lduration;

        if     (lphase <= 1.0) return lfact * maxPWM / 5.0;                // ramp up positive
        else if(lphase <= 9.0) return maxPWM / 5.0;
        else                   return (1.0 - lfact) * maxPWM / 5.0;        // ramp down positive
        break;
    }

    case staticHigh:
    {
        enable = true;

        double ltime = time % (duration/10);
        double lduration = (double)duration/10.0;
        double lphase= (double)time / lduration;
        double lfact = ltime / lduration;

        if     (lphase <= 1.0) return lfact * maxPWM;                // ramp up positive
        else if(lphase <= 9.0) return maxPWM;
        else                   return (1.0 - lfact) * maxPWM;        // ramp down positive
        break;
    }

    case disabled:
        enable = false;
        return 0.0;
        break;


    default:
        enable = false;
        return 0.0;
        break;

    }
}