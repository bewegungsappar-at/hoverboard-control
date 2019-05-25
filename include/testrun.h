#pragma once
#include <stdint.h>

class Testrun {
    public:
        enum State {
            pwmZero,
            linearRamp,
            sinus,
            staticLow,
            staticHigh,
            disabled,
            testDone,
        };

        double update(int deltaMillis, uint8_t &enable);
        State getState();
        void setState(State newstate);


        State state=testDone;
        long duration = 10000; // Duration per state
        long time = 0;
        double maxPWM = 1000.0;
    private:
};
