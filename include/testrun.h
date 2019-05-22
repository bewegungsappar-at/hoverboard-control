#pragma once

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

        double update(int deltaMillis, bool &enable);
        State getState();
        void setState(State newstate);

    private:
        State state=testDone;
        long duration = 10000; // Duration per state
        long time = 0;
        double maxPWM = 1000.0;

};
