// WRITTEN BY @Hermes2771 / rylyfo with major assistance from the following article:
// https://xiaoxiae.github.io/Robotics-Simplified-Website/motor-controllers/pid/

#include <cmath>
#include "vex.h"

using namespace vex;
timer myTimer;


namespace PID
{

    // TIMER HAS TO BE ADDED IN LATER WITH VEX DEFINITIONS


    // errors / differences
    double error = 0; // dist between robot and its goal
    double previousError = 0;
    double deltaError = 0; // current and previous error

    // used for checking time since last measurement
    double previousTime = 0;
    double currentTime = 0;
    double deltaTime = 0;

    // PIDS

    double P;
    double I;
    double D;

    // PID constants / set by programmer

    double p; // how large the error is now (present)
    double i; // how large the error was before
    double d; // how large it'll be in the future


    void Reset()
    {
        error = 0;
        deltaError = 0;
        previousTime = 0;
        currentTime = 0;

        P, p, I, i, D, d = 0;
    }

    void Initialize(double kp, double ki, double kd)
    {
        p = kp;
        i = ki;
        d = kd;
        
        currentTime = myTimer.time();
        previousError = 0;
    }

    double Update(double currentPosition, double targetPosition)
    {
        error = currentPosition - targetPosition;
        currentTime = myTimer.time();
        deltaTime = currentTime - previousTime;
        deltaError = error - previousError;
        
        P = p * error;
        I += error * deltaTime * i;

        previousTime = currentTime;
        previousError = error;

        if (deltaTime > 0)
        {
            D = deltaError / deltaTime * d;
        }

        return P + I + D;
    }
}