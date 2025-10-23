// WRITTEN BY @Hermes2771 / rylyfo with major assistance from the following article:
// https://xiaoxiae.github.io/Robotics-Simplified-Website/motor-controllers/pid/

#include <cmath>
#include "vex.h"

using namespace vex;
timer myTimer;

// --------------------- PID CONSTANTS --------------------- // 
// MUST BE DIRECTLY MODIFIED BY USER

#define p 1.0 // Controls the impact of P
#define i 1.0 // Controls the impact of I
#define d 1.0 // Controls the impact of D

// --------------------- CORE FUNCTION / NAMESPACES --------------------- //

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

    double P; // how large the error is now (present)
    double I; // how large the error was before (past)
    double D; // how large the error will be (future)

    // PID constants are at the top of the header file, change at your discretion

    void Reset()
    {
        error = 0;
        deltaError = 0;
        previousTime = 0;
        currentTime = 0;

        P, I, D = 0;
    }

    void Initialize(double kp, double ki, double kd)
    {
        currentTime = myTimer.time();
        previousError = 0;
    }

    double Update(double currentPosition, double targetPosition)
    {
        // calculate difference of positions
        error = currentPosition - targetPosition;
        currentTime = myTimer.time();

        // calculate deltas (changes / differences)
        deltaTime = currentTime - previousTime;
        deltaError = error - previousError;
        
        P = p * error;
        I += error * deltaTime * i;
        
        // update previous values
        previousTime = currentTime;
        previousError = error;

        // make sure deltaTime is greater than zero otherwise we divide by 0
        if (deltaTime > 0) D = deltaError / deltaTime * d;
    
        return P + I + D;
    }
}