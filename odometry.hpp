// WRITTEN BY @Hermes2771 / rylyfo

#include <cmath>

// --------------------- CONSTANTS --------------------- //

#define offsetRW 1 // controls the offset from the middle of the robot to the forward wheel
#define offsetSW 1 // controls the offset from the middle of the robot to the strafe wheel
#define wheelSize 3.25

// --------------------- STRUCTURES --------------------- //

struct coordPair
{
    double coord1;
    double coord2;
};

struct InfoPair
{
    coordPair absolutePos;
    double absoluteRot;
};

// --------------------- VARIABLES --------------------- //

double pEncoderValues[2]; // tracks the two previous encoder values
// make sure that it is [right wheel, strafe wheel]

double deltaRW = 0.0; 
double deltaSW = 0.0;

coordPair localCoordinates;
coordPair globalCoordinates;

// --------------------- FUNCTIOSN --------------------- //

void ToLocalCoordinates(double deltaTheta, double deltaRW, double deltaSW)
{
    if (deltaTheta == 0) // no rotations, robot just moved straight
    {
        localCoordinates = {offsetRW, offsetSW};
    }
    else // robot did rotate
    {
        localCoordinates = {2 * sin(deltaTheta / 2) * (deltaSW / deltaTheta) + offsetSW,
                       2 * sin(deltaTheta / 2) * (deltaRW / deltaTheta) + offsetRW};
    }
}

void ToGlobalCoordinates(double averageOrientation, coordPair localCoordinates)
{
    // we are rotating the vector by the negative average orientation
    // in other terms inversing the coordinate frame
    // (local coordinate frame -> global coordinate frame)
    globalCoordinates = {localCoordinates.coord1 * cos(averageOrientation) + localCoordinates.coord2 * sin(averageOrientation),
                            -localCoordinates.coord1 * sin(averageOrientation) + localCoordinates.coord2 * cos(averageOrientation)};
}

InfoPair GetOdometry(double previousHeading, double rwHeading, coordPair previousPosition, double newEncoderValues[2])
{
    // get the changes for each encoder
    deltaRW = ((newEncoderValues[0] - pEncoderValues[0]) / 360) * wheelSize; // get difference in encoders, convert to revolutions, then to wheel travel
    deltaSW = ((newEncoderValues[1] - pEncoderValues[1]) / 360) * wheelSize;

    // update previous values
    pEncoderValues[0] = deltaRW;
    pEncoderValues[1] = deltaSW;

    // difference in rotation
    double deltaTheta = rwHeading - previousHeading;

    // use the rotation information to calculate the local position for the robot.
    ToLocalCoordinates(deltaTheta, deltaRW, deltaSW);

    // average rotation
    double averageOrientation = previousHeading + (deltaTheta / 2);

    ToGlobalCoordinates(averageOrientation, localCoordinates);

    coordPair absolutePosition = {previousPosition.coord1 + globalCoordinates.coord1,
                            previousPosition.coord2 + globalCoordinates.coord2};

    InfoPair t = {absolutePosition, 0.0};

    return t;
}