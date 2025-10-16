// WRITTEN BY @Hermes2771 / rylyfo

#include <cmath>

struct tuple
{
    double coord1;
    double coord2;
};

double pEncoderValues[2];

double deltaRW = 0.0;
double deltaSW = 0.0;

tuple localCoordinates;
tuple globalCoordinates;

void ToLocalCoordinates(double deltaTheta, double offsetRW, double offsetSW, double deltaRW, double deltaSW)
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

void ToGlobalCoordinates(double averageOrientation, tuple localCoordinates)
{
    // we are rotating the vector by the negative average orientation
    globalCoordinates = {localCoordinates.coord1 * cos(averageOrientation) + localCoordinates.coord2 * sin(averageOrientation),
                            -localCoordinates.coord1 * sin(averageOrientation) + localCoordinates.coord2 * cos(averageOrientation)};

    
}

void Odometry(double offsetRW, double offsetSW, double previousHeading, double rwHeading, tuple previousPosition, double newEncoderValues[2])
{
    // get the changes for each encoder
    deltaRW = newEncoderValues[0] - pEncoderValues[0];
    deltaSW = newEncoderValues[1] - pEncoderValues[1];

    // update previous values
    pEncoderValues[0] = deltaRW;
    pEncoderValues[1] = deltaSW;

    // difference in rotation
    double deltaTheta = rwHeading - previousHeading;

    // use the rotation information to calculate the local position for the robot.
    ToLocalCoordinates(deltaTheta, offsetRW, offsetSW, deltaRW, deltaSW);

    // average rotation
    double averageOrientation = previousHeading + (deltaTheta / 2);

    ToGlobalCoordinates(averageOrientation, localCoordinates);

    tuple absolutePosition = {previousPosition.coord1 + globalCoordinates.coord1,
                            previousPosition.coord2 + globalCoordinates.coord2};
}