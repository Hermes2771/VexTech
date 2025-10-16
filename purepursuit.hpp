// WRITTEN BY @Hermes2771 / rylyfo

#include <cmath>
#include "vex.h"
#include "pid.hpp"

using namespace vex;
using namespace PID;

struct tuple
{
    double coord1;
    double coord2;
};

// settings
double errorMargin = 0.1; // determines how close the robot has to be to a point for it to stop moving / reach its goal

tuple currentTarget;
tuple goalPoints[];

drivetrain myDriveTrain;

void InitializePursuit(tuple newPoints[10])
{
    // The initialize function is to ensure points are only created once, so the path isn't constantly re-filed.
    for (int i = 0; i < 10; i++)
    {
        goalPoints[i] = newPoints[i];
    }
}

void StandardPursuit(tuple absolutePos, double absoluteOrientation, double lookDistance)
{
    // find a point.
    // after finding a point, remove it from the list of viable points.

    for (int i = 0; i < 10; i++)
    {
        double xDist = goalPoints[i].coord1 - absolutePos.coord1;
        double yDist = goalPoints[i].coord2 - absolutePos.coord2;

        double totalDistance = sqrt(pow(xDist, 2) + pow(yDist, 2));

        if (totalDistance < lookDistance) // everything will be automatically removed from the list once it enters this range,
        // and the errorMargin determines whether or not it'll actually stop at those points or not. 
        {
            tuple zeroTuple = {INFINITY, INFINITY};

            currentTarget = goalPoints[i];
            goalPoints[i] = zeroTuple;
        }
    }

    MoveToPoint(absolutePos, absoluteOrientation);
}

void MoveToPoint(tuple absolutePos, double absoluteOrientation)
{
    // get direction
    tuple direction = {currentTarget.coord1 - absolutePos.coord1, currentTarget.coord2 - absolutePos.coord2};
    double tanDirection = atan2(direction.coord2, direction.coord1);
    double totalDistance = sqrt(pow(currentTarget.coord1 - absolutePos.coord1, 2) + pow(currentTarget.coord2 - absolutePos.coord1, 2));

    double neededTurn = 0;

    // ex: curr: 90, targ: 5 = (90 - (90 + 5))
    // ex: curr: 5, targ: 90 = (5 + (90 - 5))

    if (absoluteOrientation > tanDirection)
    {
        neededTurn = absoluteOrientation - (absoluteOrientation + tanDirection);
    }
    else
    {
        neededTurn = absoluteOrientation + (absoluteOrientation - tanDirection);
    }


    if (totalDistance > errorMargin)
    {
        //  change based off orientation
        double PIDx = Update(absolutePos.coord1, currentTarget.coord1);
        double PIDy = Update(absolutePos.coord2, currentTarget.coord2);

        // changes depending on which axis has more influence
        double movePower = PIDx * cos(absoluteOrientation) + PIDy * sin(absoluteOrientation);

        myDriveTrain.setDriveVelocity(movePower, rpm);
        myDriveTrain.setTurnVelocity(Update(absoluteOrientation, tanDirection), rpm);

        myDriveTrain.turnFor(neededTurn, degrees, false);
        myDriveTrain.driveFor(totalDistance, inches);
    }   
}