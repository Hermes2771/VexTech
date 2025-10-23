// WRITTEN BY @Hermes2771 / rylyfo

#include <cmath>
#include "vex.h"
#include "pid.hpp"

using namespace vex;
using namespace PID;

#define errorMargin 0.1 // determines how close the robot has to be to a point for it to stop moving / reach its goal
#define points 0 // controls how many points are in your path

struct coordPair
{
    double coord1;
    double coord2;
};

// settings

coordPair currentTarget;
coordPair goalPoints[];

drivetrain myDriveTrain;

// ----------------------------- READ ----------------------------- //
//
// to be able to use this file, you must have a pre-made path / list of points.
// if you want my recommendation for doing this, create two base points (point 0 (robots pos), point 1 (target pos))
// between those points, using bezier splines to create a curved path between the points
// the more points you use, the more likely the pure pursuit is to stay on track but having very close points on a curve
// WILL cause the program to skip to the furthest points.
// ----------------------------- THANK YOU FOR READING ----------------------------- //

void InitializePursuit(coordPair newPoints[])
{
    // The initialize function is to ensure points are only created once, so the path isn't constantly re-filed.
    for (int i = 0; i < points; i++)
    {
        goalPoints[i] = newPoints[i];
    }
}

void StandardPursuit(coordPair absolutePos, double absoluteOrientation, double lookDistance)
{
    // find a point.
    // after finding a point, remove it from the list of viable points.

    for (int i = 0; i < points; i++)
    {
        double xDist = goalPoints[i].coord1 - absolutePos.coord1;
        double yDist = goalPoints[i].coord2 - absolutePos.coord2;

        double totalDistance = sqrt(pow(xDist, 2) + pow(yDist, 2));

        if (totalDistance < lookDistance) // everything will be automatically removed from the list once it enters this range,
        // and the errorMargin determines whether or not it'll actually stop at those points or not. 
        {
            coordPair zeroTuple = {INFINITY, INFINITY};

            currentTarget = goalPoints[i];
            goalPoints[i] = zeroTuple;
        }
    }

    MoveToPoint(absolutePos, absoluteOrientation);
}

void MoveToPoint(coordPair absolutePos, double absoluteOrientation)
{
    // get direction
    coordPair direction = {currentTarget.coord1 - absolutePos.coord1, currentTarget.coord2 - absolutePos.coord2};

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