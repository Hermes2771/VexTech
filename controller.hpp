// WRITTEN BY @Hermes2771 / rylyfo

#include <cmath>
#include "vex.h"

using namespace vex;

controller myController = controller();
drivetrain myDriveTrain;

double driveVelocity = 0;
double turnVelocity = 0;

void ExponentialDrive(drivetrain myDriveTrain, double baseVelocity, double driveVelocity, double turnVelocity)
{
    myController.Axis3.changed(OnChangeMove);
    myController.Axis1.changed(OnChangeRotate);

    ::driveVelocity = driveVelocity;
    ::myDriveTrain = myDriveTrain;
    ::turnVelocity = turnVelocity;
}

void OnChangeMove()
{
    double yAxis = myController.Axis3.position(); // up and down of the left joystick

    double finalMS = 0.0;

    if (yAxis > 0)
    {
        finalMS = driveVelocity * pow((yAxis / 10.0), 2);
    }
    else
    {
        finalMS = -1 * (driveVelocity * pow((yAxis / 10.0), 2));
    }

    myDriveTrain.setDriveVelocity(finalMS, rpm);
    myDriveTrain.drive(forward);
}

void OnChangeRotate()
{
    double xAxis = myController.Axis1.position(); // left & right of the right joystick
    
    myDriveTrain.setTurnVelocity(turnVelocity, rpm);

    if (xAxis > 0)
    {
        myDriveTrain.turn(right);
    }
    else
    {
        myDriveTrain.turn(left);
    }
}