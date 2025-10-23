// WRITTEN BY @Hermes2771 / rylyfo

#include <cmath>
#include "vex.h"

using namespace vex;

controller myController = controller();
drivetrain myDriveTrain;

// --------------------- CONSTANTS --------------------- //

#define driveVelocity 100 // controls overall speed of the robot
#define turnVelocity 100 // controls turn speed of the robot

// --------------------- FUNCTIONS --------------------- //

void ExponentialDrive(drivetrain myDriveTrain)
{
    // put trackers on the controller to check for any changes (driver moves right stick up, etc)
    myController.Axis3.changed(OnChangeMove);
    myController.Axis1.changed(OnChangeRotate);
}

void OnChangeMove()
{
    double yAxis = myController.Axis3.position(); // up and down of the left joystick

    // create a zero vector which we add movements to
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