#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

//#include <termios.h>
#include "commPackages.h"
// What is actually required to be exposed here?
// Just the packages?

// Should I put these externs in the main file instead?


namespace Comms {

    extern MovePackageResponse movePackageResponse;
    extern PosePackageResponse posePackageResponse;
    extern ServoPackageResponse servoPackageResponse;
    extern StatusPackageResponse statusPackageResponse;

    extern bool newMoveResponse;
    extern bool newPoseResponse;
    extern bool newStatusResponse;
    extern bool newServoResponse;

    extern bool receivingActive;

    void setup(void);
    void receive(void);
    void receiveInBackground(void);
    void shutdown(void);
    void sendMovePackage(MoveType moveType, float turn, float forward);
    void sendPosePackage(void);
    void sendServoPackage(float angle);
    void sendStatusPackage(Status requestedStatus);



}


#endif