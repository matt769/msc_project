#include <iostream>
#include<chrono>
#include <ctime>
#include <thread>





#include"communications.h"
//#include "commPackages.h"

int main(int argc, char **argv) {



    auto timeStart = std::chrono::system_clock::now();
    std::chrono::duration<double> elapsed = timeStart - timeStart;

    Comms::setup();
    std::thread* receiveThreadPointer;
    receiveThreadPointer = new std::thread(&Comms::receiveInBackground);


    while (elapsed.count() < 5) // THIS IS THE STAND IN FOR WHATEVER MAIN WHILE LOOP IS RUNNING
    {

//        Comms::receive();
        // TODO if we've received new data, do something
        // TODO add checks/locks on variable access


        // send some test commands (just once, and separated by some time)
        static bool test1Sent(false);
        if((!test1Sent) && (elapsed.count() > 1)) {

            float turn = 1.0;
            float forward = 1.0;
            Comms::sendMovePackage(Comms::MoveType::ABS_TURN_AND_STRAIGHT, turn, forward);

            test1Sent = true;
        }

        static bool test2Sent(false);
        if((!test2Sent) && (elapsed.count() > 2)) {

            Comms::sendPosePackage();
            test2Sent = true;
        }

        static bool test3Sent(false);
        if((!test3Sent) && (elapsed.count() > 3)) {

            float angle = 10;
            Comms::sendServoPackage(angle);
            test3Sent = true;
        }

        static bool test4Sent(false);
        if((!test4Sent) && (elapsed.count() > 4)) {

            Comms::sendStatusPackage(Comms::Status::REPORT_ONLY);
            test4Sent = true;
        }



        auto timeNow = std::chrono::system_clock::now();
        elapsed = timeNow - timeStart;
    }


    Comms::shutdown();
}


