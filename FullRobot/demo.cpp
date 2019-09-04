#include <stdio.h>

#include "FlyCapture2.h"
#include <unistd.h>

#include<iostream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include "communications.h"
#include "commPackages.h"
#include "camera.h"


int main(int argc, char **argv) {
    if (argc != 3) {
        std::cerr << "\n" << "Usage: ./xxx path_to_vocabulary path_to_settings\n";
        return 1;
    }


    // includes all required settings for this application
    FlyCapture2::setup();

    // Start listening for messages from the robot
    Comms::setup();
    std::thread *receiveThreadPointer;
    receiveThreadPointer = new std::thread(&Comms::receiveInBackground);


    std::cout << "Request/Check robot is active... \n";
    while(!Comms::newStatusResponse || Comms::statusPackageResponse.status != Comms::Status::ACTIVE){
        Comms::sendStatusPackage(Comms::Status::ACTIVE);
        usleep(250000);
    }
    std::cout << "ACTIVE!\n";



    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, true);


    std::cout << endl << "-------\n";
    std::cout << "Start processing video ...\n";

    double tframe = 0.0; // Dummy timestamp

    // Main loop
//    cv::Mat im;
    FlyCapture2::Image rawImage;
    FlyCapture2::Image convertedImage;
    cv::Mat cvImage;
    FlyCapture2::Error error;

    int nImages = 500;
    for (int ni = 0; ni < nImages; ni++) {

        int res = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
//        std::cout << res << "\n";

        // Pass the image to the SLAM system
        static bool slamInitialised = false;
        SLAM.TrackMonocular(cvImage, tframe);
        if (!slamInitialised && SLAM.GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK) {
            slamInitialised = true;
//            std::cout << "SLAM initialised\n";
        }

        // Sweep the camera
        static bool sweepComplete = false;
        static int angle = 0;
        static int direction = 1;
        const int posMin = -20;
        const int posMax = 20;

        if(!sweepComplete) {
            if (direction == 1) {
                angle++;
                Comms::sendServoPackage((float) angle);
            }
            if (direction == -1) {
                angle--;
                Comms::sendServoPackage((float) angle);
            }
            if (angle >= posMax) {
                direction = -1;
            }
            if (angle <= posMin) {
                direction = 1;
            }
        }


        // check if ORB SLAM is initialised after a full sweep
        // if so, start moving (just move a bit forward to demonstrate)
        // if not, keep sweeping
        if (direction == 1 && angle == 0) {
            if (slamInitialised) {
                sweepComplete = true;
            }
        }

//        static bool moveComplete = false;
//        if (!moveComplete && sweepComplete) {
//            float turn = 0.5;
//            float forward = 50.0;
//            Comms::sendMovePackage(Comms::MoveType::TURN_AND_STRAIGHT, turn, forward);
//            Comms::sendPosePackage();
//            Comms::sendStatusPackage(Comms::Status::REPORT_ONLY);
//            moveComplete = true;
//        }


        tframe += 0.1;
        usleep(100000);
    }


    FlyCapture2::shutdown();


    // Stop listening for communications from the robot
    Comms::shutdown();


    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
