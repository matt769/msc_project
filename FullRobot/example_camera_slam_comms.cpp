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








int main(int argc, char **argv)
{
    if(argc != 3)
    {
        std::cerr << "\n" << "Usage: ./xxx path_to_vocabulary path_to_settings\n";
        return 1;
    }


    // includes all required settings for this application
    FlyCapture2::setup();

    // Start listening for messages from the robot
    Comms::setup();
    std::thread* receiveThreadPointer;
    receiveThreadPointer = new std::thread(&Comms::receiveInBackground);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);


    std::cout << endl << "-------\n";
    std::cout << "Start processing video ...\n";

    double tframe = 0.0; // Dummy timestamp

    // Main loop
//    cv::Mat im;
    FlyCapture2::Image rawImage;
    cv::Mat cvImage;
    FlyCapture2::Error error;

    int nImages = 500;
    for(int ni=0; ni<nImages; ni++)
    {

        // TODO work out why this function leads to segmentation fault error in SLAM.TrackMonolular()
        //  (I assume some issue with the image)
//        int res = FlyCapture2::acquire(rawImage, cvImage);
//        std::cout << res << "\n";

        // TODO could this be more efficient?
        PollForTriggerReady(&FlyCapture2::cam);
        std::cout << "Sending software trigger\n";
        // Fire software trigger
        bool retVal = FireSoftwareTrigger(&FlyCapture2::cam);
        if (!retVal)
        {
            std::cout << "\n";
            std::cout << "Error firing software trigger\n";
            return -1;
        }

        // Retrieve an image
        error = FlyCapture2::cam.RetrieveBuffer(&rawImage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError(error);

            std::cout << "Error retrieving image\n";
            return -1;
        }


        std::cout << "Grabbed image " << "\n";

        // Create a converted image
        FlyCapture2::Image convertedImage;

        // Convert the raw image
        error = rawImage.Convert(FlyCapture2::PIXEL_FORMAT_BGR, &convertedImage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

        unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize()/(double)convertedImage.GetRows();
        cvImage = cv::Mat(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData(), rowBytes);



        // Pass the image to the SLAM system
        SLAM.TrackMonocular(cvImage,tframe);
        std::cout << "f\n";

        // Sweep the camera
        static int angle = 1;
        static int direction = 1;
        const int posMin = -10;
        const int posMax = 10;

        if(direction == 1){
            angle++;
            Comms::sendServoPackage((float)angle);
        }
        if(direction == -1){
            angle--;
            Comms::sendServoPackage((float)angle);
        }
        if(angle>=posMax){
            direction = -1;
        }
        if(angle<=posMin){
            direction = 1;
        }

        if(angle==0) {
            float turn = 0.0;
            float forward = 0.5;
            Comms::sendMovePackage(Comms::MoveType::TURN_AND_STRAIGHT, turn, forward);
            Comms::sendPosePackage();
            Comms::sendStatusPackage(Comms::Status::REPORT_ONLY);
        }



        // 'Sweep' the robot
//        static int angle = 1;
//        static int direction = 1;
//        const int posMin = -10;
//        const int posMax = 10;
//
//        if(direction == 1){
//            angle++;
//            Comms::sendMovePackage(Comms::MoveType::TURN_AND_STRAIGHT, 0.2, 0.0);
//        }
//        if(direction == -1){
//            angle--;
//            Comms::sendMovePackage(Comms::MoveType::TURN_AND_STRAIGHT, -0.2, 0.0);
//        }
//        if(angle>=posMax){
//            direction = -1;
//        }
//        if(angle<=posMin){
//            direction = 1;
//        }
//
//        if(angle==0) {
//            Comms::sendPosePackage();
//            Comms::sendStatusPackage();
//        }






        tframe += 0.1;
        usleep(100);
    }


    FlyCapture2::shutdown();


    // Stop listening for communications from the robot
    Comms::shutdown();


    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
