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
    if(argc != 4)
    {
        std::cerr << "\n" << "Usage: ./xxx path_to_vocabulary path_to_settings use_viewer(y/n)\n";
        return 1;
    }

    bool useViewer = false;
    if(*argv[3]=='y')
    {
        useViewer = true;
    }


    // includes all required settings for this application
    FlyCapture2::setup();

    // Start listening for messages from the robot
    Comms::setup();
    std::thread* receiveThreadPointer;
    receiveThreadPointer = new std::thread(&Comms::receiveInBackground);


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,useViewer);
    double tframe = 0.0; // Dummy timestamp // TODO change to track actual time

    // activate robot (not strictly required if radio controlling)
//    std::cout << "Request/Check robot is active... \n";
//    while(!Comms::newStatusResponse || Comms::statusPackageResponse.status != Comms::Status::ACTIVE){
//        Comms::sendStatusPackage(Comms::Status::ACTIVE);
//        usleep(250000);
//    }
//    std::cout << "ACTIVE!\n";

    // check if robot actually on
    // send message and wait (with timeout) for response



    // Main loop
//    cv::Mat im;
    FlyCapture2::Image rawImage;
    cv::Mat cvImage;
    FlyCapture2::Error error;
    bool run = true;

    //int nImages = 500;
    int i = 0;
    while(run)
    {

        // TODO work out why this function leads to segmentation fault error in SLAM.TrackMonolular()
        //  (I assume some issue with the image)
//        int res = FlyCapture2::acquire(rawImage, cvImage);
//        std::cout << res << "\n";

        // TODO could this be more efficient?
        PollForTriggerReady(&FlyCapture2::cam);
//        std::cout << "Sending software trigger\n";
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


//        std::cout << "Grabbed image " << "\n";

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

        i++;
        tframe += 0.2; // just a guess
        usleep(100); // TODO change to control timing properly
    }


    FlyCapture2::shutdown();


    // Stop listening for communications from the robot
    Comms::shutdown();


    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
