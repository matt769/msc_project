#include "FlyCapture2.h"
#include <sstream>
#include <unistd.h>
#include <iostream>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>


namespace FlyCapture2 {

    Camera cam;
    Error error;

    void PrintBuildInfo() {
        FC2Version fc2Version;
        Utilities::GetLibraryVersion(&fc2Version);

        std::ostringstream version;
        version << "FlyCapture2 library version: " << fc2Version.major << "."
                << fc2Version.minor << "." << fc2Version.type << "."
                << fc2Version.build;
        std::cout << version.str() << "\n";

        std::ostringstream timeStamp;
        timeStamp << "Application build date: " << __DATE__ << " " << __TIME__;
        std::cout << timeStamp.str() << "\n\n";
    }

    void PrintCameraInfo(CameraInfo *pCamInfo) {
        std::cout << "\n";
        std::cout << "*** CAMERA INFORMATION ***" << "\n";
        std::cout << "Serial number - " << pCamInfo->serialNumber << "\n";
        std::cout << "Camera model - " << pCamInfo->modelName << "\n";
        std::cout << "Camera vendor - " << pCamInfo->vendorName << "\n";
        std::cout << "Sensor - " << pCamInfo->sensorInfo << "\n";
        std::cout << "Resolution - " << pCamInfo->sensorResolution << "\n";
        std::cout << "Firmware version - " << pCamInfo->firmwareVersion << "\n";
        std::cout << "Firmware build time - " << pCamInfo->firmwareBuildTime << "\n"
                  << "\n";
    }

    void PrintError(Error error) { error.PrintErrorTrace(); }


    bool CheckSoftwareTriggerPresence(Camera *pCam) {
        const unsigned int k_triggerInq = 0x530;

        Error error;
        unsigned int regVal = 0;

        error = pCam->ReadRegister(k_triggerInq, &regVal);

        if (error != PGRERROR_OK) {
            PrintError(error);
            return false;
        }

        if ((regVal & 0x10000) != 0x10000) {
            return false;
        }

        return true;
    }

    bool PollForTriggerReady(Camera *pCam) {
        const unsigned int k_softwareTrigger = 0x62C;
        Error error;
        unsigned int regVal = 0;

        do {
            error = pCam->ReadRegister(k_softwareTrigger, &regVal);
            if (error != PGRERROR_OK) {
                PrintError(error);
                return false;
            }

        } while ((regVal >> 31) != 0);

        return true;
    }

    bool FireSoftwareTrigger(Camera *pCam) {
        const unsigned int k_softwareTrigger = 0x62C;
        const unsigned int k_fireVal = 0x80000000;
        Error error;

        error = pCam->WriteRegister(k_softwareTrigger, k_fireVal);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return false;
        }

        return true;
    }


    int setup() {
        Error error;

        PrintBuildInfo();

        BusManager busMgr;
        unsigned int numCameras;
        error = busMgr.GetNumOfCameras(&numCameras);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        std::cout << "Number of cameras detected: " << numCameras << "\n";

        PGRGuid guid;
        error = busMgr.GetCameraFromIndex(0, &guid);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }


        // Connect to a camera
        error = cam.Connect(&guid);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        // Get the camera information
        CameraInfo camInfo;
        error = cam.GetCameraInfo(&camInfo);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        PrintCameraInfo(&camInfo);

        // Get the camera configuration
        FC2Config config;
        error = cam.GetConfiguration(&config);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        // Set the number of driver buffers used to 10.
        //config.numBuffers = 10;
        config.grabTimeout = 5000; // in the AsyncTriggerEx example this is done after setting the trigger mode - does it make a difference?

        // Set the camera configuration
        error = cam.SetConfiguration(&config);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }


        // Power on the camera
        const unsigned int k_cameraPower = 0x610;
        const unsigned int k_powerVal = 0x80000000;
        error = cam.WriteRegister(k_cameraPower, k_powerVal);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        const unsigned int millisecondsToSleep = 100;
        unsigned int regVal = 0;
        unsigned int retries = 10;

        // Wait for camera to complete power-up
        do {
            struct timespec nsDelay;
            nsDelay.tv_sec = 0;
            nsDelay.tv_nsec = (long) millisecondsToSleep * 1000000L;
            nanosleep(&nsDelay, NULL);

            error = cam.ReadRegister(k_cameraPower, &regVal);
            if (error == PGRERROR_TIMEOUT) {
                // ignore timeout errors, camera may not be responding to
                // register reads during power-up
            } else if (error != PGRERROR_OK) {
                PrintError(error);
                return -1;
            }

            retries--;
        } while ((regVal & k_powerVal) == 0 && retries > 0);

        // Check for timeout errors after retrying
        if (error == PGRERROR_TIMEOUT) {
            PrintError(error);
            return -1;
        }


        // Get current trigger settings
        TriggerMode triggerMode;
        error = cam.GetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        // Set camera to trigger mode 0
        triggerMode.onOff = true;
        triggerMode.mode = 0;
        triggerMode.parameter = 0;
        triggerMode.source = 7;         // A source of 7 means software trigger

        error = cam.SetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }


        // Set resolution (have to set frame rate too)
        //VideoMode videoMode = VIDEOMODE_640x480Y8;
        //FrameRate frameRate = FRAMERATE_15;
        error = cam.SetVideoModeAndFrameRate(VIDEOMODE_640x480Y8, FRAMERATE_30);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }


        // Start capturing images
        // It doesn't actually capture anything until the trigger is sent
        error = cam.StartCapture();
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        return 0;

    }


    int shutdown() {
        // Turn trigger mode off.
        TriggerMode triggerMode;
        error = cam.GetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        triggerMode.onOff = false;
        error = cam.SetTriggerMode(&triggerMode);
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }
        std::cout << "\n";
        std::cout << "Finished grabbing images\n";

        // Stop capturing images
        error = cam.StopCapture();
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        // Disconnect the camera
        error = cam.Disconnect();
        if (error != PGRERROR_OK) {
            PrintError(error);
            return -1;
        }

        return 0;
    }


    // Note that we have to pass convertedImage to the function as well because the cvMat will point to the data within it
    int acquire(Image& rawImage, Image& convertedImage, cv::Mat& cvImage){
        PollForTriggerReady(&cam);
//        std::cout << "Sending software trigger\n";
        // Fire software trigger
        bool retVal = FireSoftwareTrigger(&cam);
        if (!retVal)
        {
            std::cout << "\n";
            std::cout << "Error firing software trigger\n";
            return -1;
        }

        // Retrieve an image
        error = cam.RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);

            std::cout << "Error retrieving image\n";
            return -1;
        }

//        std::cout << "Grabbed image " << "\n";

        // Create a converted image
        //Image convertedImage; // this is not passed to the function

        // Convert the raw image
        error = rawImage.Convert(PIXEL_FORMAT_BGR, &convertedImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

        unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize()/(double)convertedImage.GetRows();
        cvImage = cv::Mat(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData(), rowBytes);

        return 0;

    }


}