#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/core/core.hpp>
#include "FlyCapture2.h"

namespace FlyCapture2 {

    extern Camera cam;

    void PrintBuildInfo();
    void PrintCameraInfo(CameraInfo *pCamInfo);
    void PrintError(Error error);
    bool CheckSoftwareTriggerPresence(Camera *pCam);
    bool PollForTriggerReady(Camera *pCam);
    bool FireSoftwareTrigger(Camera *pCam);
    int setup();
    int shutdown();
    int acquire(Image& rawImage, Image& convertedImage, cv::Mat& cvImage);

}

#endif