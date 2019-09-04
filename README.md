### Code dump
This should be updated to something a little more comprehensive in the next couple of days.

Need to run cmake/make in each of the subfolders except Microcontroller which contains Arduino/Teensyduino code files.  
FullRobot should be done last.  

Host code:  
* maps  
* path_planning  
* communication  
* FullRobot  

Microntroller code:  
* Microcontroller/Robot
* Microcontroller/IMU_calibration
* Microcontroller/Rotation


Host code dependencies:
* https://github.com/matt769/ORB_SLAM2_PRJ - clone this into same directory as this repository
* Eigen
* OpenCV
* Pangolin
* FlyCapture API (if installed in other than its default location then may need to modify FullRobot/CMakeLists.txt)


Microcontroller code dependencies (see report for links):
* PU9250
* PWMServo
* PID_v1
* Rotation (files provided, but need to be placed in stardard Arduino library location)

Other
* May need to modify include path in Robot.ino to point to communication/commPackages.h