### Code
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


### Supporting materials
See the Supporting_materials folder for:  
- Instructions to run robot *TO ADD*
- Odroid setup instructions
- Robot build notes *TO ADD*
- Robot schematic *TO ADD*
- Circuit diagram *TO ADD*
- 3D/2D design files


See this link https://www.dropbox.com/sh/bz3t7idnc23njo4/AABDjr6x9dCKdW70Yl40RJT0a?dl=0 for:  
- FlyCapture API files
- Component datasheets
- Point cloud maps *TO ADD*

See this link https://github.com/matt769/2017-10_Transmitter_v2 for:
- Transmitter code