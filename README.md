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
- Odroid setup instructions
- Robot build notes
- Robot development state notes *TO ADD*
- 3D/2D design files


See this link https://www.dropbox.com/sh/bz3t7idnc23njo4/AABDjr6x9dCKdW70Yl40RJT0a?dl=0 for:  
- FlyCapture API files
- Component datasheets
- Point cloud maps *TO ADD*

See this link https://github.com/matt769/2017-10_Transmitter_v2 for:
- Transmitter code


### To run
 More details on the connections are available in `build notes.pdf` if required. This is just a quick summary.

#### Using Odroid as the main computer
 * Connect Odroid and camera
 * Connect the switches and battery connectors
 * Turn them both on (Odroid circuit can be left off if just radio controlling the vehicle)
 * Connect to Odroid via ethernet cable
 * Open SSH session: IP 169.254.130.11, Username: odroid, Password: odroid
 * Run:
   * $ cd ~/ws/project/FullRobot
   * $ ./full ../../ORB_SLAM2_PRJ/Vocabulary/ORBvoc.txt PGchameleon_lowres.yaml n

#### Using an external computer
 * Connect external computer and camera
 * Connect the switch for the low level circuitry only (i.e. not the Odroid)
 * Turn it on
 * On the external computer, assuming it has the same folder structure as the Odroid (but easy to modify if not), run:
   * $ cd ~/ws/project/FullRobot
   * $ ./full ../../ORB_SLAM2_PRJ/Vocabulary/ORBvoc.txt PGchameleon_lowres.yaml y


#### Notes
The program `full` takes 3 parameters.
1. Path to Bag of Words Vocabulary file
2. Path to ORB SLAM config file (including camera intrinsic parameters) 
3. Visualisation indicator - `y` will launch the standard ORB SLAM windows to view the camera feed and map/localisation information, `n` will suppress then

In the above running instructions I have assumed visualisation disabled when using Odroid, and enabled when using an external computer.