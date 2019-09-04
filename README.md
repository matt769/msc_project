Code dump

Need to run cmake/make in each of the subfolders except Microcontroller which contains Arduino/Teensyduino code files.
FullRobot should be done last

Host code:
	maps
	path_planning
	communication
	FullRobot

Microntroller code:
	Microcontroller/Robot
	Microcontroller/IMU_calibration


Dependencies:
https://github.com/matt769/ORB_SLAM2_PRJ - clone this into same directory as this repository
Eigen
OpenCV
Pangolin
FlyCapture API (if installed in other than its default location then may need to modify CMakeLists.txt)
