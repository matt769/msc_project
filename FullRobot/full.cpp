#include <stdio.h>

#include "FlyCapture2.h"
#include <unistd.h>

#include<iostream>
#include<chrono>

#include<opencv2/core/core.hpp>
#include <Eigen/Dense>


#include<System.h>
#include<Converter.h>
#include <atomic>

#include "communications.h"
#include "commPackages.h"
#include "camera.h"
#include "GridMap.h"
#include "astar.h"
#include "transformations.h"


// Declare supporting functions (definitions are at the bottom of the file)
bool setOrCheckRobotStatus(const Comms::Status &newOrExpectedStatus, const std::chrono::seconds &timeoutPeriod);
bool setRobotStatus(const Comms::Status &newStatus, const std::chrono::seconds &timeoutPeriod);
bool checkRobotStatus(const Comms::Status &expectedStatus, const std::chrono::seconds &timeoutPeriod);
bool servoSweep();
bool servoMove();
bool mapExtractionAndAnalysis(ORB_SLAM2::System& SLAM, const Eigen::RowVector3f robotPos,
                              const float scale, const float stepSize, const float mapLimit,
                              Eigen::MatrixXf& mapPoints, GridMap& map);
Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4);
void calculateRequiredMovement(const Eigen::Matrix<float, 4, 4>& robotPose,
                          const Eigen::Vector2f& targetPos,
                          Eigen::Vector2f& requiredMovements);
void outputMap(std::string fileName, const Eigen::MatrixXf& mapPcd);



int main(int argc, char **argv) {
    // ***************************************************************
    // ********************* SETUP ***********************************
    // ***************************************************************
    std::cout << "Starting...\n";

    // Check input args ok
    if (argc != 4) {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings use_viewer(y/n)" << endl;
        return 1;
    }

    bool useViewer = true;
    if (*argv[3] == 'n') {
        useViewer = false;
    }


    // Parameters
    constexpr float stepSize = 100.0; // 10mm, the footprint of the robot is made up of roughly 25 squares of this size
    constexpr float mapLimit = 1000.0; // 1m





    // Utility variables
    bool resBool; // for checking return values
    int resInt; // for checking return values

    // Robot state variables
//    Comms::Status robotState = Comms::Status::UNINITIALISED; // reflects state of microcontroller, not this program
    int servoAngleDeg = 0; // TODO this isn't used everywhere yet e.g. servo sweep function
                            //   should be master state of servo (always up to date)



    // Timer variables, we'll use this to control anything that requires timing e.g. servo sweep, map analysis
    auto examplePeriod = std::chrono::milliseconds(100);
    std::chrono::time_point<std::chrono::steady_clock> tStart;
    std::chrono::time_point<std::chrono::steady_clock> tEnd;
    std::chrono::time_point<std::chrono::steady_clock> tNow;
//    bool response = false;
//    bool timeout = false;
    auto timeoutPeriod = std::chrono::seconds(3);
    bool ok;

    // Start listening for messages from the robot
    std::cout << "Set up receiver\n";
    Comms::setup();
    // TODO consider changing to
    //  std::thread receiveThread(&Comms::receiveInBackground);
    //  because I'm not sure I need the pointer
//    std::thread *receiveThreadPointer;
//    receiveThreadPointer = new std::thread(&Comms::receiveInBackground);
    std::thread commThread(&Comms::receiveInBackground);
    usleep(1000);

    // Check robot status is READY
    std::cout << "Check robot status\n";
    ok = checkRobotStatus(Comms::Status::READY, std::chrono::seconds(10));
    if (!ok) {
        std::cout << "Exiting program\n";
        return 1;
    }

    std::cout << "Setting robot status to active\n";
    ok = setRobotStatus(Comms::Status::ACTIVE, std::chrono::seconds(3));
    if (!ok) {
        std::cout << "Exiting program\n";
        return 1;
    }

    // initialise camera
    // includes all required settings for this application
    // TODO check return value and exit if required
    FlyCapture2::setup();


    // initialise SLAM
    ORB_SLAM2::System SLAM(argv[1], argv[2], ORB_SLAM2::System::MONOCULAR, useViewer);
    ORB_SLAM2::Tracking* trk = SLAM.getTracking();



    // necessary variables for image capture flow
    FlyCapture2::Image rawImage;
    FlyCapture2::Image convertedImage;
    cv::Mat cvImage;
    FlyCapture2::Error error;

    // PLACEHOLDER - acquire image
    resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
    if (resInt > 0) {
        std::cout << "Error in image capture - need to handle better probably\n";
        std::cout << "Exiting program\n";
        return 1;
    }
    // run SLAM tracking
    // What format is this timestamp supposed to be??
    // I think it's seconds (based on the example sequences)
    std::chrono::duration<float> tFrame;
    tStart = std::chrono::steady_clock::now();
    tFrame = tStart - tStart;
    SLAM.TrackMonocular(cvImage, tFrame.count());




    // ***************************************************************
    // ************* TRY TO INITIALISE ORBSLAM MAP *******************
    // ***************************************************************

    // is there really much advantage to having wrapped stuff ni servoSweep() ?

    const int maxSweeps = 3;
    for (int sweepNo = 0; sweepNo < maxSweeps; sweepNo++) {

        // servoSweep will return false until it completes
        while (!servoSweep()) {

            // acquire image
            resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
            if (resInt > 0) {
                std::cout << "Error in image capture - need to handle better probably\n";
                std::cout << "Exiting program\n";
                return 1;
            }

            // PLACEHOLDER - give SLAM the current estimated pose TO BE CONFIRMED

            // run SLAM tracking
            tNow = std::chrono::steady_clock::now();
            tFrame = tNow - tStart;
            SLAM.TrackMonocular(cvImage, tFrame.count());

        }

        std::cout << "Sweep " << sweepNo + 1 << "\n";
        if (SLAM.GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK) {
            std::cout << "SLAM map initialised, ending sweep phase\n";
            break;
        }
    }


    // check SLAM state and if not initialised, end program
    if (SLAM.GetTrackingState() != ORB_SLAM2::Tracking::eTrackingState::OK) {
        std::cout << "ORB SLAM not initialised\n";
        std::cout << "Exiting program\n";
    }


    // ***************************************************************
    // ************************ GET SCALE ****************************
    // ***************************************************************

    // TODO should check after each step if tracking lost and go back to last known good point if so
    //  and stay there for a few cycles

    float scale = 1.0;
    bool haveScale = false;
    int getScaleAttempt = 0;
    const int getScaleAttemptMax = 3;
    bool reset = false;

    while(!haveScale && (getScaleAttempt < getScaleAttemptMax)) {
        reset = false;
        getScaleAttempt++;
        std::cout << "Get scale (attempt " << getScaleAttempt << " of " << getScaleAttemptMax << ")\n";

        // need to regain tracking if we lost it
        int tries = 0;
        while (SLAM.GetTrackingState() != ORB_SLAM2::Tracking::eTrackingState::OK) {
            std::cout << "Trying to regain tracking\n";
            Comms::sendServoPackage(0.0);
            // acquire image
            resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
            if (resInt > 0) {
                std::cout << "Error in image capture - need to handle better probably\n";
                std::cout << "Exiting program\n";
                return 1;
            }
            // run SLAM tracking
            tNow = std::chrono::steady_clock::now();
            tFrame = tNow - tStart;
            SLAM.TrackMonocular(cvImage, tFrame.count());

            tries++;
            if(tries>20) {
                std::cout << "Could not regain tracking\n";
                std::cout << "Exiting program\n";
                exit(1);
            }
        }




        for (int angle = 0; angle <= 32; angle += 2) {

            Comms::sendServoPackage((float) angle);
            usleep(100000); // 0.2 seconds

            // acquire image
            resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
            if (resInt > 0) {
                std::cout << "Error in image capture - need to handle better probably\n";
                std::cout << "Exiting program\n";
                return 1;
            }
            // run SLAM tracking
            tNow = std::chrono::steady_clock::now();
            tFrame = tNow - tStart;
            SLAM.TrackMonocular(cvImage, tFrame.count());

            if (SLAM.GetTrackingState() != ORB_SLAM2::Tracking::eTrackingState::OK) {
                std::cout << "Tracking state " << SLAM.GetTrackingState() << "\n";
                std::cout << "Tracking lost, resetting.\n"; // TODO it isn't really resetting
                reset = true;
                break;
            }

        }

        if(reset) continue;

        Eigen::Vector3d camPos1;
        cv::Mat latestPose1 = (trk->mCurrentFrame).mTcw;
        camPos1 = ORB_SLAM2::Converter::toVector3d(latestPose1.rowRange(0, 3).col(3));
        Eigen::Matrix<float,4,4> T_c1_world;
        T_c1_world = toMatrix4f(latestPose1);


        for (int angle = 32; angle >= -35; angle -= 2) {

            Comms::sendServoPackage((float) angle);
            usleep(100000); // 0.1 seconds

            // acquire image
            resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
            if (resInt > 0) {
                std::cout << "Error in image capture - need to handle better probably\n";
                std::cout << "Exiting program\n";
                return 1;
            }
            // run SLAM tracking
            tNow = std::chrono::steady_clock::now();
            tFrame = tNow - tStart;
            SLAM.TrackMonocular(cvImage, tFrame.count());

            if (SLAM.GetTrackingState() != ORB_SLAM2::Tracking::eTrackingState::OK) {
                std::cout << "Tracking state " << SLAM.GetTrackingState() << "\n";
                std::cout << "Tracking lost, resetting.\n"; // TODO it isn't really resetting
                reset = true;
                break;
            }
        }

        if(reset) continue;

        Eigen::Vector3d camPos2;
        cv::Mat latestPose2 = (trk->mCurrentFrame).mTcw;
        camPos2 = ORB_SLAM2::Converter::toVector3d(latestPose2.rowRange(0, 3).col(3));
        Eigen::Matrix<float,4,4> T_c2_world;
        T_c2_world = toMatrix4f(latestPose2);

        for (int angle = -35; angle <= 0; angle += 2) {

            if (SLAM.GetTrackingState() != ORB_SLAM2::Tracking::eTrackingState::OK) {
                reset = true;
                break;
            }

            Comms::sendServoPackage((float) angle);
            usleep(100000); // 0.1 seconds

            // acquire image
            resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
            if (resInt > 0) {
                std::cout << "Error in image capture - need to handle better probably\n";
                std::cout << "Exiting program\n";
                return 1;
            }
            // run SLAM tracking
            tNow = std::chrono::steady_clock::now();
            tFrame = tNow - tStart;
            SLAM.TrackMonocular(cvImage, tFrame.count());
        }

        if(reset) continue;

        Eigen::Matrix<float,4,4> T_world_c2;
        Transformations::inverseTransformation(T_c2_world, T_world_c2);
        Eigen::Matrix<float,4,4> T_c1_c2 = T_c1_world * T_world_c2;

        const float camDistanceActual = 220; // TODO move this somewhere else
//        float camDistanceOrbSlam = (camPos1 - camPos2).norm(); // So wrong
        float camDistanceOrbSlam = T_c1_c2.block(0,3,3,1).norm();

        scale = camDistanceActual / camDistanceOrbSlam;
        std::cout << "Scale calculation\n";
        std::cout << "camDistanceOrbSlam: " << camDistanceOrbSlam << "\n";
        std::cout << "scale: " << scale << "\n";

        // Return camera to middle (it should already be there)
        Comms::sendServoPackage(0);
        usleep(100000); // 0.1 seconds

        haveScale = true; // break

    }

    if (SLAM.GetTrackingState() != ORB_SLAM2::Tracking::eTrackingState::OK) {
        std::cout << "Tracking lost\n";
        std::cout << "Exiting\n";
        exit(1);         // TODO make thread safe somehow?
    }





        // ***************************************************************
    // *************** INITIALISE TRANSFORMATIONS ********************
    // ***************************************************************

    // Run SLAM again with servo centred
    // Get picture
    resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
    if (resInt > 0) {
        std::cout << "Error in image capture - need to handle better probably\n";
        std::cout << "Exiting program\n";
        return 1;
    }

    // run SLAM tracking
    tNow = std::chrono::steady_clock::now();
    tFrame = tNow - tStart;
    SLAM.TrackMonocular(cvImage, tFrame.count());

    // Get pose
    Eigen::Matrix<float, 4, 4> camPose;
    if (SLAM.GetTrackingState() == ORB_SLAM2::Tracking::eTrackingState::OK) {
        cv::Mat latestPose2 = (trk->mCurrentFrame).mTcw;
        camPose = toMatrix4f(latestPose2);
    }
    else {
        std::cout << "Tracking lost\n";
        // TODO some action (exit?)
    }

    Transformations::initialiseTransformations(scale, (float)servoAngleDeg, camPose);
    //std::cout << "T_4_5:\n" << Transformations::T_4_5 << "\n";





    // ***************************************************************
    // ******************** FIRST MAP ANALYSIS ***********************
    // ***************************************************************

    // Move to main loop only
    // TODO move to separate function
    // TODO ? Run in separate thread - so need to check that previous run has finished

    Eigen::MatrixXf mapPoints; // map TODO move this earlier (initial map analysis)
    GridMap map;
    std::vector<int> path; // holds the result of path planning

    std::atomic<bool> mapAnalysisComplete(false); // for when run in separate thread
    std::atomic<bool> mapAnalysisRunning(false); // for when run in separate thread
    constexpr int mapAnalysisPeriodN = 100;
    int nLastMapAnalysis = 0;
    std::atomic<bool> newMapAvailable(false);

    // get robot position as well, need to make sure that grid includes it
    Eigen::RowVector3f robotPosInit;
    Transformations::getRobotPositionInWorld(robotPosInit);

    // transform the map points as well


    // note that map points and camera pose are still in orb slam coord frame
    //  they will get converted within the mapExtractionAndAnalysis function
    resBool = mapExtractionAndAnalysis(SLAM, robotPosInit, scale, stepSize, mapLimit, mapPoints, map); // TODO check return value
    if(resBool){
        newMapAvailable = true;
    }

    // Output initial map for scale checking
    // TODO remove when done
    string outputMapFileName = "output/map.txt";
    ofstream fileMap(outputMapFileName);
    if (fileMap.is_open() && mapPoints.size() > 0) {
        std::cout << "Map with " << mapPoints.size() << " extracted\n";
        fileMap << mapPoints;
    }
    fileMap.close();




    // ************************ PATH PLANNING ****************************
    // TODO move to main loop only
    // TODO consider if any other criteria for running?
    if(newMapAvailable) {

//        if(map.numCells > 0) {
//            int startIdx = 0;               // PLACEHOLDER
//            int goalIdx = map.numCells-1;     // PLACEHOLDER
//            resBool = PathPlanning::a_star(map, startIdx, goalIdx, path);
//            if (resBool) {
//                std::cout << "Path planning successful\n";
//            } else {
//                std::cout << "Path planning failed\n";
//            }
//        }
//        else {
//            std::cout << "No viable map?\n";
//        }

        newMapAvailable = false;
    }

//    auto mapAnalysisPeriod = std::chrono::seconds(20);
//    auto tLastMapAnalysis = std::chrono::steady_clock::now();






    // ***************************************************************
    // ************************ MAIN LOOP ****************************
    // ***************************************************************

    bool robotMoving = false;
    int pathStepIdx = 0;
    Eigen::RowVector3f robotPos;
    Eigen::Matrix<float, 4, 4> robotPose;
    Eigen::Matrix<float, 4, 4> T_cam_orb;
    //Eigen::Matrix<float, 4, 4> camPose; // already def
    bool pathPlanAvailable = false;
//    bool tryToRegainTracking = false;
    int robotCellIdx = -1;
    int goalCellIdx = -1;
    Eigen::Vector2f persistentGoal;
    bool goalExists = false;
    bool reachedGoal = false;
    bool newPathPlanAvailable = false;
    Eigen::Matrix2f robotXY, goalXY;
//    bool movementFinished = true;
    bool firstTimePathPlanRun = true;
//    int pathFromCellIdx = -1;   // keeps temporary record of the current movement
    int pathToCellIdx = -1;     // keeps temporary record of the current movement
    bool newMapRequest = false; // flag to indicate map/path wants to run but is delayed by current movement
    bool robotInMap = true; // flag to indicate whether the robot has left the known map (in which case need to re-map)
    std::chrono::time_point<std::chrono::steady_clock> tLastFpsReport = std::chrono::steady_clock::now();
    std::chrono::duration<long> tFpsPeriod = std::chrono::seconds(5);

    ORB_SLAM2::Tracking::eTrackingState slamTrackingState;
    int nIter = 0;
    bool keepRunning = true;
    while (keepRunning) {

        // check some goal/finish criteria
        //  set keepRunning = false if met

//        if (nIter >= 100) { // TODO don't forget to remove this!
//            keepRunning = false; // maybe this is redundant?
//            break;
//        }

        // ************************ ACQUIRE IMAGE ****************************
        resInt = FlyCapture2::acquire(rawImage, convertedImage, cvImage);
        if (resInt > 0) {
            std::cout << "Error in image capture - need to handle better probably\n";
            std::cout << "Exiting program\n";
            return 1;
        }

        // TODO PLACEHOLDER - give SLAM the current estimated pose TO BE CONFIRMED

        // ************************ SLAM tracking ****************************
        tNow = std::chrono::steady_clock::now();
        tFrame = tNow - tStart;
        SLAM.TrackMonocular(cvImage, tFrame.count());
        // check tracking status and number of feature matches
        slamTrackingState = static_cast<ORB_SLAM2::Tracking::eTrackingState>(SLAM.GetTrackingState());
        std::cout << "Tracking state " << static_cast<int>(slamTrackingState) << "\n";
        // TODO get number of matches (may need to modify orbslam lib)


        // TODO handle loss of tracking
        if(slamTrackingState != ORB_SLAM2::Tracking::eTrackingState::OK) {
//            tryToRegainTracking = true;
            // TODO do something
        }
        else {

            // Update all transformations
//            std::cout << "Updating transformations\n";
            cv::Mat camPoseCV = (trk->mCurrentFrame).mTcw;
            T_cam_orb = toMatrix4f(camPoseCV);
//            std::cout << "camPose: " << camPose << "\n";
            Transformations::updateTransformations(servoAngleDeg, scale, T_cam_orb);
            Transformations::getRobotPoseInWorld(robotPose);
            Transformations::getRobotPositionInWorld(robotPos);
//            std::cout << "robotPose: " << robotPose << "\n";
//            std::cout << "robotPos: " << robotPos << "\n";


            // ************************ SCALE ****************************

            // Periodically recalculate scale







            // ************************ MOVEMENT ****************************

//            std::cout << "Handle movement\n";
//            static int movementFailureCnt = 0; // tracks how often the robot fails to get to the expected cell


            // Check latest robot position in map
//            std::cout << "Check robot position\n";
            robotCellIdx = map.getIndexUnsafe(robotPos(0), robotPos(1));
            std::cout << "Robot in cell " << robotCellIdx << "\n";
            robotInMap = (robotCellIdx >= 0);

            // Check if robot at goal
//            std::cout << "Check goal\n";
            if (goalExists && !reachedGoal) {
//                robotCellIdx = map.getIndex(robotPos(0), robotPos(1));
                if (robotCellIdx == goalCellIdx) {
                    reachedGoal = true;
                    std::cout << "Reached goal!\n";
                }
            }

            // Check latest movement package from robot (if any) first
            // TODO make thread safe, maybe make accessor function
//            std::cout << "Check robot movement status\n";
            if (Comms::newMoveResponse) {
                if (Comms::movePackageResponse.moveStatus == Comms::MoveStatus::FINISHED) {
                    robotMoving = false;
                } else {
                    robotMoving = true;
                }
            }

            // if most recent movement has finished and there's more in the list, pick the next movement from the list
            //  and send movement command
            // (don't run if map/pathplanning wants to run again)
            if (pathPlanAvailable && !robotMoving && !reachedGoal && robotInMap && !newMapRequest) {
                std::cout << "Get next movement\n";
                Eigen::Vector2f targetPos;
                Eigen::Vector2f requiredMovements; // turn and a move (forward)

                // if there's a new path plan available, then start from the beginning
                if (newPathPlanAvailable) {
                    std::cout << "New path plan\n";
                    pathStepIdx = 0;
                    newPathPlanAvailable = false;
                }

                // check if robot is in the expected cell
                // either it is in the starting cell, or it has reached an intermediate goal cell
                if (robotCellIdx == path[pathStepIdx]) {
                    std::cout << "Robot in expected cell, incrementing goal\n";
                    pathStepIdx++;
                }

                // set the new goal (unless we've reached the end of the path)
                if ((size_t) pathStepIdx < path.size()) {
                    std::cout << "New target cell: " << path[pathStepIdx] << "\n";
                    pathToCellIdx = path[pathStepIdx];
                } else {
                    std::cout << "End of current path.\n";
                    pathToCellIdx = -1;
                }

                // now if there's a (intermediate) goal set, move to it
                if (pathToCellIdx >= 0) {
                    map.getCellCentre(pathToCellIdx, targetPos);
                    calculateRequiredMovement(robotPose, targetPos, requiredMovements);
                    Comms::sendMovePackage(Comms::MoveType::REL_TURN_AND_STRAIGHT, requiredMovements(0),
                                           requiredMovements(1));
//                        movementFinished = false;
                    robotMoving = true;
                }


            }



                // ************************ MAP ANALYSIS ****************************
                // TODO move to separate function
                // if some time elapsed or number of images taken
                // should also check if there's been any movement since last time
                // TODO consider if should wait for current movement to end?
                // TODO ? Run in separate thread - so need to check that previous run has finished

                tNow = std::chrono::steady_clock::now();
                if (nIter == 0 ||
                    !robotInMap ||
                    (!mapAnalysisRunning && (nIter > nLastMapAnalysis + mapAnalysisPeriodN))) {

                    if (robotMoving) {
                        // delay map analysis and path planning until movement finished
                        if (!newMapRequest) {
                            std::cout << "Request to block next movement to allow path planning\n";
                            newMapRequest = true;
                        }
                    } else {
                        std::cout << "Running map extraction and analysis.\n";

                        resBool = mapExtractionAndAnalysis(SLAM, robotPos, scale, stepSize, mapLimit, mapPoints,
                                                           map); // TODO check return value
                        if (resBool) {
                            newMapAvailable = true;
                            map.printMapValues(false);
                        }
//                        tLastMapAnalysis = std::chrono::steady_clock::now();
                        nLastMapAnalysis = nIter;
                        newMapRequest = false; // might already be false, but no longer needed to block next movement
                    }

                }

                // ************************ PATH PLANNING ****************************
                // TODO consider if any other criteria for running?

                if (newMapAvailable) {
                    std::cout << "Running path planning.\n";
                    path.clear();
                    if (map.numCells > 0) {
                        int startIdx = map.getIndexUnsafe(robotPos(0), robotPos(1)); // get the map position of the robot again as there may have been movement since the last check
                        // goalCellIdx = map.numCells - 1;     // PLACEHOLDER
                        // try and set a goal as far 'forward' as possible
                        //  i.e. in the last cell along the x axis from the current robot position
                        // use Y position of robot, take max X position of grid
                        if (firstTimePathPlanRun) {
                            std::cout << "First time setting goal. Use a point ahead.\n";
                            Eigen::Vector2i robotXY, goalXY;
                            map.getXY(robotCellIdx, robotXY);
                            std::cout << "Robot in X,Y: " << robotXY(0) << "," << robotXY(1) << "\n";
                            goalXY(0) = map.resX - 1;
                            goalXY(1) = robotXY(1);
                            std::cout << "Goal set to X,Y: " << goalXY(0) << "," << goalXY(1) << "\n";
                            goalCellIdx = map.getIndexUnsafe(goalXY);
                        } else {
                            std::cout << "Goal already exists, try to find corresponding grid cell in new map.\n";
                            goalCellIdx = map.getIndexUnsafe(persistentGoal(0), persistentGoal(1)); // TODO add vector2f overload to this function?
                        }

                        if (goalCellIdx < 0) {
                            goalCellIdx = map.numCells - 1; // JUST IN CASE
                            std::cout << "Goal cell not in map. Setting new one.\n";
                        }

                        // try and save this position and then use it to pick the goal subsequently
                        if (firstTimePathPlanRun) {
                            map.getCellCentre(goalCellIdx, persistentGoal);
                            firstTimePathPlanRun = false;
                            std::cout << "First goal at x, y: " << persistentGoal(0) << ", " << persistentGoal(1)
                                      << "\n";
                        }


                        pathPlanAvailable = PathPlanning::a_star(map, startIdx, goalCellIdx, path);
                        if (pathPlanAvailable && path.size() > 1) {
                            newPathPlanAvailable = true;
                            //goalCellIdx = path[path.back()]; // unnecessary
                            goalExists = true;
                            std::cout << "Path planning successful for cell " << startIdx << " to " << goalCellIdx
                                      << "\n";
                            Eigen::Vector2i p;
                            for (auto i: path) {
                                map.getXY(i, p);
                                std::cout << i << ": " << p.transpose() << " : " << map.cellData(i, GridMap::MEAN)
                                          << "\n";
                            }


                        } else {
                            goalCellIdx = -1;
                            goalExists = false;
                            std::cout << "Path planning failed\n";
//                        std::cout << map;
//                        map.printMapValues(false);
                        }
                        pathStepIdx = 0; // reset to beginning of new path regardless of whether valid
                    } else {
                        goalCellIdx = -1;
                        goalExists = false;
                        std::cout << "No viable map?\n";
                    }
                    newMapAvailable = false;
                }

            }




        // check if pose outside allowed thresholds
        // if so, may need to stop / reverse last movement?
        // TODO probably need a cancel movement command (without losing the movement estimation data)

        nIter++;
        std::cout << "Iterations " << nIter << "\n";

        if(std::chrono::steady_clock::now() - tLastFpsReport > tFpsPeriod) {
            static int lastIter = 0;
            std::cout << "FPS " << (float)(nIter-lastIter)/tFpsPeriod.count() << "\n";
            tLastFpsReport += tFpsPeriod;
        }


    }





    // ***************************************************************
    // ********************* SHUTDOWN ********************************
    // ***************************************************************

    std::cout << "Setting robot status to shutdown\n";
    setRobotStatus(Comms::Status::SHUTDOWN, std::chrono::seconds(3));


    // Stop listening for communications from the robot
    Comms::shutdown();
    commThread.join(); // make sure it has finished


    return 0;
}




// ***************************************************************
// ***************** SUPPORTING FUNCTIONS ************************
// ***************************************************************



bool setOrCheckRobotStatus(const Comms::Status &newOrExpectedStatus, const std::chrono::seconds &timeoutPeriod) {

    auto tStart = std::chrono::steady_clock::now();
    auto tNow = std::chrono::steady_clock::now();
    bool response = false;
    bool timeout = false;
    Comms::Status actualStatus;
    // TODO add method to check if new status response (and it can reset flag back to false)
    while (!response && !timeout) {
        tNow = std::chrono::steady_clock::now();
        timeout = tNow - tStart >= timeoutPeriod;
        response = Comms::newStatusResponse;
        actualStatus = Comms::statusPackageResponse.status;
        usleep(250000);
    }

    if (timeout) {
        std::cout << "No response from robot in"
                  << timeoutPeriod.count()
                  << "seconds\n";
        return false;
    }


    if (response) {
        if (actualStatus != newOrExpectedStatus) {
            std::cout << "Robot not in expected status\n";
            std::cout << "Robot in status:" << static_cast<int>(actualStatus) << "\n";
            return false;
        } else {
            std::cout << "Status ok!\n";
        }
        Comms::newStatusResponse = false; // TODO replace with accessor method that does this
    }
    return true;
}


bool setRobotStatus(const Comms::Status &newStatus, const std::chrono::seconds &timeoutPeriod) {

    Comms::sendStatusPackage(newStatus);
    return setOrCheckRobotStatus(newStatus, timeoutPeriod);

}

bool checkRobotStatus(const Comms::Status &expectedStatus, const std::chrono::seconds &timeoutPeriod) {

    Comms::sendStatusPackage(Comms::Status::REPORT_ONLY);
    return setOrCheckRobotStatus(expectedStatus, timeoutPeriod);
}


// will always start from the centre
bool servoSweep() {
    bool isSweepComplete;
    static int angle = 0;
    static int direction = 1;
    constexpr int posMin = -20;
    constexpr int posMax = 20;
    constexpr int inc = 2;

    if (direction == 1) {
        angle += inc;
        Comms::sendServoPackage((float) angle);
    }
    if (direction == -1) {
        angle -= inc;
        Comms::sendServoPackage((float) angle);
    }
    if (angle >= posMax) {
        direction = -1;
    }
    if (angle <= posMin) {
        direction = 1;
    }

    // it has got back to the beginning (pointed forward)
    // TODO could some combination of limits and increments lead to not getting back to zero (i.e. might skip)?
    if (direction == 1 && angle == 0) {
        isSweepComplete = true;
    } else {
        isSweepComplete = false;
    }

    std::cout << "Angle " << angle << ", complete " << (int) isSweepComplete << "\n";

    // wait to ensure camera is still (could probably be more conservative than this)
    //int waitForServoMs = (int) (abs(anglePrev - angle) * 100000.0);
    usleep(100000);

    return isSweepComplete;


}


// note that all points passed to this function are still in orbslam coord frame
// can't make SLAM const because some of the functions are not marked as const in orb slam lib
bool mapExtractionAndAnalysis(ORB_SLAM2::System& SLAM, const Eigen::RowVector3f robotPos,
                              const float scale, const float stepSize, const float mapLimit,
                              Eigen::MatrixXf& mapPoints, GridMap& map) {
    std::cout << "Running map analysis\n";

    static int outputCnt = 0;

    // Extract map
    // TODO review for efficiency
    // TODO don't add bad points
    // TODO add float conversion earlier?
    ORB_SLAM2::Map* slamMap = SLAM.getMap(); // this won't change right? Check and move to below SLAM initialisation
    std::vector<ORB_SLAM2::MapPoint*> slamMapPoints = slamMap->GetAllMapPoints();
    // For all these MapPoint object (pointers), need to take the actual position
    // Should any of these be ignored?
    int numPoints = slamMapPoints.size();
    mapPoints.resize(numPoints,3);
    int newIdx = 0; // use this as index into new mapPoints matrix since i includes bad points
    for (int i = 0; i < numPoints; i++)
    {
        ORB_SLAM2::MapPoint* point = slamMapPoints[i];
        if(point->isBad()) {
            numPoints--;
        }
        else {
            cv::Mat pointPos = point->GetWorldPos();
            Eigen::Matrix<float, 3, 1> pointPosEig = (ORB_SLAM2::Converter::toVector3d(pointPos)).cast<float>();
            mapPoints.row(newIdx) = pointPosEig.transpose();
            newIdx++;
        }
    }
    if((size_t)numPoints != slamMapPoints.size()) {
        // there were some bad points in the map, so numPoints got reduced (this is fairly unusual in my experience)
        mapPoints.conservativeResize(numPoints,3);
    }

    // Scale and transform to common coordinate system
    mapPoints *= scale;
    Eigen::MatrixXf mapPointsWorld(mapPoints.rows(), mapPoints.cols());
    Transformations::getMapInWorld(mapPoints, mapPointsWorld);

    // Create grid map
    //int numSteps = 10; // TODO how to determine stepsize?
    //   based on robot size (but requires scale adjustment first)
    //float stepSize = (mapPointsWorld.maxCoeff() - mapPointsWorld.minCoeff()) / (float)numSteps;
    map.initialise(stepSize, robotPos, mapLimit, mapPointsWorld);
    std::cout << map;

    outputMap(std::to_string(outputCnt), mapPointsWorld);
    outputCnt++;

    return true; // TODO check if all successful and return false if not
}

// Copy of ORB SLAM function but extended for a 4x4
Eigen::Matrix<float,4,4> toMatrix4f(const cv::Mat &cvMat4) {
    Eigen::Matrix<float,4,4> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2), cvMat4.at<float>(0,3),
            cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2), cvMat4.at<float>(1,3),
            cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2), cvMat4.at<float>(2,3),
            cvMat4.at<float>(3,0), cvMat4.at<float>(3,1), cvMat4.at<float>(3,2), cvMat4.at<float>(3,3);

    return M;
}

void calculateRequiredMovement(const Eigen::Matrix<float, 4, 4>& robotPose,
                          const Eigen::Vector2f& targetPos,
                          Eigen::Vector2f& requiredMovements) {
    // PLaceholder
    requiredMovements.setZero(); // TODO add actual calc

    // NOTE that in my grid cell system, the y axis goes in the opposite direction
    // as it does in the robot frame
    // though this doesn't matter if we just use grid centres
    // and in fact it's not really the opposite direction, it's just displayed that way

    // (shortest) angle between current robot heading and heading from start to finish point
    // robotPose contains the robot orientation described in the world frame
    // for simplicity for this calculation, assume that the ground is (roughly) flat
    // TODO ideally this should be updated (for heading and distance)
    float currentHeading = atan2(-robotPose(0,1), robotPose(0,0));
    float targetHeading = atan2(targetPos(1)-robotPose(1,3), targetPos(0)-robotPose(0,3));
    float turnAngle = targetHeading - currentHeading;
    if(turnAngle > M_PI) turnAngle -= (2*M_PI);
    if(turnAngle < -M_PI) turnAngle += (2*M_PI);
    // Note that an anti-clockwise rotation is negative (opposite to common 2D representation)

    // distance between points
    float forwardDistance = (targetPos - robotPose.block(0,3,2,1)).norm();

    std::cout << "Current position and heading: " << robotPose.block(0,3,2,1).transpose() << ", " << currentHeading << "\n";
    std::cout << "Target position and heading: " << targetPos.transpose() << ", " << targetHeading << "\n";
    std::cout << "Movement: " << turnAngle << ", " << forwardDistance << "\n";

    requiredMovements(0) = turnAngle;
    requiredMovements(1) = forwardDistance;
    std::cout << "To send: " << requiredMovements.transpose() << "\n";

    // TODO actually return these values
}



void outputMap(std::string fileName, const Eigen::MatrixXf& mapPcd) {

    string outputMapFileName = "output/map_" + fileName + ".txt";
    ofstream fileMap(outputMapFileName);
    if (fileMap.is_open() && mapPcd.rows() > 0) {
        std::cout << "Map with " << mapPcd.rows() << "\n";
        fileMap << mapPcd;
    }
    fileMap.close();
}