#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <Eigen/Dense>

namespace Transformations {

    // Frame 0: Robot reference frame - at the beginning, this will also by my NED world frame
    // Frame 1: IMU
    // Frame 2: Servo
    // Frame 3: Camera (NED-like)
    // Frame 4: Camera (optical)
    // Frame 5: Orbslam frame

    // Where transformations are written as T_aaa_bbb
    //  bbb is the frame being described
    //  with respect to frame aaa


    extern Eigen::Matrix<float, 4, 4> T_0_1, T_1_2, T_2_3, T_3_4, T_4_5;
    extern Eigen::Matrix<float, 4, 4> T_0_2, T_0_3, T_0_4, T_0_5;

    // Alternate names
    // T_0_1 -> T_robot_imu
    // T_1_2 -> T_imu_servo
    // T_2_3 -> T_servo_camNed
    // T_3_4 -> T_camNed_camOpt
    // T_4_5 -> T_camOpt_orbworld
    // T_0_2 -> T_robot_servo
    // T_0_3 -> T_robot_camNed
    // T_0_4 -> T_robot_camOpt
    // T_0_5 -> T_robot_orbworld

    // When the frames are first calculated (robot has not moved anywhere)
    //  robot = world i.e. T_0_5 = T_world_orbworld


    // this gives the transformation between camera and robot
    // so can be used to get robot pose from orbslam camera pose
    // this needs to be updated with servo angle
    //Eigen::Matrix<float, 4, 4> T_robot_cam;
    // JUST USE T_0_4

    // this gives the transformation betweeh orbslam frame (in which the map points are)
    //  and the NED world frame
    // this should only be calculated once at the beginning (when it'll be the same as T_0_5
    // (probably won't use T_0_5 after that)
    extern Eigen::Matrix<float, 4, 4> T_world_orb;

    void rotatePoints(const Eigen::Matrix<float, 4, 4>& T,
                      const Eigen::MatrixXf& points,
                      Eigen::MatrixXf& rotatedpoints);

    void createRotationX(float angleRad, Eigen::Matrix<double, 3, 3>& R);
    void createRotationY(float angleRad, Eigen::Matrix<double, 3, 3>& R);
    void createRotationZ(float angleRad, Eigen::Matrix<double, 3, 3>& R);

    void initialiseTransformations(const float scale, const float servoAngleDeg,
                                   const Eigen::Matrix<float, 4, 4>& T_cam_orb);

    void updateTransformations(const float servoAngleDeg);

    void updateTransformations(const float scale, const Eigen::Matrix<float, 4, 4>& T_cam_orb);

    void updateTransformations(const float servoAngleDeg, const float scale,
                                const Eigen::Matrix<float, 4, 4>& T_cam_orb);

    void inverseTransformation(const Eigen::Matrix<float, 4, 4>& T, Eigen::Matrix<float, 4, 4>& Tinv);

    void getRobotPositionInWorld(Eigen::RowVector3f& robotPos);

    // maybe change to be able to return pitch/roll etc
    void getRobotRotationInWorld(Eigen::Matrix<float, 3, 3>& R_world_robot);

    void getRobotPoseInWorld(Eigen::Matrix<float, 4, 4>& T_world_robot);

    void getMapInWorld(const Eigen::MatrixXf& points, Eigen::MatrixXf& rotatedpoints);
}

#endif