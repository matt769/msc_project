#include "transformations.h"

#include <Eigen/Dense>

// There is definitely quite a bit of repetition here
// Not structured very optimally

namespace Transformations {

    Eigen::Matrix<float, 4, 4> T_0_1, T_1_2, T_2_3, T_3_4, T_4_5;
    Eigen::Matrix<float, 4, 4> T_0_2, T_0_3, T_0_4, T_0_5;
    Eigen::Matrix<float, 4, 4> T_world_orb;


    const float servoTiltAngleRad = -17 * M_PI / 180.0;
//    Eigen::Matrix<float, 4, 4> T_0_1, T_1_2, T_2_3, T_3_4, T_4_5;
//    Eigen::Matrix<float, 4, 4> T_0_2, T_0_3, T_0_4, T_0_5, T_robot_cam, T_world_orb;

    void rotatePoints(const Eigen::Matrix<float, 4, 4>& T,
                        const Eigen::MatrixXf& points,
                        Eigen::MatrixXf& rotatedpoints) {

        Eigen::Matrix<float, 3, 3> R = T.block(0,0,2,2);
        Eigen::Matrix<float, 3, 1> t = T.block(0,3,3,1);
        rotatedpoints = ((R*points.transpose()).colwise() + t).transpose();
    }


    void createRotationX(float angleRad, Eigen::Matrix<float, 3, 3>& R) {
        R.setZero();
        R(0,0) = 1.0;
        R(1,1) = cos(angleRad);
        R(1,2) = -sin(angleRad);
        R(2,1) = sin(angleRad);
        R(2,2) = cos(angleRad);
    }

    void createRotationY(float angleRad, Eigen::Matrix<float, 3, 3>& R) {
        R.setZero();
        R(1,1) = 1.0;
        R(0,0) = cos(angleRad);
        R(0,2) = sin(angleRad);
        R(2,0) = -sin(angleRad);
        R(2,2) = cos(angleRad);
    }

    void createRotationZ(float angleRad, Eigen::Matrix<float, 3, 3>& R) {
        R.setZero();
        R(0,0) = cos(angleRad);
        R(0,1) = -sin(angleRad);
        R(1,0) = sin(angleRad);
        R(1,1) = cos(angleRad);
        R(2,2) = 1.0;
    }

    // all transformation matrices now contain a correct (at the moment) and valid transformation
    void initialiseTransformations(const float scale, const float servoAngleDeg,
                                   const Eigen::Matrix<float, 4, 4>& T_cam_orb) {

        Eigen::Matrix<float, 3, 3> R, Rx, Ry, Rz;

        // IMU frame in world frame
        T_0_1.setIdentity();
        T_0_1(2,3) = -12.5;

        // Servo frame in IMU frame
        T_1_2.setIdentity();
        T_1_2(0,3) = -9.0;
        T_1_2(2,3) = -11.5;
        float servoAngleRad = servoAngleDeg * M_PI / 180.0; // assumes servo at 0
        createRotationZ(servoAngleRad, Rz);
        createRotationY(servoTiltAngleRad, Ry);
        T_1_2.block(0,0,3,3) = Ry * Rz;
        // Servo frame in world frame
        T_0_2 = T_0_1 * T_1_2;

        // Camera frame (NED orientation) in servo frame
        T_2_3.setIdentity();
        T_2_3(0,3) = 20.0;
        // Camera frame (NED orientation) in world frame
        T_0_3 = T_0_1 * T_1_2 * T_2_3;

        // Camera frame (optical orientation) in servo frame
        T_3_4.setIdentity();
        createRotationZ(M_PI/2.0, Rz);
        createRotationY(M_PI/2.0, Ry);
        T_3_4.block(0,0,3,3) = Ry * Rz;
        // Camera frame (optical orientation) in world frame
        T_0_4 = T_0_1 * T_1_2 * T_2_3 * T_3_4;

        T_4_5 = T_cam_orb;
        T_4_5.block(0,3,3,1) *= scale; // apply scaling to transformation part
        T_0_5 = T_0_4 * T_4_5;
        T_world_orb = T_0_5;
    }

    // update transformation matrices based on a new servo angle
    // T_4_5 is not changed which is probably unlikely
    void updateTransformations(const float servoAngleDeg) {
        float servoAngleRad = ((float)servoAngleDeg) * M_PI / 180.0;
        Eigen::Matrix<float, 3, 3> Ry, Rz;
        createRotationZ(servoAngleRad, Rz);
        createRotationY(servoTiltAngleRad, Ry);
        T_1_2.block(0,0,3,3) = Ry * Rz;
        T_0_2 = T_0_1 * T_1_2;
        T_0_3 = T_0_2 * T_2_3;
        T_0_4 = T_0_3 * T_3_4;
        T_0_5 = T_0_4 * T_4_5;
    }

    // update with new camera pose
    // assumes no change to servo angle (quite possible if robot moving around normally)
    void updateTransformations(const float scale, const Eigen::Matrix<float, 4, 4>& T_cam_orb) {
        T_4_5 = T_cam_orb;
        T_4_5.block(0,3,3,1) *= scale; // apply scaling to transformation part
        T_0_5 = T_0_4 * T_4_5; // update
    }


    // update with new camera pose and servo angle
    void updateTransformations(const float servoAngleDeg, const float scale, const Eigen::Matrix<float, 4, 4>& T_cam_orb) {
        float servoAngleRad = ((float)servoAngleDeg) * M_PI / 180.0;
        Eigen::Matrix<float, 3, 3> Ry, Rz;
        createRotationZ(servoAngleRad, Rz);
        createRotationY(servoTiltAngleRad, Ry);
        T_1_2.block(0,0,3,3) = Ry * Rz;
        T_0_2 = T_0_1 * T_1_2;
        T_0_3 = T_0_2 * T_2_3;
        T_0_4 = T_0_3 * T_3_4;
        T_4_5 = T_cam_orb;
        T_4_5.block(0,3,3,1) *= scale; // apply scaling to transformation part
        T_0_5 = T_0_4 * T_4_5; // orbslam frame w.r.t. robot frame
    }

    // Assumes that the transformations have been updated with the latest camera pose estimate from orbslam
    void getRobotPositionInWorld(Eigen::RowVector3f& robotPos) {
        Eigen::Matrix<float, 4, 4> T_world_robot;
        getRobotPoseInWorld(T_world_robot);
        robotPos = T_world_robot.block(0,3,3,1).transpose();
    }

    // maybe change to be able to return pitch/roll etc
    void getRobotRotationInWorld(Eigen::Matrix<float, 3, 3>& R_world_robot) {
        Eigen::Matrix<float, 4, 4> T_world_robot;
        getRobotPoseInWorld(T_world_robot);
        R_world_robot = T_world_robot.block(0,0,3,3);
    }

    void getRobotPoseInWorld(Eigen::Matrix<float, 4, 4>& T_world_robot) {
        Eigen::Matrix<float, 4, 4> T_orb_cam, T_cam_robot;
        inverseTransformation(T_4_5, T_orb_cam); // T_4_5 is T_cam_orb
        inverseTransformation(T_0_4, T_cam_robot); // T_0_4 is T_robot_camOpt

        T_world_robot = T_world_orb * T_orb_cam * T_cam_robot;
    }

    void inverseTransformation(const Eigen::Matrix<float, 4, 4>& T, Eigen::Matrix<float, 4, 4>& Tinv) {
        Eigen::Matrix<float, 3, 3> Rinv = T.block(0, 0, 3, 3).transpose();
        Tinv.block(0, 0, 3, 3) = Rinv;
        Tinv.block(0, 3, 3, 1) = -Rinv * T.block(0, 3, 3, 1);
        Tinv.block(3,0,1,3).setZero();
        Tinv(3,3) = 1.0;

    }

    void getMapInWorld(const Eigen::MatrixXf& points, Eigen::MatrixXf& rotatedpoints) {
        rotatePoints(T_world_orb, points, rotatedpoints);
    }


}
