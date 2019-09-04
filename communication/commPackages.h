#ifndef COMM_PACKAGES_H
#define COMM_PACKAGES_H

namespace Comms {

    const char startByte = '<';
    const char endByte = '>';
    const char delim = '\t';

    enum PackageType {
        EXAMPLE0 = 0,
        MOVE = 1,
        POSE = 2,
        SERVO = 3,
        STATUS = 5,
        EXAMPLE1 = 11,
        EXAMPLE2 = 12
    };

    enum MoveType {
        ABS_TURN_AND_STRAIGHT = 0,
        REL_TURN_AND_STRAIGHT = 1,
        ARC= 2
    };

    enum MoveStatus {
        MOVING = 0,
        FINISHED = 1
    };

    enum Status {
	    UNINITIALISED = 0,
        INITIALISING = 1,
        READY = 2,
        ACTIVE = 3,
        SHUTDOWN = 4,
        SHUTTING_DOWN = 5,
        ERROR = 6,
        REPORT_ONLY = 9,
        UNDER_MANUAL_CONTROL = 10
    };

    struct MovePackageRequest {
        MoveType moveType = MoveType::REL_TURN_AND_STRAIGHT;
        float turn = 0.0;
        float forward = 0.0;
    };

    struct MovePackageResponse {
        MoveStatus moveStatus = MoveStatus::FINISHED;
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
    };


    struct PosePackageRequest {
        uint8_t dummy = 0;
    };

    struct PosePackageResponse {
        float qw = 0.0;
        float qx = 0.0;
        float qy = 0.0;
        float qz = 0.0;
    };

    struct ServoPackageRequest {
        int8_t angle = 0;
    };

    struct ServoPackageResponse {
        int8_t dummy = 0;
    };

    struct StatusPackageRequest {
        Status requestedStatus = Status::UNINITIALISED;
    };

    struct StatusPackageResponse {
        Status status = Status::UNINITIALISED;
    };

    struct ErrorPackageRequest {
        uint8_t dummy = 0;
    };

    struct ErrorPackageResponse {
        uint8_t errorFlags = 0;
    };


    struct SensorPackage {
        float a = 0.0;
        float b = 0.0;
        float c = 0.0;
    };

    struct PosePackage {
        float w = 0.0;
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
    };

}

#endif
