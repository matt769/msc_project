#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset
#include <iostream>
#include <vector>
#include <chrono>


#include <stdint.h>
#include "commPackages.h"
#include "communications.h"


namespace Comms {

    struct termios tio;
    int tty_fd;
    unsigned char c = 0;
    constexpr uint8_t rxBufferSize = 32;
    char rxBuffer[rxBufferSize]; // TODO review size
    uint8_t idx = 0;
    bool receivingPackage = false;
    bool receiptComplete = false;
    bool parseComplete = false;
    uint32_t packageId = 0;
    std::vector<std::string> receivedValues(10); // TODO review size
    const int heartbeatPeriodMillis = 1000;


    MovePackageRequest movePackageRequest;
    PosePackageRequest posePackageRequest;
    ServoPackageRequest servoPackageRequest;
    StatusPackageRequest statusPackageRequest;

    MovePackageResponse movePackageResponse;
    PosePackageResponse posePackageResponse;
    ServoPackageResponse servoPackageResponse;
    StatusPackageResponse statusPackageResponse;

    bool newMoveResponse(false), newPoseResponse(false), newStatusResponse(false), newServoResponse(false);

    bool receivingActive(false);

    // TODO add return value
    // TODO add error checking
    void setup() {
        memset(&tio, 0, sizeof(tio));
        tio.c_iflag = 0; // Not using any of the c_iflag (input) options
        tio.c_oflag = 0; // Not using any of the (c_oflag) output options
        tio.c_cflag = CS8 |                 // 8 bit, no parity, 1 stop bit
                      CREAD |             // Enable receiver
                      CLOCAL;             // Ignore modem control lines (not sure if required)
        tio.c_lflag = 0;  // Not using any of the c_lflag options
        tio.c_cc[VMIN] = 1; // Minimum number of characters for noncanonical read (not sure if necessary given non-blocking option)
        tio.c_cc[VTIME] = 0; // Timeout in deciseconds for noncanonical read (not sure if necessary given non-blocking option)

        // TODO add check if open was successful (check errno?)
        tty_fd = open("/dev/ttyACM0",
                      O_RDWR | O_NONBLOCK); // open for read and write | non-blocking (read will return immediately)
        cfsetospeed(&tio, B115200);            // 115200 baud
        cfsetispeed(&tio, B115200);            // 115200 baud

        tcsetattr(tty_fd, TCSANOW, &tio); // Apply changes (immediately)

        receivingActive = true;
    }

    void receiveInBackground(){
        auto period = std::chrono::milliseconds(heartbeatPeriodMillis);
        std::chrono::time_point<std::chrono::steady_clock> lastRun = std::chrono::steady_clock::now();
        while(receivingActive){
            auto now = std::chrono::steady_clock::now();
            auto elapsedMillis = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastRun);
            if(elapsedMillis > period){
                sendStatusPackage(Status::REPORT_ONLY);
                lastRun += period;
            }

            receive();
            usleep(1000);
        }
    }


    void receive() {
        // SEPARATE FUNCTIONS
        // receive to buffer
        // split to separate value strings
        // convert these into useful values

        // FUNCTION 1
        // if during the reading of data into the buffer, it finds that a complete package has been received
        //      it needs to break out so it can parse it before receiving the next one
        while ((read(tty_fd, &c, 1) > 0) && !receiptComplete) {
            if (c == startByte) {
                receivingPackage = true;
                idx = 0;
//                std::cout << "\nStart\n";
            } else if (c == endByte) {
                // only want to flag as complete if previously received a start marker
                if (receivingPackage) {
                    receivingPackage = false;
                    receiptComplete = true;
                    rxBuffer[idx] = '\0';
//                    std::cout << "\nStop\n";
                }
            } else if (receivingPackage) {
                // handle too large packages just in case
                if (idx >= rxBufferSize) {
                    // stop receiving and wait for new start marker
                    receivingPackage = false;
                } else {
                    rxBuffer[idx] = c;
                    idx++;
                }
//                std::cout << c;
            }
        }

        // FUNCTION 2
        if (receiptComplete) {
            // now split the data (move to separate function?)
//            std::cout << rxBuffer << "\n";
            receivedValues.clear();

            char *val = strtok(rxBuffer, "\t");
            while (val != nullptr) {
                receivedValues.push_back(val);
//                std::cout << val << "\n";
                val = strtok(nullptr, "\t"); // note strtok maintains a static pointer to the previously passed string
            }
            receiptComplete = false;
            parseComplete = true;

//            for (const auto& s: receivedValues){
//                std::cout << s << "\n";
//            }
        }


        // FUNCTION 3
        if (parseComplete) {
            // check what type it is
//            std::cout << "a\n";
            uint8_t packageType = (uint8_t) stoi(receivedValues[0]);
//            std::cout << "b\n";
            // check that it's the expected size

            // do any necessary check on the id
            uint8_t packageId = (uint8_t) stoi(receivedValues[1]);
//            std::cout << "c\n";
            // .....

//            std::cout << (uint16_t)packageType << '\t' << (uint16_t)packageId << '\t';
//            std::cout << "d\n";

            // split appropriately
            switch (packageType) {   // TODO add explicit cast to enum?
                case PackageType::MOVE:
                    movePackageResponse.moveStatus = static_cast<MoveStatus>(stoi(receivedValues[2]));
                    movePackageResponse.x = stof(receivedValues[3]);
                    movePackageResponse.y = stof(receivedValues[4]);
                    movePackageResponse.z = stof(receivedValues[5]);
                    newMoveResponse = true;
                    std::cout << "Move response: " << (int) movePackageResponse.moveStatus << "\t"
                              << movePackageResponse.x << "\t" << movePackageResponse.y << "\t" << movePackageResponse.z
                              << "\n";
                    break;
                case PackageType::POSE:
                    posePackageResponse.qw = stof(receivedValues[2]);
                    posePackageResponse.qx = stof(receivedValues[3]);
                    posePackageResponse.qy = stof(receivedValues[4]);
                    posePackageResponse.qz = stof(receivedValues[5]);
                    std::cout << "Pose response: " << posePackageResponse.qw << "\t" << posePackageResponse.qx << "\t"
                              << posePackageResponse.qy << "\t" << posePackageResponse.qz << "\n";
                    newPoseResponse = true;
                    break;
                case PackageType::SERVO:
                    servoPackageResponse.dummy = stoi(receivedValues[2]);
                    std::cout << "Servo response: " << (int) servoPackageResponse.dummy << "\n";
                    newServoResponse = true;
                    break;
                case PackageType::STATUS:
                    statusPackageResponse.status = static_cast<Status>(stoi(receivedValues[2]));
                    std::cout << "Status response: " << (int) statusPackageResponse.status << "\n";
                    newStatusResponse = true;
                    break;
                default:
                    std::cout << "Unrecognised package type\n";
            }
            parseComplete = false;
        }
    }

    void shutdown(){
        receivingActive = false;
        close(tty_fd);
    }

    // TODO should I use the request package struct or is it redundant?
    // TODO This all seems very long winded, can it be improved?
    void sendMovePackage(MoveType moveType, float turn, float forward){
        std::string cmd;
        cmd += startByte;
        cmd += std::to_string(static_cast<uint8_t>(PackageType::MOVE));
        cmd += delim;
        cmd += std::to_string(packageId); // TODO replace
        cmd += delim;
        cmd += std::to_string(static_cast<uint8_t>(moveType));
        cmd += delim;
        cmd += std::to_string(turn);
        cmd += delim;
        cmd += std::to_string(forward);
        cmd += endByte;

        //        std::cout << "Sending: " << cmd << "\n";
        size_t ret = write(tty_fd, cmd.c_str(), cmd.length());
        if(ret != cmd.length()){
                std::cout << "Sending error - need to handle properly\n";
        }

        packageId++;
    }

    void sendPosePackage(void){

        std::string cmd;
        cmd += startByte;
        cmd += std::to_string(static_cast<uint8_t>(PackageType::POSE));
        cmd += delim;
        cmd += std::to_string(packageId);
        cmd += delim;
        cmd += std::to_string(0); // dummy
        cmd += endByte;

//        std::cout << "Sending: " << cmd << "\n";
        size_t ret = write(tty_fd, cmd.c_str(), cmd.length());
        if(ret != cmd.length()){
                std::cout << "Sending error - need to handle properly\n";
        }

        packageId++;
    }

    void sendServoPackage(float angle){

        std::string cmd;
        cmd += startByte;
        cmd += std::to_string(static_cast<uint8_t>(PackageType::SERVO));
        cmd += delim;
        cmd += std::to_string(packageId);
        cmd += delim;
        cmd += std::to_string(angle);
        cmd += endByte;

        //        std::cout << "Sending: " << cmd << "\n";
        size_t ret = write(tty_fd, cmd.c_str(), cmd.length());
        if(ret != cmd.length()){
                std::cout << "Sending error - need to handle properly\n";
        }

        packageId++;
    }

    void sendStatusPackage(Status requestedStatus){

        std::string cmd;
        cmd += startByte;
        cmd += std::to_string(static_cast<uint8_t>(PackageType::STATUS));
        cmd += delim;
        cmd += std::to_string(packageId);
        cmd += delim;
        cmd += std::to_string(static_cast<uint8_t>(requestedStatus)); // dummy
        cmd += endByte;

        //        std::cout << "Sending: " << cmd << "\n";
        size_t ret = write(tty_fd, cmd.c_str(), cmd.length());
        if(ret != cmd.length()){
                std::cout << "Sending error - need to handle properly\n";
        }

        packageId++;
    }




}
