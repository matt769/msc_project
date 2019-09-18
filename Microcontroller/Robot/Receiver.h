#ifndef RECEIVER_H
#define RECEIVER_H

#include <RF24.h>

class Receiver {
  public:
    Receiver(uint8_t ce, uint8_t csn);
    void setup();
    bool checkHeartbeat();
    bool checkForNewData();
    uint8_t getThrottle();
    uint8_t getRoll();
    uint8_t getPitch();
    uint8_t getYaw();
    uint8_t getControlFlags();
    void setAckBattery(uint8_t ackBattery);
    void setAckFlags(uint8_t ackFlags);
    void armingProcedureBlocking();
    bool armingProcedure();
    void printPackage();
    uint8_t getControlBit(uint8_t position);

  private:
    RF24 radio;
    struct RCPackage {
      uint8_t throttle;
      uint8_t roll;
      uint8_t pitch;
      uint8_t yaw;
      uint8_t control; // control bit flags
      uint8_t alive; // this will increment every time the data is sent
      uint8_t checksum;
    } rcPackage;
    uint8_t ackStatus;  // acknowledgement status
    bool rxHeartbeat;
    unsigned long lastRxReceived;
    uint8_t calculateCheckSum();
    void setAck();
    
};

#endif
