#include <Arduino.h>
#include "Receiver.h"
#include "Parameters.h"

// Could add mapping to the class, but make it configurable?
// should I also manage the timings here? No

static const uint8_t OK = 1;
static const uint32_t heartbeatTimeout = 500; // milliseconds
const uint8_t address[6] = "1Node";
const uint8_t pipeNumber = 1;

Receiver::Receiver(uint8_t ce, uint8_t csn): radio(ce, csn) {
}

void Receiver::setup() {
  radio.begin();
  radio.setPALevel(RF24_PA_MIN);  // MIN, LOW, HIGH, MAX
  radio.enableAckPayload();
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_250KBPS); // slower is more reliable and gives longer range
  radio.openReadingPipe(pipeNumber, address);
  radio.startListening();
}


bool Receiver::checkForNewData() {
  if ( radio.available()) {
    radio.read( &rcPackage, sizeof(rcPackage) );
    // load acknowledgement payload for the next transmission (first transmission will not get any ack payload (but will get normal ack))
    radio.writeAckPayload(1, &ackStatus, sizeof(ackStatus));
    if (rcPackage.checksum != calculateCheckSum()) {
      radio.flush_rx();
      return false;
    }
    lastRxReceived = millis();
    radio.flush_rx(); // probably remove
    setAck(); // for next time
    return true;
  }
  return false;
}

bool Receiver::checkHeartbeat() {
  if (millis() - lastRxReceived > heartbeatTimeout) {
    return false;
  }
  else {
    return true;
  }
}


uint8_t Receiver::getThrottle(){
  return rcPackage.throttle;
}
uint8_t Receiver::getRoll(){
  return rcPackage.roll;
}
uint8_t Receiver::getPitch(){
  return rcPackage.pitch;
}
uint8_t Receiver::getYaw(){
  return rcPackage.yaw;
}
uint8_t Receiver::getControlFlags(){
  return rcPackage.control;
}

// Report the battery level back to the transmitter
// only expecting values up to 7 (any bits above will be lost anyway)
// will store them in bits 5/6/7 of the acknowledgement byte (as ecpected by the transmitter)
void Receiver::setAckBattery(uint8_t ackBattery) {
  ackStatus &= 0b00011111; // Remove previous battery information
  ackStatus |= ackBattery << 5;
}

// Space for reporting other information
// only expecting values up to 7 (any other bits will be cleared)
void Receiver::setAckFlags(uint8_t ackFlags) {
  ackFlags &= 0b00000111; // Remove anything not in the allowed positions
  ackStatus &= 0b11100011; // Remove previous information
  ackStatus |= ackFlags << 2;
}

// TODO change these numbers to pre-defined consts
void Receiver::armingProcedureBlocking() {
  while (!checkForNewData() && rcPackage.throttle < 50) {
    // wait until receiving transmissions and stick is low
  }
  while (rcPackage.throttle < 200) {
    checkForNewData(); // wait until the stick is moved high
  }
  while (rcPackage.throttle > 50) {
    checkForNewData(); // wait until the stick is moved low
  }
}

// this one needs to be called every time we have input
// it will return false until the procedure is complete
bool Receiver::armingProcedure() {
  static bool stage1 = false;
  static bool stage2 = false;
  static bool stage3 = false;

  
  if (!stage1 && rcPackage.throttle < 30) {
    // wait until receiving transmissions and stick is low
    stage1 = true;
  }

   // wait until the stick is moved high
  if (stage1 && !stage2 && rcPackage.throttle > 200) {
    stage2 = true;
  }

   // wait until the stick is moved low
  if (stage2 && !stage3 && rcPackage.throttle < 30) {
    stage3 = true;
  }

  return stage3;
}


void Receiver::printPackage() {
  Serial.print(rcPackage.throttle); Serial.print('\t');
  Serial.print(rcPackage.roll); Serial.print('\t');
  Serial.print(rcPackage.pitch); Serial.print('\t');
  Serial.print(rcPackage.yaw); Serial.print('\t');
  Serial.print(rcPackage.control); Serial.print('\t');
  Serial.print(rcPackage.alive); Serial.print('\t');
  Serial.print(rcPackage.checksum); Serial.print('\t');
  Serial.print("CHKSUM_DIFF: "); Serial.println(rcPackage.checksum - calculateCheckSum());
}


uint8_t Receiver::calculateCheckSum() {
  uint8_t sum = 0;
  sum += rcPackage.throttle;
  sum += rcPackage.pitch;
  sum += rcPackage.roll;
  sum += rcPackage.yaw;
  sum += rcPackage.control;
  sum += rcPackage.alive;
  sum = 1 - sum;
  return sum;
}

void Receiver::setAck() {
  ackStatus &= 0b11111100; // clear lowest 2 bits
  ackStatus |= OK << 1; // obviously need to change if not ok // NOT YET IMPLEMENTED
  ackStatus |= 1; // set low bit to 1 always to distinguish from a value of zero
}

// TODO why not just shift first and then apply static mask of 0b00000001 ?
uint8_t Receiver::getControlBit(uint8_t position){
  uint8_t mask = 1 << position;
  return (rcPackage.control & mask) >> position;
}
