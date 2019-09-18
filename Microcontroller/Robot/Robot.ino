#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#include "Receiver.h"
#include "Motors.h"
#include "MPU9250.h"
#include <PWMServo.h>
#include <PID_v1.h>
#include "/home/matt/ws/project/communication/commPackages.h"
#include <Rotation.h>
#include"movingAverageFilter.h"


using namespace Comms;

uint8_t sendPackageId = 0;

// STATUS - note this is currently part of Comms, maybe should change that
Status status = Status::UNINITIALISED;
Status statusPrev = Status::UNINITIALISED;
constexpr uint8_t pinLED = 13;

MoveStatus moveStatus = MoveStatus::FINISHED;
MoveStatus moveStatusPrev = MoveStatus::FINISHED;
namespace MoveStatusDetail {
enum Enum {
  NONE,
  TURN,
  FORWARD,
  ARC
};
}
MoveStatusDetail::Enum moveStatusDetail = MoveStatusDetail::NONE;



// MODE
namespace Mode {
enum Enum {
  MANUAL,
  AUTO
};
}
Mode::Enum mode = Mode::AUTO;
Mode::Enum modePrev = Mode::AUTO;

namespace TurnDirection {
enum Enum {
  NONE,
  LEFT,
  RIGHT
};
}
TurnDirection::Enum turnDirection = TurnDirection::NONE;

namespace RobotDirection {
enum Enum {
  FORWARD,
  BACKWARD
};
}
RobotDirection::Enum robotDirection = RobotDirection::FORWARD;


// For communication with Odroid
constexpr uint8_t rxBufferSize = 32;
char rxBuffer[rxBufferSize]; // TODO review size

MovePackageRequest movePackageRequest;
PosePackageRequest posePackageRequest;
ServoPackageRequest servoPackageRequest;
StatusPackageRequest statusPackageRequest;

PackageType packageType = PackageType::EXAMPLE0;
bool newMoveRequest(false), newPoseRequest(false), newServoRequest(false), newStatusRequest(false);

constexpr uint32_t moveUpdatePeriod = 500; // how often during a movement phase that the robot will send an update to the host with distance so far



// Motors
constexpr uint8_t pinDir1 = 38;
constexpr uint8_t pinPwm1 = 37;
constexpr uint8_t pinDir2 = 36;
constexpr uint8_t pinPwm2 = 35;

Motor motorLeft(pinDir1, pinPwm1);
Motor motorRight(pinDir2, pinPwm2);
uint8_t throttleLeft;
uint8_t throttleRight;

constexpr uint8_t LOW_THROTTLE_LIMIT = 30;
constexpr uint8_t STICK_DEADZONE = 8; // on either side of centre
constexpr uint8_t STICK_CENTRE = 127;
constexpr uint8_t THROTTLE_MAX = 250;

// Servo
constexpr uint8_t pinServo = 30;
PWMServo camServo;
constexpr int servoPositionCentre = 85;
int servoPosition = servoPositionCentre; // offset + 0 // note that position is relative to centred position
constexpr int SERVO_POS_MIN = servoPositionCentre - 35; // to the left
constexpr int SERVO_POS_MAX = servoPositionCentre + 35; // to the right


// Receiver
constexpr uint8_t pinRadioCE = 25;  // RF24 enable
constexpr uint8_t pinRadioCSN = 26; // SPI Chip Select
Receiver receiver(pinRadioCE, pinRadioCSN);
uint32_t receiverLast = 0;
constexpr uint32_t receiverFreq = 50;
bool newRxData;
bool radioArmed = false;

// Encoders
constexpr uint8_t pinEncoderIntL = 18; // S1 (orange)
constexpr uint8_t pinEncoderOtherL = 17; // S2 (green)
constexpr uint8_t pinEncoderIntR = 16; // S1 (orange)
constexpr uint8_t pinEncoderOtherR = 15; // S2 (green)
volatile int32_t encCountLeft = 0;
volatile int32_t encCountRight = 0;

float trackSpeedLeft;
float trackSpeedRight;


// PIDs
// Track speeds are in millimetres per second
uint32_t lastCalculateSpeeds = 0;
constexpr uint32_t pidPeriodSpeeds = 20; // milliseconds (e.g. 20ms -> 1000/20 = 50Hz)

float leftTrackSpeedSetpoint = 0, leftTrackSpeedOutput;
float rightTrackSpeedSetpoint = 0, rightTrackSpeedOutput;
float leftKp = 0.1, leftKi = 0, leftKd = 0;
float rightKp = 0.1, rightKi = 0, rightKd = 0;

PID pidLeftTrackSpeed(&trackSpeedLeft, &leftTrackSpeedOutput, &leftTrackSpeedSetpoint, leftKp, leftKi, leftKd, PID_DIRECT);
PID pidRightTrackSpeed(&trackSpeedRight, &rightTrackSpeedOutput, &rightTrackSpeedSetpoint, rightKp, rightKi, rightKd, PID_DIRECT);



uint32_t lastCalculateHeading = 0;
constexpr uint32_t pidPeriodHeading = 100; // milliseconds

float heading = 0, requiredAngularVelocity = 0, headingSetpoint = 0;
float zeroHeading = 0;
float headingError = 0; // This will be used along with a setpoint of zero instead of the actual heading and setpoint
//  as it allows easier control of the heading wrapping
float headingKp = 2, headingKi = 0, headingKd = 0;
PID pidHeading(&headingError, &requiredAngularVelocity, &zeroHeading, headingKp, headingKi, headingKd, PID_DIRECT);




// Track movement
Vector position(0, 0, 0);


// Note, turn target is headingSetpoint so no separate variable created
float forwardTarget = 0.0;
float forwardActual = 0.0;

constexpr float headingTolerance = 5.0 * PI / 180.0;
constexpr float forwardTolerance = 5.0; // millimetres

uint32_t lastAccumulate;
constexpr uint32_t accumulatePeriod = 5;
uint32_t lastHandleMovement;
constexpr uint32_t handleMovementPeriod = 20;



// Robot measurements
constexpr int32_t driveSprocketRadius = 21;
constexpr float driveSprocketCircumference = 2.0 * PI * (float)driveSprocketRadius;
constexpr int32_t pulsesPerMotorRotation = 2;
constexpr int32_t gearRatio = 75;
constexpr float distancePerPulse = driveSprocketCircumference / (float)(pulsesPerMotorRotation * gearRatio);
constexpr float distanceBetweenTracks = 211.0;


// IMU
MPU9250 IMU(SPI1, 4);
int imuStatus;

unsigned long readImuLast = 0;
constexpr unsigned long readImuPeriod = 2; // milliseconds, e.g. 2ms = 500Hz
constexpr float readImuPeriodFloat = (float)readImuPeriod / 1000.0; // convert to seconds
Euler eulerAngles;
Quaternion orientation(1, 0, 0, 0);
float alpha = 0.01; // weight of accel in complementary filter

movingAverageFilter magXfilter;
movingAverageFilter magYfilter;
movingAverageFilter magZfilter;






// INTERRUPTS
// Because the interrutps are set to trigger on CHANGE we need to find out whether it was rising or falling
// So just read the pin again, if it's high, then it was rising
// And in one direction, both pins will be the same level, and in the other they'll be the opposite
void incrementCounterL() {
  if (digitalReadFast(pinEncoderIntL) == digitalReadFast(pinEncoderOtherL)) {
    encCountLeft += 1;
  }
  else {
    encCountLeft -= 1;
  }
}

void incrementCounterR() {
  if (digitalReadFast(pinEncoderIntR) != digitalReadFast(pinEncoderOtherR)) {
    encCountRight += 1;
  }
  else {
    encCountRight -= 1;
  }
}


// **************************************************************************************
// ************************************  SETUP  *****************************************
// **************************************************************************************

void setup() {
  //delay(2000); // required so we don't miss the first serial prints
  //while (!Serial); // This is required for the Teensy to get set up properly as a serial port on Linux
  // TODO ideally we should be able to start up with just the radio

  statusPrev = status;
  status = Status::INITIALISING;
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, HIGH);

  // Radio alternative SPI pins
  SPI.setSCK(27);

  // IMU alternative SPI pins
  SPI1.setSCK(20);
  SPI1.setMISO(5);
  SPI1.setMOSI(21);

  // Radio setup
  receiver.setup(); // TODO add check and halt program like IMU if required
  receiver.setAckBattery(7); // report full battery (to stop controller battery light flashing a lot)

  // if there's no serial connection but we do have radio input, then continue
  while ((!Serial) && (!receiver.checkForNewData())) {
    delay(100);
  }


  // Encoder setup
  setupEncoders();

  // IMU setup
  imuStatus = IMU.begin();
  if (imuStatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imuStatus);
    while (1) {} // TODO change to set status and error flags
  }

  // Specific settings
  // Some of these are already the default but for clarity I put them here anyway
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // There aren't really any options available for Mag
  // Except read rate which is already set to 100Hz within IMU.begin()

  // calibration info
  IMU.setAccelCalX(0.2534242, 0.9977166);
  IMU.setAccelCalY(0.2825031, 0.9991337);
  IMU.setAccelCalZ(0.5199513, 0.9813061);

  IMU.setMagCalX(52.7316322, 0.8228859);
  IMU.setMagCalY(-36.4627380, 1.1055350);
  IMU.setMagCalZ(-24.8827744, 1.1360730);

  // initialise magmetometer filters
  initialiseMagFilters();
  // use accel and mag to get initial orientation
  calculateInitialOrientation();



  // Servo setup
  camServo.write(servoPosition);
  camServo.attach(pinServo);
  camServo.write(servoPosition);

  // wait for the expected radio input (throttle down-up-dowm) and then start main program
  //  receiver.armingProcedure();/
  motorLeft.setDir(MotorDirection::FORWARD);
  motorRight.setDir(MotorDirection::FORWARD);

  // PIDs
  pidLeftTrackSpeed.SetSampleTime(pidPeriodSpeeds);
  pidRightTrackSpeed.SetSampleTime(pidPeriodSpeeds);
  pidLeftTrackSpeed.SetOutputLimits(-30, 30);   // PWM
  pidRightTrackSpeed.SetOutputLimits(-30, 30);   // PWM
  pidLeftTrackSpeed.SetMode(PID_AUTOMATIC); // TODO after testing this should be removed - PID should only be on when a movement is being made
  pidRightTrackSpeed.SetMode(PID_AUTOMATIC); // TODO after testing this should be removed - PID should only be on when a movement is being made

  pidHeading.SetSampleTime(pidPeriodHeading);
  pidHeading.SetOutputLimits(-1, 1);  // Radians
  pidHeading.SetMode(PID_AUTOMATIC);


  // initialise all timing variables
  uint32_t timeNow = millis();
  lastCalculateSpeeds = timeNow;
  lastAccumulate = timeNow;
  lastHandleMovement = timeNow;
  readImuLast = timeNow;


  statusPrev = status;
  status = Status::READY;

  digitalWrite(pinLED, LOW);
  Serial.println(F("Starting..."));

  // FOR TESTING
  //  statusPrev = status;
  //  status = Status::ACTIVE;
}


// **************************************************************************************
// ************************************  LOOP  ******************************************
// **************************************************************************************

void loop() {

  receiveRequests();
  processStatusRequest(); // ALWAYS respond to status request

  // TODO handle status changes

  if (status == Status::ACTIVE) {
    processMoveRequest();
    processPoseRequest();
    processServoRequest();
  }

  // note that it doesn't matter what kind of status the robot is in
  // can always take manual control by turning the radio on
  // at the moment, no way to go back to auto
  if (millis() - receiverLast >= receiverFreq) {
    receiverLast += receiverFreq;
    newRxData = receiver.checkForNewData();
    if (newRxData) {
      //Serial.println("Radio data received");
      if (mode != Mode::MANUAL) {
        modePrev = mode;
        mode = Mode::MANUAL;
      }
      if (status != Status::UNDER_MANUAL_CONTROL) {
        statusPrev = status;
        status = Status::UNDER_MANUAL_CONTROL;
      }
    }
  }

  // TODO handle mode changes

  if ((mode == Mode::MANUAL) && (!receiver.checkHeartbeat())) {
    statusPrev = status;
    status = Status::SHUTDOWN;
  }

  handleStatusChanges();


  if ((mode == Mode::MANUAL) && newRxData) {
    // Why is this not in the checkForNewData() bit?
    // because I want to make it clearer where the mode change is
    if (radioArmed) {
      processRadioInput();
    }
    else {
      radioArmed = receiver.armingProcedure();
    }
    newRxData = false;
  }



  if (status != Status::SHUTDOWN) {

    if (millis() - lastAccumulate > accumulatePeriod) {
      accumulateMovement();
      lastAccumulate += accumulatePeriod;
    }

    // TODO move actual move functions out of handleMovement()
    //   then handleMovement can run every loop
    // and the move functions can be run periodically
    if (millis() - lastHandleMovement > handleMovementPeriod) {
      handleMovement();
      lastHandleMovement += handleMovementPeriod;
    }

    if (millis() - lastCalculateSpeeds > pidPeriodSpeeds) {
      if (moveStatus == MoveStatus::MOVING) {
        calculateTrackSpeeds();
        pidLeftTrackSpeed.Compute();
        pidRightTrackSpeed.Compute();
        // update motors
        // TODO add methods to motors to handle this stuff?
        updateMotorsWithPidOutputs();

        // TODO note that if taking the speed down to 0, it will get there before PWM is zero
        // will need to handle this properly
      }
      lastCalculateSpeeds += pidPeriodSpeeds;
    }


    if (millis() - readImuLast > readImuPeriod) {
      IMU.readSensor(true, true, true); // read accels and gyros and mags
      // only update filters if readings have changed
      // if move read to separate lower frequency loop, may be no need for this
      float mx = IMU.getMagX_uT();
      float my = IMU.getMagY_uT();
      float mz = IMU.getMagZ_uT();

      if (mx != magXfilter.latest() ||
          my != magYfilter.latest() ||
          mz != magZfilter.latest()) {
        magXfilter.update(mx);
        magYfilter.update(my);
        magZfilter.update(mz);
        calculateYaw(magXfilter.output, magYfilter.output, magZfilter.output); // not actually using the result from this at the moment
      }

      gyroUpdate(IMU.getGyroX_rads(), IMU.getGyroY_rads(), IMU.getGyroZ_rads(), readImuPeriodFloat);
      accelUpdate(IMU.getAccelX_mss(), IMU.getAccelY_mss(), IMU.getAccelZ_mss(), alpha);

      readImuLast += readImuPeriod;
    }

    // TEST IMU
    static unsigned long lastPrintImu = 0;
    if (millis() - lastPrintImu > 1000) {
      //Euler currentAngles(orientation);
      //printEuler(currentAngles, 1, true);
      //Serial.println(orientation.getYaw());
      lastPrintImu += 1000;
    }

  }

} // Loop




// **************************************************************************************
// ******************************  SETUP FUNCTIONS  *************************************
// **************************************************************************************


void setupEncoders() {
  pinMode(pinEncoderIntL, INPUT);
  pinMode(pinEncoderOtherL, INPUT);
  pinMode(pinEncoderIntR, INPUT);
  pinMode(pinEncoderOtherR, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEncoderIntL), incrementCounterL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinEncoderIntR), incrementCounterR, CHANGE);
}


// **************************************************************************************
// ****************************  RECEIVE SERIAL DATA  ***********************************
// **************************************************************************************

// If a whole package is available (and it should be given the way data is transmitted over USB)
//  then this function will block until the whole package is processed
// If there are multiple then the next package will be processed the next time this function is called
void receiveRequests() {
  static uint8_t c;
  static uint8_t idx = 0;
  static bool receivingPackage = false;
  static bool receiptComplete = false;
  //  static bool parseComplete = false; // Think I can remove this
  static uint8_t packageId;
  //static uint8_t packageIdPrev;
  while ((Serial.available() > 0) && !receiptComplete) {
    c = Serial.read();
    digitalWrite(pinLED, LOW);
    if (c == '<') {
      receivingPackage = true;
      idx = 0;
      //      Serial.print("Start\n");
    }
    else if (c == '>') {
      // only want to flag as complete if previously received a start marker
      if (receivingPackage) {
        receivingPackage = false;
        receiptComplete = true;
        rxBuffer[idx] = '\0';
        //        Serial.print("Stop\n");
      }
    }

    else if (receivingPackage) {
      // handle too large packages just in case
      if (idx >= rxBufferSize) {
        // stop receiving and wait for new start marker
        receivingPackage = false;
      }
      else {
        rxBuffer[idx] = c;
        idx++;
      }
    }
  }

  if (receiptComplete) {
    //    Serial.println(rxBuffer);

    char *val = strtok(rxBuffer, "\t");
    if (val != nullptr) {
      packageType = static_cast<PackageType>(atoi(val));
      //      Serial.println(packageType);
    }
    else {
      Serial.println("Package type problem");
      receiptComplete = false;
      return;
    }

    val = strtok(nullptr, "\t");
    if (val != nullptr) {
      packageId = atoi(val);
      //      Serial.println(packageId);
    }

    // add check on package id?

    switch (packageType) {

      case PackageType::MOVE:
        val = strtok(nullptr, "\t");
        movePackageRequest.moveType = static_cast<MoveType>(atoi(val));
        val = strtok(nullptr, "\t");
        movePackageRequest.turn = atof(val);
        val = strtok(nullptr, "\t"); // Does it not matter that there isn't a \t following the final value ??
        movePackageRequest.forward = atof(val);
        //        Serial.println(movePackageRequest.moveType);
        //        Serial.println(movePackageRequest.turn);
        //        Serial.println(movePackageRequest.forward);
        newMoveRequest = true;
        break;

      case PackageType::POSE:
        // This request has no associated data
        newPoseRequest = true;
        break;

      case PackageType::SERVO:
        val = strtok(nullptr, "\t");
        servoPackageRequest.angle = atoi(val);
        newServoRequest = true;
        break;

      case PackageType::STATUS:
        val = strtok(nullptr, "\t");
        statusPackageRequest.requestedStatus = static_cast<Status>(atoi(val));
        newStatusRequest = true;
        break;

      default:
        Serial.println("Unrecognised package type");
        break;
    }
    receiptComplete = false;
  }
}


// **************************************************************************************
// ****************************  RECEIVE RADIO DATA  ************************************
// **************************************************************************************

// TODO decouple receipt and motor setting
//  // RECEIVER AND MOTORS
//  // Should the motors actually be updated here? Or decouple setting pwm/direction from actual update and do that in separate bit?
//  // Would need to save motor directions as well
void processRadioInput() {
  //      receiver.printPackage();

  // Not using this mode yet
  //      if (receiver.getControlBit(3) == 0) {
  //        mode = Mode::MANUAL;
  //      }
  //      else {
  //        mode = Mode::AUTO;
  //      }

  if (receiver.getControlBit(2) == 0) {
    robotDirection = RobotDirection::FORWARD;
  }
  else {
    robotDirection = RobotDirection::BACKWARD;
  }

  uint8_t throttle = receiver.getThrottle();
  if (abs(throttle) <= LOW_THROTTLE_LIMIT) {
    throttle = 0;
  }
  int16_t yawDifferential = (int16_t)receiver.getYaw() - STICK_CENTRE;
  if (abs(yawDifferential) <= STICK_DEADZONE) {
    yawDifferential = 0;
    turnDirection = TurnDirection::NONE;
  }
  if (yawDifferential < 0) {
    turnDirection = TurnDirection::LEFT;
  }
  else if (yawDifferential > 0) {
    turnDirection = TurnDirection::RIGHT;
  }
  float yawDifferentialPC = abs((float)yawDifferential) / (float)STICK_CENTRE;

  // TURN
  if (throttle > 0) {
    if (robotDirection == RobotDirection::FORWARD) {
      motorLeft.setDir(MotorDirection::FORWARD);
      motorRight.setDir(MotorDirection::FORWARD);
    }
    else if (robotDirection == RobotDirection::BACKWARD) {
      motorLeft.setDir(MotorDirection::BACKWARD);
      motorRight.setDir(MotorDirection::BACKWARD);
    }
    if (turnDirection == TurnDirection::NONE) {
      throttleLeft = throttle;
      throttleRight = throttle;
    }
    else if (turnDirection == TurnDirection::LEFT) {
      throttleLeft = (uint8_t)(min((float)throttle * (1.0 - yawDifferentialPC), 255.0));
      throttleRight = (uint8_t)(min((float)throttle * (1.0 + yawDifferentialPC), 255.0));
    }
    else if (turnDirection == TurnDirection::RIGHT) {
      throttleLeft = (uint8_t)(min((float)throttle * (1.0 + yawDifferentialPC), 255.0));
      throttleRight = (uint8_t)(min((float)throttle * (1.0 - yawDifferentialPC), 255.0));
    }
  }

  // PIVOT
  else if (throttle == 0 && yawDifferential != 0) {
    if (turnDirection == TurnDirection::LEFT) {
      motorLeft.setDir(MotorDirection::BACKWARD);
      motorRight.setDir(MotorDirection::FORWARD);
    }
    else if (turnDirection == TurnDirection::RIGHT) {
      motorLeft.setDir(MotorDirection::FORWARD);
      motorRight.setDir(MotorDirection::BACKWARD);
    }
    uint8_t pivotThrottle = (uint8_t)((float)THROTTLE_MAX * yawDifferentialPC);
    throttleLeft = pivotThrottle;
    throttleRight = pivotThrottle;
  }
  else {
    throttleLeft = 0;
    throttleRight = 0;
  }

  if (throttleLeft < LOW_THROTTLE_LIMIT) throttleLeft = 0;
  if (throttleRight < LOW_THROTTLE_LIMIT) throttleRight = 0;
  motorLeft.setPwmValue(throttleLeft);
  motorRight.setPwmValue(throttleRight);

  // And let the roll (right stick left/right) control the servo
  uint8_t servoInput = receiver.getRoll();
  servoPosition = map(servoInput, 0, 255, 60, 120);
  camServo.write(servoPosition);
}




// **************************************************************************************
// *******************************  SEND PACKAGES  **************************************
// **************************************************************************************


void sendMovePackage() {
  Serial.print(startByte);
  Serial.print(static_cast<int>(PackageType::MOVE));
  Serial.print(delim);
  Serial.print(sendPackageId);
  Serial.print(delim);
  Serial.print(static_cast<int>(moveStatus));
  Serial.print(delim);
  Serial.print(position.x);
  Serial.print(delim);
  Serial.print(position.y);
  Serial.print(delim);
  Serial.print(position.z);
  Serial.print(endByte);
  Serial.print('\n'); // not required - just for ease of viewing if doing so directly
  Serial.send_now();
  sendPackageId++;
}



void sendPosePackage() {
  Serial.print(startByte);
  Serial.print(static_cast<int>(PackageType::POSE));
  Serial.print(delim);
  Serial.print(sendPackageId);
  Serial.print(delim);
  Serial.print(orientation.a);
  Serial.print(delim);
  Serial.print(orientation.b);
  Serial.print(delim);
  Serial.print(orientation.c);
  Serial.print(delim);
  Serial.print(orientation.d);
  Serial.print(endByte);
  Serial.print('\n'); // not required - just for ease of viewing if doing so directly
  Serial.send_now();
  sendPackageId++;
}

void sendServoPackage() {
  Serial.print(startByte);
  Serial.print(static_cast<int>(PackageType::SERVO));
  Serial.print(delim);
  Serial.print(sendPackageId);
  Serial.print(delim);
  Serial.print(servoPackageRequest.angle);
  Serial.print(endByte);
  Serial.print('\n'); // not required - just for ease of viewing if doing so directly
  Serial.send_now();
  sendPackageId++;
}

void sendStatusPackage() {
  Serial.print(startByte);
  Serial.print(static_cast<int>(PackageType::STATUS));
  Serial.print(delim);
  Serial.print(sendPackageId);
  Serial.print(delim);
  Serial.print(static_cast<uint8_t>(status));
  Serial.print(endByte);
  Serial.print('\n'); // not required - just for ease of viewing if doing so directly
  Serial.send_now();
  sendPackageId++;
}




// **************************************************************************************
// ***********************************  SERVO  ******************************************
// **************************************************************************************

void processServoRequest() {
  if (newServoRequest) {
    // do something
    servoPosition = servoPositionCentre + servoPackageRequest.angle;
    servoPosition = constrain(servoPosition, SERVO_POS_MIN, SERVO_POS_MAX);
    camServo.write(servoPosition);
    sendServoPackage(); // TODO there should be a (ideally non-blocking) delay before this is sent
    newServoRequest = false;
  }
}


// **************************************************************************************
// ***********************************  STATUS  *****************************************
// **************************************************************************************


void processStatusRequest() {

  if (newStatusRequest) {

    // update robot status if allowed
    // TODO this should only be allowed from specific states
    // TODO when becoming active, it should erase any previously received messages of any other kind (?)
    if ((mode == Mode::AUTO)
        && (status == Status::READY)
        && (statusPackageRequest.requestedStatus == Status::ACTIVE))
    {
      status = statusPackageRequest.requestedStatus;
      sendStatusPackage();
    }
    else if ((mode == Mode::AUTO) && (statusPackageRequest.requestedStatus == Status::SHUTDOWN)) {
      status = Status::SHUTDOWN;
      sendStatusPackage();
    }
    else {
      // default to report only
      sendStatusPackage();
    }
  }
  newStatusRequest = false;
}


void handleStatusChanges() {
  if (status == Status::SHUTDOWN) {
    motorLeft.setPwmValue(0);
    motorRight.setPwmValue(0);
    //camServo.detach(); // this gives an 'undefined reference' error - not sure why
    camServo.write(90);
    radioArmed = false;
  }
}


// **************************************************************************************
// **********************************  MOVEMENT  ****************************************
// **************************************************************************************


void processMoveRequest() {
  if (newMoveRequest) {
    // ADD HANDLING OF REQUESTS BEFORE THIS ONE FINISHED
    // although at the moment I will assume that won't happen

    // change move status
    moveStatus = MoveStatus::MOVING;

    // set targets
    if(movePackageRequest.moveType == MoveType::ABS_TURN_AND_STRAIGHT) {
      headingSetpoint = wrapAngle(movePackageRequest.turn);
    }
    else if (movePackageRequest.moveType == MoveType::REL_TURN_AND_STRAIGHT) {
      headingSetpoint = wrapAngle(orientation.getYaw() + movePackageRequest.turn);
    }
    forwardTarget = movePackageRequest.forward;

    // clear previous movement information
    forwardActual = 0.0;
    position = Vector(0, 0, 0);

    newMoveRequest = false;
  }
}


void handleMovement() {
  static uint32_t lastUpdate;
  // handle state changes
  if (moveStatus != moveStatusPrev) {
    // moveStatus has just been updated, robot should either start or finish a movement
    moveStatusPrev = moveStatus; // save
    if (moveStatus == MoveStatus::MOVING) {
      // send initial response with zero movement
      lastUpdate = millis();
      sendMovePackage();
      sendPosePackage();

      // we've just started moving, TURN is first
      moveStatusDetail = MoveStatusDetail::TURN;
      // enable the relevant PIDs
      pidHeading.SetMode(PID_AUTOMATIC);
      pidLeftTrackSpeed.SetMode(PID_AUTOMATIC);
      pidRightTrackSpeed.SetMode(PID_AUTOMATIC);
    }
    else {
      // moveStatus == MoveStatus::FINISHED
      // we've just finished moving
      // turn off PIDs
      leftTrackSpeedSetpoint = 0;
      rightTrackSpeedSetpoint = 0;
      motorLeft.setPwmValue(0);
      motorRight.setPwmValue(0);
      pidHeading.SetMode(PID_MANUAL);
      pidLeftTrackSpeed.SetMode(PID_MANUAL);
      pidRightTrackSpeed.SetMode(PID_MANUAL);

      // send final response with final movement
      sendMovePackage();
      sendPosePackage();
    }
  }

  // run the functions that actually control the speeds
  // maybe these should move to separate bit
  if (moveStatus == MoveStatus::MOVING) {
    if (moveStatusDetail == MoveStatusDetail::TURN) {
      movePivot();
    }

    if (moveStatusDetail == MoveStatusDetail::FORWARD) {
      moveForward();
    }

    // Tperiodic sending of move package while movement is happening
    if (millis() - lastUpdate > moveUpdatePeriod) {
      sendMovePackage();
      sendPosePackage();
      lastUpdate += moveUpdatePeriod;
    }
  }
}


// although there may be some duplication in calculating the distance travelled here
//    as well as elsewhere (for calculating the movement in pose direction)
//    it's not a big overhead and it makes it simpler to have them decoupled
void calculateTrackSpeeds() {
  static uint32_t lastRead = 0;
  static int32_t lastCountLeft = 0;
  static int32_t lastCountRight = 0;
  uint32_t timeNow = millis();
  int32_t encCountLeftCopy = encCountLeft;  // take a copy since these variables will change via the ISR
  int32_t encCountRightCopy = encCountRight;
  int32_t countDiffLeft = encCountLeftCopy - lastCountLeft;
  int32_t countDiffRight = encCountRightCopy - lastCountRight;
  uint32_t timeDiff = (timeNow - lastRead); // time difference in milliseconds
  trackSpeedLeft = (distancePerPulse * 1000 * (float)countDiffLeft) / (float)timeDiff; // speed in millimetres per second
  trackSpeedRight = (distancePerPulse * 1000 * (float)countDiffRight) / (float)timeDiff;

  lastRead = timeNow;
  lastCountLeft = encCountLeftCopy;
  lastCountRight = encCountRightCopy;
}




// TODO consider that could probably merge movePivot and moveForward
//    the only different being the base speed in movePivot will be zero
void movePivot() {
  //  Serial.println("In movePivot()");
  heading = orientation.getYaw();
  headingError = headingSetpoint - heading; // TODO ensure that heading is updated before this function called
  headingError = wrapAngle(headingError);

  if (checkTargets()) {
    // update status
    moveStatusDetail = MoveStatusDetail::FORWARD;
    // TODO should PIDs be turned off and on as well?
    leftTrackSpeedSetpoint = 0;
    rightTrackSpeedSetpoint = 0;
    motorLeft.setPwmValue(0);
    motorRight.setPwmValue(0);
  }
  else {
    pidHeading.Compute(); // at the moment the output of this should always be 0
    const float baseSpeed = 0.0; // TODO move elsewhere
    calculateRequiredTrackSpeeds(baseSpeed, requiredAngularVelocity, leftTrackSpeedSetpoint, rightTrackSpeedSetpoint);
  }
}



void moveForward() {
  //  Serial.println("In moveForward()");
  heading = orientation.getYaw();
  headingError = headingSetpoint - heading; // TODO ensure that heading is updated before this function called
  headingError = wrapAngle(headingError);

  if (checkTargets()) {
    // update status
    moveStatusDetail = MoveStatusDetail::NONE;
    moveStatus = MoveStatus::FINISHED;
  }
  else {
    pidHeading.Compute();
    float baseSpeed = 100.0; // TODO move elsewhere
    if ((forwardTarget - forwardActual) < 0) {
      baseSpeed *= -1;
    }
    calculateRequiredTrackSpeeds(baseSpeed, requiredAngularVelocity, leftTrackSpeedSetpoint, rightTrackSpeedSetpoint);
  }
}

// TODO reconsider the args since they are globals anyway
void calculateRequiredTrackSpeeds(const float v, const float w, float& vLeft, float& vRight) {
  vLeft = (2.0 * v - w * distanceBetweenTracks) / 2.0;
  vRight = (2.0 * v + w * distanceBetweenTracks) / 2.0;
}


float wrapAngle(float angle) {
  // the 'whiles' will deal with angle being more than 2pi outside range
  while (angle > PI) {
    angle -= (2 * PI);
  }
  while (angle < -PI) {
    angle += (2 * PI);
  }
  return angle;
}


void accumulateMovement() {
  // Use current pose and odometry information to update x, y, z
  static int32_t lastCountLeft = 0;
  static int32_t lastCountRight = 0;
  int32_t encCountLeftCopy = encCountLeft;  // take a copy since these variables will change via the ISR
  int32_t encCountRightCopy = encCountRight;
  int32_t countDiffLeft = encCountLeftCopy - lastCountLeft;
  int32_t countDiffRight = encCountRightCopy - lastCountRight;
  float trackDistLeft = distancePerPulse * (float)countDiffLeft; // speed in millimetres per second
  float trackDistRight = distancePerPulse * (float)countDiffRight;
  float centreDist = (trackDistLeft + trackDistRight) / 2.0;

  // apply the distance in the direction the robot is pointing
  Vector travel(centreDist, 0, 0);
  Vector travelRot = travel.rotate(orientation);
  position = position + travelRot;
  forwardActual += centreDist;

  lastCountLeft = encCountLeftCopy;
  lastCountRight = encCountRightCopy;
}






bool checkTargets() {
  if (moveStatusDetail == MoveStatusDetail::TURN) {
    // is current heading close enough to target
    // current heading must have been updated
    if (abs(heading - headingSetpoint) < headingTolerance) {
      Serial.println("Turn target reached");
      return true;
    }
  }

  if (moveStatusDetail == MoveStatusDetail::FORWARD) {
    // is current heading close enough to target
    // current heading must have been updated
    //    Serial.print(forwardTarget);
    //    Serial.print("\t");
    //    Serial.print(forwardActual);
    //    Serial.print("\t");
    //    Serial.print(forwardTarget - forwardActual);
    //    Serial.print("\t");
    //    Serial.print(forwardTolerance);
    //    Serial.print("\n");
    if (abs(forwardTarget - forwardActual) < forwardTolerance) {
      //Serial.println("Forward target reached");
      //      Serial.print(forwardTarget);
      //      Serial.print("\t");
      //      Serial.print(forwardActual);
      //      Serial.print("\t");
      //      Serial.print(abs(forwardTarget - forwardActual));
      //      Serial.print("\t");
      //      Serial.print(forwardTolerance);
      //      Serial.print("\n");
      return true;
    }
  }

  return false;
}


void updateMotorsWithPidOutputs() {
  int16_t newPwmLeft = (int16_t)motorLeft.getPwmValue();
  if (motorLeft.getDirection() == MotorDirection::BACKWARD) {
    newPwmLeft *= -1;
  }
  newPwmLeft += (int16_t)leftTrackSpeedOutput;

  int16_t newPwmRight = (int16_t)motorRight.getPwmValue();
  if (motorRight.getDirection() == MotorDirection::BACKWARD) {
    newPwmRight *= -1;
  }
  newPwmRight += (int16_t)rightTrackSpeedOutput;

  motorLeft.setDirAndPwmValue(newPwmLeft);
  motorRight.setDirAndPwmValue(newPwmRight);

  //  Serial.print(trackSpeedLeft); Serial.print('\t');
  //  Serial.print(leftTrackSpeedSetpoint); Serial.print('\t');
  //  Serial.print(leftTrackSpeedOutput); Serial.print('\t');
  //  Serial.print(newPwmLeft); Serial.print('\t');
  //  Serial.print(motorLeft.getPwmValue()); Serial.print('\n');

  //  Serial.print(trackSpeedLeft); Serial.print('\t');
  //  Serial.print(trackSpeedRight); Serial.print('\t');
  //  Serial.print(leftTrackSpeedSetpoint); Serial.print('\t');
  //  Serial.print(rightTrackSpeedSetpoint); Serial.print('\t');
  //  Serial.print(leftTrackSpeedOutput); Serial.print('\t');
  //  Serial.print(rightTrackSpeedOutput); Serial.print('\t');
  //  Serial.print(newPwmLeft); Serial.print('\t');
  //  Serial.print(newPwmRight); Serial.print('\t');
  //  Serial.print(motorLeft.getPwmValue()); Serial.print('\t');
  //  Serial.print(motorRight.getPwmValue()); Serial.print('\n');



}



// **************************************************************************************
// **********************************    IMU    *****************************************
// **************************************************************************************

void processPoseRequest() {
  if (newPoseRequest) {
    sendPosePackage();
    newPoseRequest = false;
  }
}


void gyroUpdate(float gx, float gy, float gz, float dt) {
  // based on ee269_notes_imu.pdf
  // although there are quite a few other (different) approaches

  // create angle and axis from gyro measurements
  float norm = sqrt(gx * gx + gy * gy + gz * gz);
  float angle = norm * dt;
  Vector v;
  v.x = gx / norm;
  v.y = gy / norm;
  v.z = gz / norm;
  Quaternion delta(angle, v);

  orientation = orientation * delta;
  orientation = orientation.normalise(); // to prevent a slow deviation due to small rounding errors
}



void accelUpdate(float ax, float ay, float az, float alpha) {
  // based on ee269_notes_imu.pdf
  // although there are quite a few other (different) approaches

  // note gravity vector g [0, 0, -1]

  // create vector from accelerometer measurements
  float norm = sqrt(ax * ax + ay * ay + az * az);
  float invNorm = 1.0 / norm;
  Vector vb(ax * invNorm, ay * invNorm, az * invNorm);

  // transform towards gravity vector using current orientation estimate
  Vector vw = vb.rotate(orientation);

  // find the angle and axis required transform this into exactly the gravity vector g (in inertial frame)
  // cross product of vw and g (note that multiple terms are zero so it reduces to the simple parts below)
  Vector vc(-vw.y, vw.x, 0.0);
  vc = vc.normalise();

  // vw dot product g = norm(vw)*norm(g)*cos(theta)   // norms are both 0
  // cos(theta) = -vw.z

  // make sure that it doesn't creep over the valid input range
  if (vw.z < -1.0) {
    //    Serial.println(vw.z, 7);
    vw.z = -1.0;
  }
  else if (vw.z > 1.0) {
    //    Serial.println(vw.z, 7);
    vw.z = 1.0;
  }
  float theta = acos(-vw.z); // add handling for over 90 degrees?
  //Serial.println(-vw.z, 7);

  // TODO what if rotation is going the 'long' way around??
  //  if(theta<0){
  //    vc = -vc;
  //    theta = -theta;
  //  }

  theta *= alpha; // how much of the correction we want to apply
  Quaternion correction(theta, vc);

  orientation = correction * orientation;
  orientation = orientation.normalise(); // to prevent a slow deviation due to small rounding errors
}



void calculateInitialOrientation() {
  // (a lot of the mag samples will be repeated)
  float axTrack = 0, ayTrack = 0, azTrack = 0;
  float mxTrack = 0, myTrack = 0, mzTrack = 0;
  int count = 1000;
  for (int i = 0; i < count; i++) {
    IMU.readSensor(true, true, true);
    axTrack += IMU.getAccelX_mss();
    ayTrack += IMU.getAccelY_mss();
    azTrack += IMU.getAccelZ_mss();
    mxTrack += IMU.getMagX_uT();
    myTrack += IMU.getMagY_uT();
    mzTrack += IMU.getMagZ_uT();
    delay(readImuPeriod);
  }

  axTrack /= (float)count;
  ayTrack /= (float)count;
  azTrack /= (float)count;
  mxTrack /= (float)count;
  myTrack /= (float)count;
  mzTrack /= (float)count;

  calculateRollAndPitch(axTrack, ayTrack, azTrack);
  calculateYaw(mxTrack, myTrack, mzTrack);

  orientation = Quaternion(eulerAngles);

}


// from 'resources/AN3461.pdf'
// Note that the results are Euler angles in the order PITCH then ROLL
// No yaw
void calculateRollAndPitch(float ax, float ay, float az) {
  // first normalise accel data (this can be moved elsewhere later if necessary)
  float length = sqrt(ax * ax + ay * ay + az * az);
  float axn = ax / length;
  float ayn = ay / length;
  float azn = az / length;

  // not certain why this is required - the data is already transformed into an NED like frame
  // TODO investigate
  azn = -azn;


  // roll
  eulerAngles.roll = atan2(ayn, azn);

  // pitch
  // can add a small component of y to denom
  float denom = sqrt(ayn * ayn + azn * azn);
  eulerAngles.pitch = atan2(-axn, denom);

  // yaw
  eulerAngles.yaw = 0.0;
}


void calculateYaw(float mx, float my, float mz) {
  // normalise
  float length = sqrt(mx * mx + my * my + mz * mz);
  float mxn = mx / length;
  float myn = my / length;
  //float mzn = mz / length;

  // TODO check this and uncomment
  // adjust for current pitch and roll
  //  Euler e = eulerAngles; // for brevity
  //  float mxnLevel = mxn * cos(e.roll) + myn * sin(e.roll) * cos(e.pitch) - mzn * cos(e.roll) * sin(e.pitch);
  //  float mynLevel = myn * cos(e.roll) + mzn * sin(e.roll);
  //  eulerAngles.yaw = atan2(mynLevel, mxnLevel);

  eulerAngles.yaw = atan2(myn, mxn);
  // TODO fix issue with magnetometer (hardware?)
  // until then, will set heading to be zero
  eulerAngles.yaw = 0;

}


void initialiseMagFilters() {
  for (int i = 0; i < movingAverageFilter::size; i++) {
    IMU.readSensor(true, true, true);
    magXfilter.update(IMU.getMagX_uT());
    magYfilter.update(IMU.getMagY_uT());
    magZfilter.update(IMU.getMagZ_uT());
    delay(readImuPeriod);
  }
}


void printQuaternion(const Quaternion& q, int precision) {
  Serial.print(q.a, precision);
  Serial.print("\t");
  Serial.print(q.b, precision);
  Serial.print("\t");
  Serial.print(q.c, precision);
  Serial.print("\t");
  Serial.print(q.d, precision);
  Serial.print("\n");
}



void printEuler(const Euler& e, int precision, bool inDegrees) {
  float fact = 1.0;
  if (inDegrees) {
    fact *= (180.0 / PI);
  }
  Serial.print(e.yaw * fact, precision);
  Serial.print('\t');
  Serial.print(e.pitch * fact, precision);
  Serial.print('\t');
  Serial.print(e.roll * fact, precision);
  Serial.print('\n');
}

void printVector(const Vector& v, int precision) {
  Serial.print(v.x, precision);
  Serial.print('\t');
  Serial.print(v.y, precision);
  Serial.print('\t');
  Serial.print(v.z, precision);
  Serial.print('\n');
}

void imuPrintAllData() {
  Serial.print(IMU.getAccelX_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(), 6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(), 6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(), 6);
}

void imuPrintMagData() {
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(), 6);
  Serial.print("\t");
  Serial.print(atan2(IMU.getMagY_uT(), IMU.getMagX_uT()) * 180 / PI, 6);
  Serial.print("\t");
  Serial.print(atan2(magYfilter.output, magXfilter.output) * 180 / PI, 6);
  Serial.print("\n");
}
