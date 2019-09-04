// TODO change delays into waits for serial input so it can be triggered

// results
//0.2534242  0.2825031 0.5199513
//0.9977166 0.9991337 0.9813061


// Library: https://github.com/bolderflight/MPU9250
#include "MPU9250.h"

MPU9250 IMU(SPI, 15);
int imuStatus;


void imuReadAndPrint() {
  IMU.readSensor();
  Serial.print(IMU.getAccelX_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(), 6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(), 6);
  Serial.print("\n");
}


void setup() {
  while (!Serial);

  imuStatus = IMU.begin();
  if (imuStatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(imuStatus);
    while (1) {}
  }
  Serial.println(F("Starting..."));

  int res;
  Serial.println("Z up");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  res = IMU.calibrateAccel();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }

  Serial.println("Z down");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  res = IMU.calibrateAccel();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }

  Serial.println("Y up");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  res = IMU.calibrateAccel();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }

  Serial.println("Y down");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  res = IMU.calibrateAccel();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }

  Serial.println("X up");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  res = IMU.calibrateAccel();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }

  Serial.println("X down");
  Serial.println("5");
  delay(1000);
  Serial.println("4");
  delay(1000);
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);
  res = IMU.calibrateAccel();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }


  Serial.println("Calibration finished");
  Serial.println(res);
  Serial.print(IMU.getAccelBiasX_mss(), 7);
  Serial.print('\t');
  Serial.print(IMU.getAccelBiasY_mss(), 7);
  Serial.print('\t');
  Serial.print(IMU.getAccelBiasZ_mss(), 7);
  Serial.print('\n');
  Serial.print(IMU.getAccelScaleFactorX(), 7);
  Serial.print('\t');
  Serial.print(IMU.getAccelScaleFactorY(), 7);
  Serial.print('\t');
  Serial.print(IMU.getAccelScaleFactorZ(), 7);
  Serial.print('\n');
  Serial.print('\n');
  Serial.println("Calibrated readings below");
}


void loop() {



  imuReadAndPrint();
  delay(1000);

} // Loop
