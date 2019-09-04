// TODO change delays into waits for serial input so it can be triggered

// results from calibration with full robot but no batteries present
// 52.7316322  -36.4627380 -24.8827744
// 0.8228859 1.1055350 1.1360730


// Library: https://github.com/bolderflight/MPU9250
#include "MPU9250.h"

MPU9250 IMU(SPI1, 4);
int imuStatus;


void imuReadAndPrint() {
  IMU.readSensor();
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(), 6);
  Serial.print("\n");
}


void setup() {
  while (!Serial);


  // IMU alternative SPI pins
  SPI1.setSCK(20);
  SPI1.setMISO(5);
  SPI1.setMOSI(21);


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
  Serial.println("Start moving in figure 8");
  Serial.println("3");
  delay(1000);
  Serial.println("2");
  delay(1000);
  Serial.println("1");
  delay(1000);

  res = IMU.calibrateMag();
  if (res < 1) {
    Serial.println("Calibration error");
    while (1);
  }


  Serial.println("Calibration finished");
  Serial.println(res);
  Serial.print(IMU.getMagBiasX_uT(), 7);
  Serial.print('\t');
  Serial.print(IMU.getMagBiasY_uT(), 7);
  Serial.print('\t');
  Serial.print(IMU.getMagBiasZ_uT(), 7);
  Serial.print('\n');
  Serial.print(IMU.getMagScaleFactorX(), 7);
  Serial.print('\t');
  Serial.print(IMU.getMagScaleFactorY(), 7);
  Serial.print('\t');
  Serial.print(IMU.getMagScaleFactorZ(), 7);
  Serial.print('\n');
  Serial.print('\n');
  Serial.println("Calibrated readings below");
}


void loop() {



  imuReadAndPrint();
  delay(1000);

} // Loop
