// make all rotation objects and their methods const?
// need to test slerp

#include "Rotation.h"

void setup() {
  Serial.begin(230400);
  delay(4000);

  test();
}

void loop() {}

void test() {
  // initialise some Euler angles
  Euler e0(0.97738, 1.8675, -0.50615);
  printEulerDegrees(e0);

  // initialise some Vectors
  Vector v0(1, 0, 0); // x axis
  Vector v1(0, 1, 0); // y axis
  Vector v2(0, 0, 1); // z axis

  // initialise some quaternions
  Quaternion q0;  // default
  printQuat(q0);
  Quaternion q1(1, 2, 3, 4); // setting elements directly
  printQuat(q1);
  Quaternion q2(0.7854, v2); // rotation around a vector
  printQuat(q2);
  Quaternion q3(Vector(0, 0, 1)); // vector as quaternion
  printQuat(q3);
  Quaternion q4(e0); // inialise from euler angles
  printQuat(q4);

  // and another Euler object from a quaternion
  Euler e1(q2);
  printEulerDegrees(e1);

  // Operations
  Quaternion result;
  // Quaternion operations
  result = q2 + q3; // addition
  printQuat(result);
  result = q2 * q3; // multiplication
  printQuat(result);
  result = q3 * q2; // multiplication (the opposite way)
  printQuat(result);
  result = result.normalise();
  printQuat(result);
  float n = q1.norm();
  Serial.println(n);
  result = q1.normalise();
  printQuat(result);
  result = q2.conjugate();
  printQuat(result);

  // scalar operations
  result = q2 + 2.5; // addition
  printQuat(result);
  result = q2 - 2.5; // subtraction
  printQuat(result);
  result = q2 * 2.5; // multiplication
  printQuat(result);
  result = q2 / 2.5; // division
  printQuat(result);

  // rotation of a vector by a quaternion
  Quaternion q5 = q1.normalise();
  Vector vResult = v1.rotate(q5);
  Serial.print(vResult.x, 3); Serial.print('\t');
  Serial.print(vResult.y, 3); Serial.print('\t');
  Serial.print(vResult.z, 3); Serial.print('\n');

  // angle between quaternions
  Quaternion q6(0.7854, v0);
  Quaternion q7(1.5708, v0);
  Serial.print(angleBetween(q6, q7), 3); Serial.print('\n');
  Quaternion q8(0.7854, v1);
  Quaternion q9(1.5708, v2);
  Serial.print(angleBetween(q8, q9), 3); Serial.print('\n');
  Quaternion q10(1, 0, 0, 0);
  Quaternion q11(1.5708, v1);
  Serial.print(angleBetween(q10, q11), 3); Serial.print('\n');

  // SLERP
  Quaternion slerpResult;
  slerpResult = slerp(q10, q11, 0.333);
  printQuat(slerpResult);
  slerpResult = slerp(q10, q11, 0.5);
  printQuat(slerpResult);
  slerpResult = slerp(q10, q11, 0.666);
  printQuat(slerpResult);

  slerpResult = slerp(q8, q9, 0.333);
  printQuat(slerpResult);
  Serial.print(slerpResult.getRoll()); Serial.print('\t');
  Serial.print(slerpResult.getPitch()); Serial.print('\t');
  Serial.print(slerpResult.getYaw()); Serial.print('\n');
}

// print operations
void printQuat(Quaternion q) {
  Serial.print(q.a, 3); Serial.print('\t');
  Serial.print(q.b, 3); Serial.print('\t');
  Serial.print(q.c, 3); Serial.print('\t');
  Serial.print(q.d, 3); Serial.print('\n');
}

void printEuler(Euler e) {
  Serial.print(e.roll, 3); Serial.print('\t');
  Serial.print(e.pitch, 3); Serial.print('\t');
  Serial.print(e.yaw, 3); Serial.print('\n');
}

void printEulerDegrees(Euler e) {
  Serial.print(e.roll * 180 / M_PI, 3); Serial.print('\t');
  Serial.print(e.pitch * 180 / M_PI, 3); Serial.print('\t');
  Serial.print(e.yaw * 180 / M_PI, 3); Serial.print('\n');
}


