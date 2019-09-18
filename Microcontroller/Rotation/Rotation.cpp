#include "Rotation.h"
#include "math.h"

Quaternion::Quaternion() {
  a = 1.0f;
  b = 0.0f;
  c = 0.0f;
  d = 0.0f;
}

Quaternion::Quaternion(const Quaternion& quat) {
  a = quat.a;
  b = quat.b;
  c = quat.c;
  d = quat.d;
}

Quaternion::Quaternion(const Vector& v) {
  a = 0;
  b = v.x;
  c = v.y;
  d = v.z;
}

// Vector must (should?) be normalised
Quaternion::Quaternion(float angle, const Vector& v) {
  a = cos(angle / 2);
  b = v.x * sin(angle / 2);
  c = v.y * sin(angle / 2);
  d = v.z * sin(angle / 2);
}

Quaternion::Quaternion(const float scalar, const float i, const float j, const float k) {
  a = scalar;
  b = i;
  c = j;
  d = k;
}

// taken from here: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// or here: http://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
Quaternion::Quaternion(const Euler& e) {
  float cy = cos(e.yaw * 0.5);
  float sy = sin(e.yaw * 0.5);
  float cr = cos(e.roll * 0.5);
  float sr = sin(e.roll * 0.5);
  float cp = cos(e.pitch * 0.5);
  float sp = sin(e.pitch * 0.5);

  a = cy * cr * cp + sy * sr * sp;
  b = cy * sr * cp - sy * cr * sp;
  c = cy * cr * sp + sy * sr * cp;
  d = sy * cr * cp - cy * sr * sp;
}


Quaternion Quaternion::operator+(const Quaternion& q2) const {
  return Quaternion(a + q2.a , b + q2.b, c + q2.c, d + q2.d);
}

Quaternion Quaternion::operator-(const Quaternion& q2) const {
  return Quaternion(a - q2.a , b - q2.b, c - q2.c, d - q2.d);
}

Quaternion Quaternion::operator-() const {
  return Quaternion(-a, -b, -c, -d);
}

Quaternion Quaternion::operator*(const Quaternion& q2) const {
  Quaternion result;
  result.a = a * q2.a - b * q2.b - c * q2.c - d * q2.d;
  result.b = a * q2.b + b * q2.a + c * q2.d - d * q2.c;
  result.c = a * q2.c - b * q2.d + c * q2.a + d * q2.b;
  result.d = a * q2.d + b * q2.c - c * q2.b + d * q2.a;
  return result;
}

Quaternion Quaternion::conjugate() const {
  return Quaternion(a, -b, -c, -d);
}

Quaternion Quaternion::operator+(const float scalar) const {
  return Quaternion(a + scalar, b, c, d);
}

Quaternion Quaternion::operator-(const float scalar) const {
  return Quaternion(a - scalar, b, c, d);
}

Quaternion Quaternion::operator*(const float scalar) const {
  return Quaternion(a * scalar, b * scalar, c * scalar, d * scalar);
}

Quaternion Quaternion::operator/(const float scalar) const {
  Quaternion result;
  result.a = a / scalar;
  result.b = b / scalar;
  result.c = c / scalar;
  result.d = d / scalar;
  return result;
}

float Quaternion::norm() const {
  return sqrt(a * a + b * b + c * c + d * d);
}

Quaternion Quaternion::normalise() const {
  return Quaternion(*this / norm());
}

// assumes Euler rotation order of yaw, pitch, roll
float Quaternion::getRoll() const {
  return atan2(2 * (a * b + c * d), a * a - b * b - c * c + d * d);
}

float Quaternion::getPitch() const {
  return -asin(2 * (b * d - a * c));
}

float Quaternion::getYaw() const {
  return atan2(2 * (a * d + b * c), a * a + b * b - c * c - d * d);
}

float Quaternion::dotProduct(const Quaternion q2) const {
  return a * q2.a + b * q2.b + c * q2.c + d * q2.d;
}

// assumes that both quaternions are unit quaterions i.e. norm = 1
// TODO check this is correct (not sure about the angle calculation)
float Quaternion::angleBetween(const Quaternion& q2) const {
  //  return 2 * acos((this->dotProduct(q2)) / (this->norm() * q2.norm()));
  return 2 * acos(this->dotProduct(q2));
}

// TODO check this is correct (not sure about the angle calculation)
Quaternion Quaternion::slerp(const Quaternion& q2, float ratio) const {
  float dotP = dotProduct(q2);
  float theta = 0.5 * 2 * acos(dotP);
  Quaternion q1Temp = (*this) * sin((1 - ratio) * theta);
  Quaternion q2Temp;
  if (dotP < 0) {
    q2Temp = -q2 * sin(ratio * theta);
  }
  else {
    q2Temp = q2 * sin(ratio * theta);
  }
  return (q1Temp + q2Temp ) / sin(theta);
}


Vector::Vector() {
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vector::Vector(float xin, float yin, float zin) {
  x = xin;
  y = yin;
  z = zin;
}

Vector::Vector(const Quaternion& q) {
  x = q.b;
  y = q.c;
  z = q.d;
}

Vector::Vector(const Vector& v) {
  x = v.x;
  y = v.y;
  z = v.z;
}

Vector Vector::rotate(Quaternion& q) {
  Quaternion vectorAsQuat(*this);
  Quaternion qResult = q * vectorAsQuat * q.conjugate();
  return Vector(qResult);
}

Vector Vector::operator-() const {
  return Vector(-x, -y, -z);
}

Vector Vector::operator+(const Vector& v) const {
  return Vector(v.x + x, v.y + y, v.z + z);
}


Vector Vector::operator/(const float scalar) const {
  return Vector(x/scalar, y/scalar, z/scalar);
}

float Vector::norm() const {
  return sqrt(x * x + y * y + z * z);
}

Vector Vector::normalise() const {
  return Vector(*this / norm());
}


Euler::Euler() {
  roll = 0;
  pitch = 0;
  yaw = 0;
}

Euler::Euler(float rollIn, float pitchIn, float yawIn, bool inRadians) {
  if (inRadians) {
    roll = rollIn;
    pitch = pitchIn;
    yaw = yawIn;
  }
  else {
    roll = rollIn / (180 / M_PI);
    pitch = pitchIn / (180 / M_PI);
    yaw = yawIn / (180 / M_PI);
  }
}

// from here: http://www.chrobotics.com/library/understanding-quaternions
Euler::Euler(const Quaternion& q) {
  roll = atan2(2 * (q.a * q.b + q.c * q.d) , (q.a * q.a - q.b * q.b - q.c * q.c + q.d * q.d));
  pitch = -asin(2 * (q.b * q.d - q.a * q.c));
  yaw = atan2(2 * (q.a * q.d + q.b * q.c) , (q.a * q.a + q.b * q.b - q.c * q.c - q.d * q.d));
}

Euler::Euler(const Euler& e) {
  roll = e.roll;
  pitch = e.pitch;
  yaw = e.yaw;
}


// NON-MEMBER VERSION OF SOME FUNCTIONS
float dotProduct(const Quaternion& q1, const Quaternion q2) {
  return q1.a * q2.a + q1.b * q2.b + q1.c * q2.c + q1.d * q2.d;
}

// TODO check this is correct (not sure about the angle calculation)
float angleBetween(const Quaternion& q1, const Quaternion& q2) {
  return 2 * acos(dotProduct(q1, q2));
}

// TODO check this is correct (not sure about the angle calculation)
Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float ratio) {
  // not using angleBetween directly because want to check result of dotProduct
  float dotP = dotProduct(q1, q2);
  float theta = 0.5 * 2 * acos(dotP);
  Quaternion q1Temp = q1 * sin((1 - ratio) * theta);
  Quaternion q2Temp;
  if (dotP < 0) {
    q2Temp = -q2 * sin(ratio * theta);
  }
  else {
    q2Temp = q2 * sin(ratio * theta);
  }
  return (q1Temp + q2Temp ) / sin(theta);
}
