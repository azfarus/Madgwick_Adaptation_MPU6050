/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include <MPU6050_light.h>
#define deg_to_rad (3.1415/180)


MPU6050 mpu(Wire);

float timer = 0;


volatile float beta = .01;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

struct Quaternion {
    double w, x, y, z;
};

Quaternion multiplyQuaternions(const Quaternion& qa, const Quaternion& qb) {
    Quaternion result;
    result.w = qa.w * qb.w - qa.x * qb.x - qa.y * qb.y - qa.z * qb.z;
    result.x = qa.w * qb.x + qa.x * qb.w + qa.y * qb.z - qa.z * qb.y;
    result.y = qa.w * qb.y - qa.x * qb.z + qa.y * qb.w + qa.z * qb.x;
    result.z = qa.w * qb.z + qa.x * qb.y - qa.y * qb.x + qa.z * qb.w;
    return result;
}


void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az , float del) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
  del = del * 1e-6;
	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * del;
	q1 += qDot2 * del;
	q2 += qDot3 * del;
	q3 += qDot4 * del;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}


void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.upsideDownMounting = true;
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,false); // gyro and accelero
  Serial.println("Done!\n");
  timer = micros();
}

void printQ(Quaternion axis){
  Serial.print(axis.w);
  Serial.print(" ");
  Serial.print(axis.x);
  Serial.print(" ");
  Serial.print(axis.y);
  Serial.print(" ");
  Serial.print(axis.z);
  Serial.print("\n");
}

void loop() {
  mpu.update();

  MadgwickAHRSupdateIMU(mpu.getGyroX()*deg_to_rad,mpu.getGyroY()*deg_to_rad,mpu.getGyroZ()*deg_to_rad,mpu.getAccX(),mpu.getAccY(),mpu.getAccZ(),micros()-timer);
  timer = micros();
  
  // if(millis() - timer > 1000){ // print data every second
  //   Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
  //   Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
  //   Serial.print("\tY: ");Serial.print(mpu.getAccY());
  //   Serial.print("\tZ: ");Serial.println(mpu.getAccZ());
  
  //   Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
  //   Serial.print("\tY: ");Serial.print(mpu.getGyroY());
  //   Serial.print("\tZ: ");Serial.println(mpu.getGyroZ());
  
  //   Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
  //   Serial.print("\tY: ");Serial.println(mpu.getAccAngleY());
    
  //   Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
  //   Serial.print("\tY: ");Serial.print(mpu.getAngleY());
  //   Serial.print("\tZ: ");Serial.println(mpu.getAngleZ());
  //   Serial.println(F("=====================================================\n"));
  //   timer = millis();
  // }

  Quaternion main , main_inv , axis;
  main.w = q0;
  main.x = q1;
  main.y = q2;
  main.z = q3;

  axis.w = 0;
  axis.x = 0;
  axis.w = 0;
  axis.z = 1;

  main_inv.w = q0;
  main_inv.x = -q1;
  main_inv.y = -q2;
  main_inv.z = -q3;

  axis = multiplyQuaternions(main , axis);
  axis = multiplyQuaternions(axis , main_inv);
  float recip = invSqrt(axis.x*axis.x + axis.y*axis.y +axis.z*axis.z);
  const float rad_deg = 180/3.1415;
  Serial.print(acos(axis.x*recip)*rad_deg );
  Serial.print(" ");
  Serial.print(acos(axis.y*recip)*rad_deg);
  Serial.print(" ");
  Serial.print(acos(axis.z*recip)*rad_deg);
  Serial.print("\n");
}
