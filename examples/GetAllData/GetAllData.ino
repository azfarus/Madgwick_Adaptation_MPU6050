/* Get all possible data from MPU6050
 * Accelerometer values are given as multiple of the gravity [1g = 9.81 m/s²]
 * Gyro values are given in deg/s
 * Angles are given in degrees
 * Note that X and Y are tilt angles and not pitch/roll.
 *
 * License: MIT
 */

#include "Wire.h"
#include "MPU6050_light_custom.h"

MPU6050 mpu(Wire);

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  mpu.upsideDownMounting = true;
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  mpu.setBeta(4);
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,false); // gyro and accelero
  Serial.println("Done!\n");
  timer = micros();
}

void loop() {


  mpu.MadgwickUpdate((float)(micros() - timer));
  timer = micros();
  Quaternion axis(0,0,0,1);
  axis = Quaternion::multiply(mpu.getQuaternion(), axis);
  axis = Quaternion::multiply(axis,mpu.getQuaternion().getInverse());
  float recip = 1.0/sqrt(axis.x*axis.x + axis.z*axis.z + axis.y*axis.y);

  float rad_deg = (180.0/3.1415);
  Serial.printf("%f %f %f \n" , acos(axis.x/recip)*rad_deg , acos(axis.y/recip)*rad_deg , acos(axis.z/recip)*rad_deg);
  //Serial.printf("%f %f %f \n" , mpu.getAccX() , mpu.getAccY() , mpu.getAccZ());
  
  
}
