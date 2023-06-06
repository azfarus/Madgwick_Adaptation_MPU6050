  /* Considers the vector [0 0 1]  and shows the angles with x y and z axes with the vector
 */

#include "Wire.h"
#include "MPU6050_light_custom.h"

MPU6050 mpu(Wire);

unsigned long timer = 0;

void setup() {
  Serial.begin(115200);
 
  Wire.begin();
  mpu.upsideDownMounting = true;
  mpu.setBeta(4);
  
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

void loop() {


  mpu.MadgwickUpdate((float)(micros() - timer));
  timer = micros();
  Quaternion axis(0,0,0,1);
  axis = Quaternion::multiply(mpu.getQuaternion(), axis);       
  axis = Quaternion::multiply(axis,mpu.getQuaternion().getInverse()); // execute the p = qpq^-1 quaternion multiplication  
  float recip = 1.0/sqrt(axis.x*axis.x + axis.z*axis.z + axis.y*axis.y);

  float rad_deg = (180.0/3.1415);
  Serial.printf("%f %f %f \n" , acos(axis.x/recip)*rad_deg , acos(axis.y/recip)*rad_deg , acos(axis.z/recip)*rad_deg);
  //Serial.printf("%f %f %f \n" , mpu.getAccX() , mpu.getAccY() , mpu.getAccZ());
  
  
}
