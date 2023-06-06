/* MPU6050_light library for Arduino
 *
 * Authors: Romain JL. Fétick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050_light_custom.h"
#include "Arduino.h"

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!) */
static float wrap(float angle, float limit)
{
  while (angle > limit)
    angle -= 2 * limit;
  while (angle < -limit)
    angle += 2 * limit;
  return angle;
}

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(TwoWire &w)
{
  wire = &w;
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0, 0, 0);
  setAccOffsets(0, 0, 0);
  q = Quaternion(1.0f , 0.0f , 0.0f , 0.0f);
  beta = 3.5;
}

byte MPU6050::begin(int gyro_config_num, int acc_config_num)
{
  // changed calling register sequence [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  byte status = writeData(MPU6050_PWR_MGMT_1_REGISTER, 0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  setGyroConfig(gyro_config_num);
  setAccConfig(acc_config_num);

  this->update(true);
  angleX = this->getAccAngleX();
  angleY = this->getAccAngleY();
  preInterval = millis(); // may cause lack of angular accuracy if begin() is much before the first update()
  return status;
}

byte MPU6050::writeData(byte reg, byte data)
{
  wire->beginTransmission(address);
  wire->write(reg);
  wire->write(data);
  byte status = wire->endTransmission();
  return status; // 0 if success
}

// This method is not used internaly, maybe by user...
byte MPU6050::readData(byte reg)
{
  wire->beginTransmission(address);
  wire->write(reg);
  wire->endTransmission(true);
  wire->requestFrom(address, (uint8_t)1);
  byte data = wire->read();
  return data;
}

/* SETTER */

byte MPU6050::setGyroConfig(int config_num)
{
  byte status;
  switch (config_num)
  {
  case 0: // range = +- 250 deg/s
    gyro_lsb_to_degsec = 131.0;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
    break;
  case 1: // range = +- 500 deg/s
    gyro_lsb_to_degsec = 65.5;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
    break;
  case 2: // range = +- 1000 deg/s
    gyro_lsb_to_degsec = 32.8;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10);
    break;
  case 3: // range = +- 2000 deg/s
    gyro_lsb_to_degsec = 16.4;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18);
    break;
  default: // error
    status = 1;
    break;
  }
  return status;
}

byte MPU6050::setAccConfig(int config_num)
{
  byte status;
  switch (config_num)
  {
  case 0: // range = +- 2 g
    acc_lsb_to_g = 16384.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
    break;
  case 1: // range = +- 4 g
    acc_lsb_to_g = 8192.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
    break;
  case 2: // range = +- 8 g
    acc_lsb_to_g = 4096.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
    break;
  case 3: // range = +- 16 g
    acc_lsb_to_g = 2048.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18);
    break;
  default: // error
    status = 1;
    break;
  }
  return status;
}

void MPU6050::setGyroOffsets(float x, float y, float z)
{
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z)
{
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff)
{
  if ((gyro_coeff < 0) or (gyro_coeff > 1))
  {
    gyro_coeff = DEFAULT_GYRO_COEFF;
  } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff)
{
  setFilterGyroCoef(1.0 - acc_coeff);
}

/* CALC OFFSET */

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc)
{
  if (is_calc_gyro)
  {
    setGyroOffsets(0, 0, 0);
  }
  if (is_calc_acc)
  {
    setAccOffsets(0, 0, 0);
  }
  float ag[6] = {0, 0, 0, 0, 0, 0}; // 3*acc, 3*gyro

  for (int i = 0; i < CALIB_OFFSET_NB_MES; i++)
  {
    this->fetchData();
    ag[0] += ax;
    ag[1] += ay;
    ag[2] += (az - 1.0);
    ag[3] += gx;
    ag[4] += gy;
    ag[5] += gz;
    delay(1); // wait a little bit between 2 measurements
  }

  if (is_calc_acc)
  {
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }

  if (is_calc_gyro)
  {
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050::fetchData()
{
  wire->beginTransmission(address);
  wire->write(MPU6050_ACCEL_OUT_REGISTER);
  wire->endTransmission(false);
  wire->requestFrom(address, (uint8_t)14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]

  for (int i = 0; i < 7; i++)
  {
    rawData[i] = wire->read() << 8;
    rawData[i] |= wire->read();
  }

  ax = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  ay = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  az = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) / acc_lsb_to_g - accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gx = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gy = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gz = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
  gx*=DEG_2_RAD;
  gy*=DEG_2_RAD;
  gz*=DEG_2_RAD;
}

void MPU6050::MadgwickUpdate( float del){  //del is the time step, must be in microseconds

  this->fetchData();
  float q0 = q.w , q1 = q.x  , q2 = q.y , q3 = q.z;
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
		recipNorm = fastInv(ax * ax + ay * ay + az * az);
   
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
		recipNorm = fastInv(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    
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

  q.w = q0;
  q.x = q1;
  q.y = q2;
  q.z = q3;

  q.normalize();
  
	
}

void MPU6050::update(bool exec_angle_calc = true)
{
  // retrieve raw data
  this->fetchData();

  // estimate tilt angles: this is an approximation for small angles!

  if (exec_angle_calc)
  {
    float sgZ = (az >= 0) - (az < 0);                                       // allow one angle to go from -180 to +180 degrees
    angleAccX = atan2(ax, sgZ * sqrt(az * az + ax * ax)) * RAD_2_DEG; // [-180,+180] deg
    angleAccY = -atan2(ax, sqrt(az * az + ay * ay)) * RAD_2_DEG;      // [- 90,+ 90] deg

    unsigned long Tnew = millis();
    float dt = (Tnew - preInterval) * 1e-3;
    preInterval = Tnew;

    // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
    // https://github.com/gabriel-milan/TinyMPU6050/issues/6
    angleX = wrap(filterGyroCoef * (angleAccX + wrap(angleX + gx * dt - angleAccX, 180)) + (1.0 - filterGyroCoef) * angleAccX, 180);
    angleY = wrap(filterGyroCoef * (angleAccY + wrap(angleY + sgZ * gy * dt - angleAccY, 90)) + (1.0 - filterGyroCoef) * angleAccY, 90);
    angleZ += gz * dt; // not wrapped (to do???)
  }
}

float MPU6050::fastInv(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


//Quaternion Class Functions
void Quaternion::normalize()
{
  float rec = fastInv(w*w + x*x + y*y + z*z);
  this->w *= rec;
  this->x *= rec;
  this->y *= rec;
  this->z *= rec;
}

float Quaternion::fastInv(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long *)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float *)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


Quaternion Quaternion::getInverse(){
  return Quaternion(w, -x, -y, -z);
}

void Quaternion::serialPrint(Quaternion x){   //Requires Serial.begin
  Serial.print(x.w);
  Serial.print(" ");
  Serial.print(x.x);
  Serial.print(" ");
  Serial.print(x.y);
  Serial.print(" ");
  Serial.print(x.z);
  Serial.print("\n");
}
