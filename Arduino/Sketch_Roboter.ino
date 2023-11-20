#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "ac58input.h"
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>

Adafruit_ICM20948 imu1;
Adafruit_ICM20948 imu2;
AC58Input myAC58Input; 

float AngleRoll1, AngleRoll2, AnglePitch1, AnglePitch2;
byte buffer[27];

double qToFloat(const int fixedPointValue, const double qPoint)
{
    double qFloat = static_cast<double>(fixedPointValue);
    qFloat *= pow(2.0, qPoint * -1.0);
    return qFloat;
}

AC58Input ::AC58Input()
{

}

bool AC58Input::dataAvailable()
{
  return true;
}
static double quaternionToPitch(const float i, const float j, const float k, const float r) {
    const float norm = sqrt(r * r + i * i + j * j + k * k);
    const float dqw = r / norm;
    const float dqx = i / norm;
    const float dqy = j / norm;
    const float dqz = k / norm;

    const float ysqr = dqy * dqy;

    // Roll (x-axis rotation)
    float t0 = +2.0f * (dqw * dqx + dqy * dqz);
    float t1 = +1.0f - 2.0f * (dqx * dqx + ysqr);
    float roll = atan2(t0, t1) * 180.0f / M_PI;

    // Pitch (y-axis rotation)
    float t2 = +2.0f * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float pitch = asin(t2) * 180.0f / M_PI;

    // Yaw (z-axis rotation)
    float t3 = +2.0f * (dqw * dqz + dqx * dqy);
    float t4 = +1.0f - 2.0f * (ysqr + dqz * dqz);
    float heading = atan2(t3, t4) * 180.0f / M_PI;

    return pitch;
}


void AC58Input::readData(byte *buffer)
{
    buffer[0]  = 'A';
    buffer[1]  = 'E';
    buffer[2]  = 'N';
    buffer[3]  = 'C';
    int* Result = (int*) &buffer[4];
    for (int i = 0; i < 23; i++) {
    digitalWrite(12, HIGH);  // LED einschalten (HIGH ist das Spannungsniveau)
    digitalWrite(13, LOW);
    delayMicroseconds(5);
    bitWrite(*Result, 23 - i, digitalRead(4));
    digitalWrite(12, LOW);   // LED ausschalten (Spannung auf LOW setzen)
    digitalWrite(13, HIGH);
    delayMicroseconds(5);
}
bitWrite(*Result, 23, 0);

}
bool AC58Input::begin()
{
    pinMode(13, OUTPUT);
    pinMode(12, OUTPUT);
    pinMode(4, INPUT);

    return true; 
}

void AC58Input::setMode(int mode)
{

}

static char pingCount = 7;

void getAngleIMU(double* angle, int64_t* angle_ts) {

        if (buffer[0] == 'A' && buffer[1] == 'E' && buffer[2] == 'N' && buffer[3] == 'C') {
            double pitch = qToFloat(*(const int32_t*)&buffer[4], 23.0) * 360.0;
            if (pitch > 180.0) {
                pitch -= 360.0;
            }
            *angle = pitch;
            // Serial.print("AC58 Angle: ");
            // Serial.println(pitch);
        }

        if (buffer[0] == 'R' && buffer[1] == 'H' && buffer[2] == 'I' && buffer[3] == 'D') {
            double i = qToFloat(*(const int32_t*)&buffer[4], 14.0);
            double j = qToFloat(*(const int32_t*)&buffer[6], 14.0);
            double k = qToFloat(*(const int32_t*)&buffer[8], 14.0);
            double r = qToFloat(*(const int32_t*)&buffer[10], 14.0);
            *angle = quaternionToPitch(i, j, k, r);
            // Serial.print("BNO080 Angle: ");
            // Serial.println(*angle);
        }
     else {
    }

    pingCount = (pingCount + 1) % 220;
}

void setup() {
  Wire.begin();
  Serial.begin(115200);

  if (!imu1.begin_I2C(0x68))
  {
    while (1);
  }

  if (!imu2.begin_I2C(0x69))
  {
    while (1);
  }

  imu1.setAccelRateDivisor(0);
  imu1.setGyroRateDivisor(0);
  imu2.setAccelRateDivisor(0);
  imu2.setGyroRateDivisor(0);
 

}

void loop() {
  
  myAC58Input.readData(buffer);
  float timestamp;
  sensors_event_t accel1,accel2;
  sensors_event_t gyro1,gyro2;
  sensors_event_t mag1,mag2;
  sensors_event_t temp1,temp2;
  imu1.getEvent(&accel1, &gyro1, &temp1, &mag1);
  imu2.getEvent(&accel2, &gyro2, &temp2, &mag2);
  timestamp = millis()/10;
  double angle=-100;
  double angle2=-100;
  int64_t angle_ts;
  getAngleIMU(&angle,&angle_ts);
  Serial.print("$");
  Serial.print(" ; ");
  Serial.print(angle);
  Serial.print(" ; ");
  Serial.print(temp1.temperature);
  Serial.print(" ; ");
  Serial.print(timestamp);
  Serial.print(" ; ");
  Serial.print(accel1.acceleration.x);
  Serial.print(" ; ");
  Serial.print(accel1.acceleration.y);
  Serial.print(" ; ");
  Serial.print(accel1.acceleration.z);
  Serial.print(" ; ");
  Serial.print(gyro1.gyro.x);
  Serial.print(" ; ");
  Serial.print(gyro1.gyro.y);
  Serial.print(" ; ");
  Serial.print(gyro1.gyro.z);
  Serial.print(" ; ");
  Serial.print(accel2.acceleration.x);
  Serial.print(" ; ");
  Serial.print(accel2.acceleration.y);
  Serial.print(" ; ");
  Serial.print(accel2.acceleration.z);
  Serial.print(" ; ");
  Serial.print(gyro2.gyro.x);
  Serial.print(" ; ");
  Serial.print(gyro2.gyro.y);
  Serial.print(" ; ");
  Serial.print(gyro2.gyro.z);
  Serial.print(" ; ");
  Serial.println("%");


}
