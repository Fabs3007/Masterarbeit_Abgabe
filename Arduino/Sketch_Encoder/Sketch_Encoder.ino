#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
Adafruit_ICM20948 imu1;
Adafruit_ICM20948 imu2;

float AngleRoll1, AngleRoll2, AnglePitch1, AnglePitch2;
static auto startTime = micros();

byte buffer[23];

double angle = -100;

class AC58Input_Test
{
  AC58Input();
  void setMode(int mode);
  
};

double qToFloat(const int fixedPointValue, const double qPoint)
{
    double qFloat = static_cast<double>(fixedPointValue);
    qFloat *= pow(2.0, qPoint * -1.0);
    return qFloat;
}

static double quaternionToPitch(const float i, const float j, const float k, const float r) {
    const float norm = sqrt(r * r + i * i + j * j + k * k);
    const float dqw = r / norm;
    const float dqx = i / norm;
    const float dqy = j / norm;
    const float dqz = k / norm;

    const float ysqr = dqy * dqy;


    // Pitch (y-axis rotation)
    float t2 = +2.0f * (dqw * dqy - dqz * dqx);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    float pitch = asin(t2) * 180.0f / M_PI;


    return pitch;
}




void AC58Input(byte *buffer)
{
  int* Result =(int*) &buffer[0];
  for (int i =0; i <23; i++)
  {
    digitalWrite(12,HIGH);
    digitalWrite(13,LOW);
    delayMicroseconds(5);
    bitWrite(*Result,23-i,digitalRead(4));
    digitalWrite(12,LOW);
    digitalWrite(13,HIGH);
    delayMicroseconds(5);
  }
  bitWrite(*Result,23,0);
}

void getAngle(double* angle,int64_t* angle_ts)
{
  double pitch = qToFloat(*(const int32_t*)&buffer[0],23.0) * 360.0;
  if(pitch > 180)
  {
    pitch -= 360.0;
  }
  *angle = pitch;
}

AC58Input_Test::AC58Input()
{
  
}
void AC58Input_Test::setMode(int mode)
{

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

  pinMode(13,OUTPUT);
  pinMode(12,OUTPUT);
  pinMode(4,INPUT);
 

}

void loop() {
  AC58Input(buffer);
  sensors_event_t accel1,accel2;
  sensors_event_t gyro1,gyro2;
  sensors_event_t mag1,mag2;
  sensors_event_t temp1,temp2;
  imu1.getEvent(&accel1, &gyro1, &temp1, &mag1);
  imu2.getEvent(&accel2, &gyro2, &temp2, &mag2);
  int64_t angle_ts;
  getAngle(&angle,&angle_ts);
  auto currentTime = micros();
  auto elapsed_time = (currentTime - startTime);
  Serial.print("$");
  Serial.print(" ; ");
  Serial.print(temp1.temperature,8);
  Serial.print(" ; ");
  Serial.print(temp2.temperature,8);
  Serial.print(" ; ");
  Serial.print(elapsed_time);
  Serial.print(" ; ");
  Serial.print(accel1.acceleration.x,8);
  Serial.print(" ; ");
  Serial.print(accel1.acceleration.y,8);
  Serial.print(" ; ");
  Serial.print(accel1.acceleration.z,8);
  Serial.print(" ; ");
  Serial.print(gyro1.gyro.x,8);
  Serial.print(" ; ");
  Serial.print(gyro1.gyro.y,8);
  Serial.print(" ; ");
  Serial.print(gyro1.gyro.z,8);
  Serial.print(" ; ");
  Serial.print(accel2.acceleration.x,8);
  Serial.print(" ; ");
  Serial.print(accel2.acceleration.y,8);
  Serial.print(" ; ");
  Serial.print(accel2.acceleration.z,8);
  Serial.print(" ; ");
  Serial.print(gyro2.gyro.x,8);
  Serial.print(" ; ");
  Serial.print(gyro2.gyro.y,8);
  Serial.print(" ; ");
  Serial.print(gyro2.gyro.z,8);
  Serial.print(" ; ");
  Serial.print(angle,8);
  Serial.print(" ; ");
  Serial.println("%");



}
