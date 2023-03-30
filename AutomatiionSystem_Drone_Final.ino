#include <Servo.h>
#include "PPMReader.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Bitcraze_PMW3901.h>
//----------------------------제어주기-----------------------
#define MCU_CONTROL_RATE 8
// ------------------------------motor-----------------------------------------
#define MIN_PULSE_LENGTH 1000
#define MAX_PULSE_LENGTH 2000

Servo motA, motB, motC, motD;
char data;
int motorA, motorB, motorC, motorD;
//------------------------IR센서--------------------------
Adafruit_VL53L0X sensor = Adafruit_VL53L0X();

const int maxDistance = 2000;
int height;

float target_height = 0;
float height_in;
float height_kp = 0.9;
float height_ki = 0;
float height_kd = 0.05; //0.01
float height_pterm;
float height_iterm;
float height_dterm;
float height_output;
float prev_height;

int count = 0;
//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡppm엔코더ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ//
byte interruptPin = 2;
byte chAmount = 8;
PPMReader ppm(interruptPin, chAmount);
double ch_array[9];

unsigned long t=0;
int prev_ch_array = 1000;
int throttle;
int gear;
//-------------MPU----------------------------------------------
const int MPU_ADDR = 0x68;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
double angleAcX, angleAcY, angleAcZ;
double angleGyX, angleGyY, angleGyZ;
double angleFiX, angleFiY, angleFiZ;
double angvelFiX, angvelFiY, angvelFiZ;

const double RADIAN_TO_DEGREE = 180 / 3.14159;  
const double DEG_PER_SEC = 32767 / 250;    // 1초에 회전하는 각도
const double ALPHA = 0.992;

double dt = 0;           // 한 사이클 동안 걸린 시간 변수 

double past_DT = 0;           // 한 사이클 동안 걸린 시간 변수 
double DT = 0;           // 한 사이클 동안 걸린 시간 변수 
double free_dt = 0;

double averAcX, averAcY, averAcZ;
double averGyX, averGyY, averGyZ;
//ㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ이중루프 PIDㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡㅡ
float roll_target_angle = 0;
float roll_angle_in;
float roll_rate_in;
float roll_stabilize_kp = 6;
float roll_stabilize_ki = 3;
float roll_rate_kp = 0.5;
float roll_rate_ki = 0;
float roll_rate_kd = 0;
float roll_stabilize_iterm;
float roll_rate_iterm;
float roll_rate_dterm;
float roll_output;

float pitch_target_angle = 0;
float pitch_angle_in;
float pitch_rate_in;  
float pitch_stabilize_kp = 5;
float pitch_stabilize_ki = 0;
float pitch_rate_kp = 0.4;
float pitch_rate_ki = 0;
float pitch_rate_kd = 0.0001;
float pitch_stabilize_iterm;
float pitch_rate_iterm;
float pitch_rate_dterm;
float pitch_output;
float pitch_output_prev;

float yaw_target_rate = 0.0;
float yaw_rate_in;
float yaw_rate_kp = 5;
float yaw_rate_ki = 0.2;
float yaw_rate_kd = 0;
float yaw_rate_pterm;
float yaw_rate_iterm;
float yaw_rate_dterm;
float yaw_output;
//--------------------------optical flow--------------------
Bitcraze_PMW3901 flow(10);

float x_dot, y_dot;
int16_t deltaX, deltaY;

const float theta_px=42.0;
const float theta_py=42.0;
const float Nx=300.0;
const float Ny=300.0;

float prev_x_dot = 0;
float prev_y_dot = 0;

float x_target_dot = 0;
float x_dot_in;
float x_dot_kp = 7;
float x_dot_ki = 0;
float x_dot_kd = 0;
float x_dot_pterm;
float x_dot_iterm;
float x_dot_dterm;
float x_dot_output;

float y_target_dot = 0;
float y_dot_in;
float y_dot_kp = 7;
float y_dot_ki = 0;
float y_dot_kd = 0;
float y_dot_pterm;
float y_dot_iterm;
float y_dot_dterm;
float y_dot_output;

void setup()
{
  Serial.begin(115200);

  initMPUSensor();
  initIR();
  if (!flow.begin())
  {
    Serial.println("connection fail");
    while(1);
  }
  
  motA.attach(3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motB.attach(5, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motC.attach(6, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  motD.attach(9, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  
  motA.writeMicroseconds(MIN_PULSE_LENGTH);
  motB.writeMicroseconds(MIN_PULSE_LENGTH);
  motC.writeMicroseconds(MIN_PULSE_LENGTH);
  motD.writeMicroseconds(MIN_PULSE_LENGTH);

  delay(5000);
  
  caliSensor();
}
 

void loop()
{
  past_DT = free_dt;
  
  getangle();

  if (sensor.isRangeComplete())
  {
    height = sensor.readRangeResult();
    
    if (height < maxDistance)
    {
      height;
    }
  }

  flow.readMotionCount(&deltaX, &deltaY);
  optical_vel();
  
  for (byte ch = 0; ch <= chAmount; ++ch)
  {
    ch_array[ch] = ppm.latestValidChannelValue(ch, 0);
  }

  if (ch_array[4] < 1090) ch_array[4] = 1000;
  ch_array[4] = map(ch_array[4], 1100, 1904, 1000, 2000);
  if (ch_array[4] == 0) ch_array[4] = prev_ch_array;

  ch_array[8] = mapping((double)ch_array[8], 1100.0, 1904.0, 14, -16); // 13 -17
  ch_array[2] = mapping((double)ch_array[2], 1100.0, 1904.0, 3.0, -3.0);
  ch_array[3] = mapping((double)ch_array[3], 1100.0, 1904.0, 30.0, -30.0);
  
  throttle = ch_array[4];
  
  y_target_dot = ch_array[2];
  x_target_dot = ch_array[8];

  roll_target_angle = y_dot_output;  // 여기 다시 확인
  pitch_target_angle = x_dot_output;  // 여기 다시 확인
  yaw_target_rate = ch_array[3];
  gear = ch_array[6];

  if(gear < 1500)
  {
    motorA = 1000;
    motorB = 1000;
    motorC = 1000;
    motorD = 1000;
  }
  else if(gear > 1500)
  {
    calcdot();
    calcYPRtoDualPID();
    
    if(throttle >= 1300 && throttle < 1700)
    {
      target_height = prev_height;

      count = 0;
    }
    else if(throttle >= 1700)
    {
      count++;
      
      if(count%30 == 0)
      {
        target_height += 10;
      }
    }
    else if(throttle < 1300)
    {
      count++;
      
      if(count%30 == 0)
      {
        target_height -= 10;
      }
    }

    if(target_height >= 200)
    {
      target_height = 200;
    }
    else if(target_height <= 0)
    {
      target_height = 0;
    }

    PID_height(target_height, height_in, height_kp, height_ki, height_kd, height_pterm, height_iterm, height_dterm, height_output);
    
    motorA = 1200 - pitch_output - roll_output + yaw_output + height_output;
    motorB = 1200 - pitch_output + roll_output - yaw_output + height_output;
    motorC = 1200 + pitch_output - roll_output - yaw_output + height_output;
    motorD = 1200 + pitch_output + roll_output + yaw_output + height_output;
  }
  
  
  motA.writeMicroseconds(motorA);
  motB.writeMicroseconds(motorB);
  motC.writeMicroseconds(motorC);
  motD.writeMicroseconds(motorD);

  prev_height = target_height;  

  Serial.println(gear);


  while(free_dt - past_DT < MCU_CONTROL_RATE)
  {
    free_dt = millis();
  }

  dt = (free_dt - past_DT)/1000.0;
}

void initMPUSensor()
{
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);// I2C 통신용 어드레스(주소)
  Wire.write(0x6B);// MPU6050과 통신을 시작하기 위해서는 0x6B번지에    
  Wire.write(0x01);
  Wire.endTransmission(true);

  Wire.write(0x1A);
  Wire.write(0x03);//  DLPF 10Hz = 0x05, DLPF 260Hz = 0x00 //0x02 42Hz 0x03
  Wire.endTransmission(true);

  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission(true);  
}

void getangle()
{
  getData();

  double las_angle_gx = angleFiX;
  double las_angle_gy = angleFiY;
  double las_angle_gz = angleFiZ;

  angleAcX = atan(AcY / sqrt(pow(AcX, 2) + pow(AcZ, 2))) * 180 / PI;
  angleAcY = atan(-AcX / sqrt(pow(AcY, 2) + pow(AcZ, 2))) * 180 / PI;

  angvelFiX = ((GyX - averGyX) / DEG_PER_SEC);
  angvelFiY = ((GyY - averGyY) / DEG_PER_SEC);
  angvelFiZ = ((GyZ - averGyZ) / DEG_PER_SEC);
  
  double angleTmpX = las_angle_gx + angvelFiX * dt;
  double angleTmpY = las_angle_gy + angvelFiY * dt;
  double angleTmpZ = las_angle_gz + angvelFiZ * dt;

  angleFiX = ALPHA * angleTmpX + (1.0 - ALPHA) * angleAcX;
  angleFiY = ALPHA * angleTmpY + (1.0 - ALPHA) * angleAcY;
  angleFiZ = angleGyZ;
}

void getData()
{
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void caliSensor()
{
  double sumAcX = 0 , sumAcY = 0, sumAcZ = 0;
  double sumGyX = 0 , sumGyY = 0, sumGyZ = 0;
  
  getData();
  
  for (int i=0;i<200;i++)
  {
    getData();
    sumAcX+=AcX;  sumAcY+=AcY;  sumAcZ+=AcZ;
    sumGyX+=GyX;  sumGyY+=GyY;  sumGyZ+=GyZ;
    delay(5);
  }
  
  averAcX=sumAcX/200;  averAcY=sumAcY/200;  averAcZ=sumAcY/200;
  averGyX=sumGyX/200;  averGyY=sumGyY/200;  averGyZ=sumGyZ/200;
}

void dualPID(float target_angle,
             float angle_in,
             float rate_in,
             float stabilize_kp,
             float stabilize_ki,
             float rate_kp,
             float rate_ki,
             float rate_kd,
             float &stabilize_iterm,
             float &rate_iterm,
             float &rate_dterm,
             float &output
             ){
  float angle_error;
  float desired_rate;
  float rate_error;
  float stabilize_pterm, rate_pterm;
  float rate_prev_dterm;
  float prev_rate;
  
  angle_error = target_angle - angle_in;

  stabilize_pterm = stabilize_kp * angle_error;
  stabilize_iterm += stabilize_ki * angle_error * dt; //안정화 적분항//

  desired_rate = stabilize_pterm;

  rate_error = desired_rate - rate_in;
  
  rate_pterm = rate_kp * rate_error; //각속도 비례항//
  rate_iterm += rate_ki * rate_error * dt; //각속도 적분항//
  rate_dterm = -((rate_in - prev_rate)/dt)*rate_kd;

  rate_dterm = rate_prev_dterm*0.9 + rate_dterm*0.1;

  if(stabilize_iterm > 100)
  {
    stabilize_iterm = 100;
  }
  else if(stabilize_iterm < -100)
  {
    stabilize_iterm = -100;
  }

  if(rate_iterm > 50)
  {
    rate_iterm = 50;
  }
  else if(rate_iterm < -50)
  {
    rate_iterm = -50;
  }
  
  output = rate_pterm + rate_iterm + stabilize_iterm + rate_dterm;

  prev_rate = rate_in;
  rate_prev_dterm = rate_dterm;
}

void first_PID(float target_rate, float rate_in, float rate_kp, float rate_ki, float rate_kd, float &rate_pterm, float &rate_iterm, float &rate_dterm, float &output)
{
  float rate_error;
  float desired_rate;
  float rate_prev_dterm;
  float prev_rate;

  rate_error = target_rate - rate_in;

  rate_pterm = rate_kp * rate_error;
  rate_iterm += rate_ki * rate_error * dt;
  rate_dterm = -((rate_in - prev_rate)/dt)*rate_kd;

  rate_dterm = rate_prev_dterm*0.9 + rate_dterm*0.1;

  output = rate_pterm + rate_iterm + rate_dterm;

  prev_rate = rate_in;
  rate_prev_dterm = rate_dterm;
}

void PID_height(float target_height, float height_in, float height_kp, float height_ki, float height_kd, float &height_pterm, float &height_iterm, float &height_dterm, float &output)
{
  float height_error;
  float height_prev_dterm;
  float prev_height;

  height_error = target_height - height_in;

  height_pterm = height_kp * height_error;
  height_iterm += height_ki * height_error * dt;
  height_dterm = -((height_in - prev_height)/dt)*height_kd;

  height_dterm = height_prev_dterm*0.9 + height_dterm*0.1;

  output = height_pterm + height_iterm + height_dterm;

  if(output >= 130)
  {
    output = 130;
  }
  
  prev_height = height_in;
  height_prev_dterm = height_dterm;
}

void PID_velocity(float target_dot, float dot_in, float dot_kp, float dot_ki, float dot_kd, float &dot_pterm, float &dot_iterm, float &dot_dterm, float &dot_output)
{
  float dot_error;
  float prev_dot_dterm;
  float prev_dot;

  dot_error = target_dot - dot_in;

  dot_pterm = dot_error * dot_kp;
  dot_iterm += dot_error * dot_ki * dt;
  dot_dterm = -((dot_in - prev_dot)/dt) * dot_kd;

  dot_dterm = prev_dot_dterm*0.9 + dot_dterm*0.1;
  
  dot_output = dot_pterm + dot_iterm + dot_dterm;

  prev_dot_dterm = dot_dterm;
  prev_dot = dot_in;
}

void calcdot()
{
  x_dot_in = x_dot;
  PID_velocity(x_target_dot, x_dot_in, x_dot_kp, x_dot_ki, x_dot_kd, x_dot_pterm, x_dot_iterm, x_dot_dterm, x_dot_output);

  if(x_dot_output >= 3)
  {
    x_dot_output = 3;
  }
  else if(x_dot_output <= -5)
  {
    x_dot_output = -5;
  }
  
  y_dot_in = y_dot;
  PID_velocity(y_target_dot, y_dot_in, y_dot_kp, y_dot_ki, y_dot_kd, y_dot_pterm, y_dot_iterm, y_dot_dterm, y_dot_output);

  if(y_dot_output >= 3)
  {
    y_dot_output = 3;
  }
  else if(y_dot_output <= -3)
  {
    y_dot_output = -3;
  }
}

void calcYPRtoDualPID(){
  roll_angle_in = angleFiX;
  roll_rate_in = angvelFiX;

  dualPID(roll_target_angle,
          roll_angle_in,
          roll_rate_in,
          roll_stabilize_kp,
          roll_stabilize_ki,
          roll_rate_kp,
          roll_rate_ki,
          roll_rate_kd,
          roll_stabilize_iterm,
          roll_rate_iterm,
          roll_rate_dterm,
          roll_output
  );

  pitch_angle_in = angleFiY;
  pitch_rate_in = angvelFiY;

  dualPID(pitch_target_angle,
          pitch_angle_in,
          pitch_rate_in,
          pitch_stabilize_kp,
          pitch_stabilize_ki,
          pitch_rate_kp,
          pitch_rate_ki,
          pitch_rate_kd,
          pitch_stabilize_iterm,
          pitch_rate_iterm,
          pitch_rate_dterm,
          pitch_output
  );
  
  yaw_rate_in = angvelFiZ;

  first_PID(yaw_target_rate, yaw_rate_in, yaw_rate_kp, yaw_rate_ki, yaw_rate_kd, yaw_rate_pterm, yaw_rate_iterm, yaw_rate_dterm, yaw_output);
}

double mapping(float x, float b, float c, float d, float e)
{
  double k = ((d-e)/(b-c));
  double result = k*(x-1500)+((d+e)/2);

  return result;
}

void initIR()
{
  Wire.begin(0x69);

  if (!Serial) delay(3000);

  if (!sensor.begin())
  {
    Serial.println("Sensor nor responding. Check wiring.");
    while (true);
  }
  sensor.configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT);
  sensor.startRangeContinuous();
}

void optical_vel()
{
  x_dot = -(height*(theta_py*PI/180.0)*deltaY/(dt*Ny) - (height*(angvelFiY*PI/180.0))*1.5)/1000.0;
  y_dot = (height*(theta_px*PI/180.0)*deltaX/(dt*Nx) + (height*(angvelFiX*PI/180.0))*1.5)/1000.0;
  
  x_dot = prev_x_dot*0.9 + x_dot*0.1;
  y_dot = prev_y_dot*0.9 + y_dot*0.1;

  if(x_dot > 5)
  { 
    x_dot = prev_x_dot;
  }
  else if(x_dot < -5)
  {
    x_dot = prev_x_dot;
  }

  if(y_dot > 5)
  {
    y_dot = prev_y_dot;
  }
  else if(y_dot < -5)
  {
    y_dot = prev_y_dot;
  }
  
  prev_x_dot = x_dot;
  prev_y_dot = y_dot;
}
