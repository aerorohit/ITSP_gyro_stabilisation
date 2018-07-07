

#include <Wire.h>
#include <Kalman.h>
#include <MPU6050.h>
#include <PID_v1.h>
#include <Servo.h>

Servo s1,s2,s3;

double setpoint=0;
double output1,output2,output3;
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll1=0,pitch1=0,yaw1=0;
int pos1,pos2,pos3;

unsigned int ch6;



PID p1(&kalAngleX,&output1,&setpoint,1.4,0.048,0.1,REVERSE);
PID p2(&kalAngleY,&output2,&setpoint,1.4,0.048,0.1,DIRECT);
//PID p3(&yaw1,&output3,&setpoint,3,0.04,1.9,DIRECT);


MPU6050 mpu;

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only

uint32_t timer;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  Wire.begin();

  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  mpu.calibrateGyro();
  mpu.setThreshold(3);

  delay(100); // Wait for sensor to stabilize

  Vector rawAccel = mpu.readRawAccel();



  /* Set kalman and gyro starting angle */
  accX = rawAccel.XAxis;
  accY = rawAccel.YAxis;
  accZ = rawAccel.ZAxis;


  // It is then converted from radians to degrees
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;


  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;

  timer = micros();

  s1.attach(3);
  s2.attach(5);
  s3.attach(10);

  p1.SetMode(AUTOMATIC);
  p2.SetMode(AUTOMATIC);
  //p3.SetMode(AUTOMATIC);
  p1.SetOutputLimits(-45,45);
  p2.SetOutputLimits(-45,45);
  //p3.SetOutputLimits(-45,45);

  pinMode(2,INPUT);
  pinMode(7,INPUT);
  pinMode(8,INPUT);
  pinMode(12,INPUT);


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() {

  ch6=pulseIn(2,HIGH);
  if(ch6>1500)
  {
    txcontrol();
  }
  else
  {
    gyrocontrol();
  }

}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyrocontrol()
{
  /* Update all the values */
  //while (i2cRead(0x3B, i2cData, 14));


  Vector racc = mpu.readRawAccel();
  Vector rawGyro = mpu.readRawGyro();

  accX = racc.XAxis;
  accY = racc.YAxis;
  accZ = racc.ZAxis;
  //tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = rawGyro.XAxis;
  gyroY = rawGyro.YAxis;
  gyroZ = rawGyro.ZAxis;


  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Read normalized values
   Vector norm = mpu.readNormalizeGyro();

   // Calculate Pitch, Roll and Yaw
   pitch1 = pitch1 + norm.YAxis * dt;
   roll1 = roll1 + norm.XAxis * dt;
   yaw1 = yaw1 + norm.ZAxis * dt;


  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;


  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s
  double gyroZrate = gyroZ /131.0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  //void relative_to_euler(double w_x, double w_y, double w_z){
  	// w refers to omega (angular rate about body axes)
  	//0.01745329251 = (3.142(PI) / 180degr) The Arduino sin function is in radians
  	gyroXrate  = gyroXrate + gyroYrate*sin(roll*0.01745329251)*tan(pitch*0.01745329251) + gyroZrate*cos(roll*0.01745329251)*tan(pitch*0.01745329251);
  	gyroYrate =       gyroYrate*cos(pitch*0.01745329251)                               - gyroZrate*sin(roll*0.01745329251);
  	gyroZrate  =       gyroYrate*sin(roll*0.01745329251)/cos(pitch*0.01745329251) + gyroZrate*cos(roll*0.01745329251)*cos(pitch*0.01745329251);




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);


  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;



  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;
////////////////////////////////////////////////////////////////////////////////////////////////////



  p1.Compute();
  p2.Compute();

  

  s1.write(90+output1);
  s2.write(85-output2);
  s3.write(90);

  delay(2);
}
/////////////////////---------///////------//////--------///////////---------//////////////////////////////////
void txcontrol()
{
  pos1=pulseIn(7,HIGH);
  pos2=pulseIn(8,HIGH);
  pos3=pulseIn(12,HIGH);
  s1.writeMicroseconds(pos1);
  s2.writeMicroseconds(pos2);
  s3.writeMicroseconds(pos3);



}
