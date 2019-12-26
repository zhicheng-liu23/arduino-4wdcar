#include <JY901.h>
#include <Wire.h>
#include "config.h"
#include "Encoder.h"
#include "Motor.h"
#include "Kinematics.h"
#include "PID.h"

#define ENCODER_OPTIMIZE_INTERRUPTS

//left side motors
Motor motor1(MOTOR1_PWM, MOTOR1_IN_A, MOTOR1_IN_B); //front
Motor motor3(MOTOR3_PWM, MOTOR3_IN_A, MOTOR3_IN_B);// rear
//right side motors
Motor motor2(MOTOR2_PWM, MOTOR2_IN_A, MOTOR2_IN_B); // front
Motor motor4(MOTOR4_PWM, MOTOR4_IN_A, MOTOR4_IN_B); // rear

//COUNTS_PER_REV = 0 if no encoder
int Motor::counts_per_rev_ = COUNTS_PER_REV;

//left side encoders
Encoder motor1_encoder(MOTOR1_ENCODER_A,MOTOR1_ENCODER_B); //front
Encoder motor3_encoder(MOTOR3_ENCODER_A,MOTOR3_ENCODER_B); //rear

//right side encoders
Encoder motor2_encoder(MOTOR2_ENCODER_A,MOTOR2_ENCODER_B); //front
Encoder motor4_encoder(MOTOR4_ENCODER_A,MOTOR4_ENCODER_B); //rear

Kinematics kinematics(MAX_RPM, WHEEL_DIAMETER, BASE_WIDTH, PWM_BITS);

PID motor1_pid(-255, 255, K_P, K_I, K_D);
PID motor2_pid(-255, 255, K_P, K_I, K_D);
PID motor3_pid(-255, 255, K_P, K_I, K_D);
PID motor4_pid(-255, 255, K_P, K_I, K_D);

double g_req_angular_vel_z = 0;
double g_req_linear_vel_x = 0;
double g_req_linear_vel_y = 0;

unsigned long g_prev_command_time = 0;
unsigned long g_prev_control_time = 0;
unsigned long g_publish_vel_time = 0;
unsigned long g_prev_imu_time = 0;
unsigned long g_prev_debug_time = 0;

bool g_is_first = true;

char g_buffer[50];
String command = "";
//  int x_int;
//  int y_int;
//  int z_int;
void setup()
{
  Wire.begin();
  delay(5);
  Serial3.begin(115200);
  //Serial2.begin(115200);
  Serial2.begin(115200);
  Serial.begin(115200);
}

void loop()
{
  static bool imu_is_initialized;
  //this block drives the robot based on defined rate

  while(Serial2.available()>0)
  {
      command +=char(Serial2.read());    
//    g_req_linear_vel_x = cmd_msg.linear.x;
//    g_req_linear_vel_y = cmd_msg.linear.y;
//    g_req_angular_vel_z = cmd_msg.angular.z;
      delay(2);
      g_prev_command_time = millis();
  }
  int x_placeinstring = 0;
  int y_placeinstring = 0;
  int z_placeinstring = 0;
  String x_command;
  String y_command;
  String z_command;
  int x_int;
  int y_int;
  int z_int;
  if(command.length()>0)
  {
//    Serial.print(command);
    x_placeinstring = command.indexOf('x');
    y_placeinstring = command.indexOf('y');
    z_placeinstring = command.indexOf('z');
    x_command = command.substring(x_placeinstring+1,y_placeinstring);
    y_command = command.substring(y_placeinstring+1,z_placeinstring);
    z_command = command.substring(z_placeinstring+1);
    x_int = x_command.toInt();
    y_int = y_command.toInt();
    z_int = z_command.toInt();
    g_req_linear_vel_x = x_int / 100.0;
    g_req_linear_vel_y = y_int / 100.0;
    g_req_angular_vel_z = z_int / 100.0;//the unit of input velocity is cm/s
//    Serial.println(x_int);
//    Serial.println(y_int);
//    Serial.println(z_int);
    command = "";   
  }


  
  if ((millis() - g_prev_control_time) >= (1000 / COMMAND_RATE)){
    moveBase();
    g_prev_control_time = millis();
  }

  //this block stops the motor when no command is received
  if ((millis() - g_prev_command_time) >= 400){
    stopBase();
  }

  //this block publishes velocity based on defined rate
  if ((millis() - g_publish_vel_time) >= (1000 / VEL_PUBLISH_RATE)){
    publishVelocities();
    g_publish_vel_time = millis();
  }

  //this block publishes the IMU data based on defined rate
  if ((millis() - g_prev_imu_time) >= (1000 / IMU_PUBLISH_RATE)){
    if (!imu_is_initialized){
      imu_is_initialized = initIMU();
      if(imu_is_initialized){
        motor1_encoder.write(0);
        motor2_encoder.write(0);
        motor3_encoder.write(0);
        motor4_encoder.write(0);
       //nh.loginfo("IMU Initialized");
       Serial.println("IMU Initialized");
      }else{
       //nh.logfatal("IMU failed to initialize. Check your IMU connection.");
       Serial.println("IMU failed to initialize. Check your IMU connection.");
      }
    }else{
      publishIMU();
    }
//
    g_prev_imu_time = millis();
  }

  //this block displays the encoder readings. change DEBUG to 0 if you don't want to display
  if(DEBUG){
    if ((millis() - g_prev_debug_time) >= (1000 / DEBUG_RATE)){
      printDebug();
      g_prev_debug_time = millis();
    }
  }

}

void moveBase()
{
  Kinematics::output req_rpm;
  //get the required rpm for each motor based on required velocities
  req_rpm = kinematics.getRPM(g_req_linear_vel_x, g_req_linear_vel_y, g_req_angular_vel_z);

  //the required rpm is capped at -/+ MAX_RPM to prevent the PID from having too much error
  //the PWM value sent to the motor driver is the calculated PID based on required RPM vs measured RPM
  Kinematics::output req_pwm;
  req_pwm.motor1 = kinematics.rpmToPWM(constrain(req_rpm.motor1, -MAX_RPM, MAX_RPM));
  req_pwm.motor2 = kinematics.rpmToPWM(constrain(req_rpm.motor2, -MAX_RPM, MAX_RPM));
  req_pwm.motor3 = kinematics.rpmToPWM(constrain(req_rpm.motor3, -MAX_RPM, MAX_RPM));
  req_pwm.motor4 = kinematics.rpmToPWM(constrain(req_rpm.motor4, -MAX_RPM, MAX_RPM));
  Kinematics::output real_pwm;
  real_pwm.motor1 = kinematics.rpmToPWM(motor1.rpm);
  real_pwm.motor2 = kinematics.rpmToPWM(motor2.rpm);
  real_pwm.motor3 = kinematics.rpmToPWM(motor3.rpm);
  real_pwm.motor4 = kinematics.rpmToPWM(motor4.rpm);
  Kinematics::output output_pwm;
  output_pwm.motor1 = motor1_pid.compute(req_pwm.motor1, real_pwm.motor1)*1;
  output_pwm.motor2 = motor2_pid.compute(req_pwm.motor2, real_pwm.motor2)*1;
  output_pwm.motor3 = motor3_pid.compute(req_pwm.motor3, real_pwm.motor3)*1;
  output_pwm.motor4 = motor4_pid.compute(req_pwm.motor4, real_pwm.motor4)*1;
  motor1.spin(output_pwm.motor1);
  motor3.spin(output_pwm.motor3);
  motor2.spin(output_pwm.motor2);
  motor4.spin(output_pwm.motor4);
//  Serial.print("x:");Serial.println(g_req_linear_vel_x);
//  Serial.print("y:");Serial.println(g_req_linear_vel_y);
//  Serial.print("z:");Serial.println(g_req_angular_vel_z);
  Serial.print("Encoder1:");Serial.print(req_rpm.motor1);Serial.print("\t");Serial.println(motor1.rpm);
  Serial.print("Encoder2:");Serial.print(req_rpm.motor2);Serial.print("\t");Serial.println(motor2.rpm);
  Serial.print("Encoder3:");Serial.print(req_rpm.motor3);Serial.print("\t");Serial.println(motor3.rpm);
  Serial.print("Encoder4:");Serial.print(req_rpm.motor4);Serial.print("\t");Serial.println(motor4.rpm);
}

void stopBase()
{
  g_req_linear_vel_x = 0.0;
  g_req_linear_vel_y = 0.0;
  g_req_angular_vel_z = 0.0;
}

void publishVelocities()
{
  //update the current speed of each motor based on encoder's count
  motor1.updateSpeed(motor1_encoder.read());
  motor2.updateSpeed(motor2_encoder.read());
  motor3.updateSpeed(motor3_encoder.read());
  motor4.updateSpeed(motor4_encoder.read());

  Kinematics::velocities vel;
  vel = kinematics.getVelocities(motor1.rpm, motor2.rpm, motor3.rpm, motor4.rpm);
//  Serial.print("Encoder1:");Serial.print(req_rpm.motor1);Serial.print("\t");Serial.println(motor1.rpm);
//  Serial.print("Encoder2:");Serial.print(req_rpm.motor2);Serial.print("\t");Serial.println(motor2.rpm);
//  Serial.print("Encoder3:");Serial.print(req_rpm.motor3);Serial.print("\t");Serial.println(motor3.rpm);
//  Serial.print("Encoder4:");Serial.print(req_rpm.motor4);Serial.print("\t");Serial.println(motor4.rpm);
//  Serial.print("velocities is:");Serial.print(vel.linear_x);Serial.print("\t");Serial.print(vel.linear_y);Serial.print("\t");Serial.print(vel.angular_z);Serial.println("\t");

  //fill in the object
  //raw_vel_msg.linear_x = vel.linear_x;
  //raw_vel_msg.linear_y = vel.linear_y;
  //raw_vel_msg.angular_z = vel.angular_z;

  //publish raw_vel_msg object to ROS
  //raw_vel_pub.publish(&raw_vel_msg);
}

bool initIMU()
{
    bool accel, gyro, mag;
    accel = initAccelerometer();
    gyro = initGyroscope();
    mag = initMagnetometer();

    if(accel && gyro && mag)
        return true;
    
    else
        return false;
}

bool initAccelerometer()
{
    delay(1000);
    return true;
}

bool initGyroscope()
{
    delay(1000);
    return true;
}

bool initMagnetometer()
{
    delay(1000);
    return true;
}
struct msg
{
  float linear_acceleration[3];
  float angular_velocity[3];   
  float angular[3];
};
void publishIMU()
{
//  struct msg
//  {
//    float linear_acceleration[3];
//    float angular_velocity[3];
//    float magnetic_field[3];
//  }
  
  msg raw_imu_msg;
  raw_imu_msg.linear_acceleration[0] = (float) JY901.stcAcc.a[0]/32768*16;
  raw_imu_msg.linear_acceleration[1] = (float) JY901.stcAcc.a[1]/32768*16;
  raw_imu_msg.linear_acceleration[2] = (float) JY901.stcAcc.a[2]/32768*16;

  raw_imu_msg.angular_velocity[0] = (float) JY901.stcGyro.w[0]/32768*2000;
  raw_imu_msg.angular_velocity[1] = (float) JY901.stcGyro.w[1]/32768*2000;
  raw_imu_msg.angular_velocity[2] = (float) JY901.stcGyro.w[2]/32768*2000;
  
  raw_imu_msg.angular[0] = (float) JY901.stcAngle.Angle[0]/32768*180;
  raw_imu_msg.angular[1] = (float) JY901.stcAngle.Angle[1]/32768*180;
  raw_imu_msg.angular[2] = (float) JY901.stcAngle.Angle[2]/32768*180;
  //Serial.print("acceleration is:");Serial.print(raw_imu_msg.linear_acceleration[0]);Serial.print("\t");Serial.print(raw_imu_msg.linear_acceleration[1]);Serial.print("\t");Serial.print(raw_imu_msg.linear_acceleration[2]);Serial.println();
  //Serial.print("angular_velocity is:");Serial.print(raw_imu_msg.angular_velocity[0]);Serial.print("\t");Serial.print(raw_imu_msg.angular_velocity[1]);Serial.print("\t");Serial.print(raw_imu_msg.angular_velocity[2]);Serial.println();
  //Serial.print("angular is:");Serial.print(raw_imu_msg.angular[0]);Serial.print("\t");Serial.print(raw_imu_msg.angular[1]);Serial.print("\t");Serial.print(raw_imu_msg.angular[2]);Serial.println();

  while (Serial3.available()) 
  {
    JY901.CopeSerialData(Serial3.read()); //Call JY901 data cope function
  }
  //pass accelerometer data to imu object
  //raw_imu_msg.linear_acceleration = readAccelerometer();

  //pass gyroscope data to imu object
  //raw_imu_msg.angular_velocity = readGyroscope();

  //pass accelerometer data to imu object
  //raw_imu_msg.magnetic_field = readMagnetometer();

  //publish raw_imu_msg object to ROS
  //raw_imu_pub.publish(&raw_imu_msg);
}

void printDebug()
{
//  sprintf (g_buffer, "Encoder FrontLeft: %ld", motor1_encoder.read());
//  nh.loginfo(g_buffer);
//  sprintf (g_buffer, "Encoder RearLeft: %ld", motor3_encoder.read());
//  nh.loginfo(g_buffer);
//  sprintf (g_buffer, "Encoder FrontRight: %ld", motor2_encoder.read());
//  nh.loginfo(g_buffer);
//  sprintf (g_buffer, "Encoder RearRight: %ld", motor4_encoder.read());
//  nh.loginfo(g_buffer);
//  Serial2.print("Encoder1:");Serial2.println(motor1_encoder.read());
//  Serial2.print("Encoder2:");Serial2.println(motor2_encoder.read());
//  Serial2.print("Encoder3:");Serial2.println(motor3_encoder.read());
//  Serial2.print("Encoder4:");Serial2.println(motor4_encoder.read());
}
