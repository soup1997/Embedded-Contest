///// Embedded System Contest /////

/******** PIN MAP *********/
// 1. Servo Motor
/* Pin Number : 10 (PWM)
 * Servo Range(angle) : 140 ~ 40
 * 140 : Left
 *  90 : Middle
 *  40 : Right
 */

// 2. DC Motor with Motor Driver
/* Pin Number : 6 (Direction 1)
 *              7 (Direction 2)
 *              11 (PWM)
 */
/**************************/

// include library
#include <ros.h>
#include <Servo.h>
#include <ackermann_msgs/AckermannDriveStamped.h>


//////////////////////////////////////////////////

// PIN Connection
const int servo_motor = 10;
const int motor_1 = 6;
const int motor_2 = 7;
const int motor_pwm = 11;

// Set maximum and minimum limits for the PWM values
const int ANGLE_MIN = 40;
const int ANGLE_MAX = 140;

// Set maximum and minimum limits for the Motor Speed values
const int SPEED_MIN = 150;
const int SPEED_MAX = 250;

// Create Servo Motor Instance
Servo servo;

// Controlling Speed, Angle PWM Value
int motor_speed = 0;
int motor_angle = 0;

// Function Declaration
void ackermannCallback(const ackermann_msgs::AckermannDriveStamped & ackermann);
void motor_forward(void);
void motor_backward(void);
void motor_stop(void);
void calc_pwm_values(int speed, int angle);
void motor_control(int speed, int angle);

// ROS Init
ros::Subscriber<ackermann_msgs::AckermannDriveStamped> ackermannSubscriber("/ackermann_cmd", &ackermannCallback);
ros::NodeHandle nh;


//////////////////////////////////////////////////

// Callback func.
void ackermannCallback(const ackermann_msgs::AckermannDriveStamped & ackermann)
{
  int sub_speed = ackermann.drive.speed;
  int sub_angle = ackermann.drive.steering_angle;
  
  motor_control(sub_speed, sub_angle);
}

// Motor Direction Setting
void motor_forward(void){
  digitalWrite(motor_1, HIGH);
  digitalWrite(motor_2, LOW);
}
void motor_backward(void){
  digitalWrite(motor_1, LOW);
  digitalWrite(motor_2, HIGH);
}
void motor_stop(void){
  digitalWrite(motor_1, LOW);
  digitalWrite(motor_2, LOW);
}

// 
void calc_pwm_values(int speed, int angle)
{
  int pwm_speed = 0, pwm_angle = 0;
  
  // Motor direction setting
  if (speed > 0){          // Forward setting
    motor_forward();
  }
  else if (speed < 0){     // Backward setting
    motor_backward();
  }
  else {                   // Stop setting
    motor_stop();
  }
  
  /*** Speed ***/
  // Speed mapping
  if (speed == 0){
    pwm_speed = 0;
  }
  else {
    pwm_speed = abs(speed)*2 + 150;    // 0 ~ 50 => 150 ~ 250
  }
  
  // Speed limit
  if (pwm_speed < SPEED_MIN){
    pwm_speed = SPEED_MIN;
  }
  else if (pwm_speed > SPEED_MAX){
    pwm_speed = SPEED_MAX;
  }

  /*** Angle ***/
  // Angle mapping
  pwm_angle = -angle + 90;  // Sub => Left MAX = -50  ~  Right MAX = 50
                            // pwm => Left MAX = 140  ~  Right MAX = 40
  
  // Angle limit
  if (pwm_angle < ANGLE_MIN){
    pwm_angle = ANGLE_MIN;
  }
  else if (pwm_angle > ANGLE_MAX){
    pwm_angle = ANGLE_MAX;
  }
  
  // global variables
  motor_speed = pwm_speed;
  motor_angle = pwm_angle;
}

// Motor Control
void motor_control(int speed, int angle){
  calc_pwm_values(speed, angle);
  
  servo.write(motor_angle);
  analogWrite(motor_pwm, motor_speed);
}


//////////////////////////////////////////////////

void setup(){
  servo.attach(servo_motor);
  servo.write(90);
  
  pinMode(motor_1, OUTPUT);
  pinMode(motor_2, OUTPUT);
  pinMode(motor_pwm, OUTPUT);
  
  digitalWrite(motor_1, HIGH);
  digitalWrite(motor_2, LOW);
  
  motor_control(0, 0);    // speed(-50~50), angle(-50~50)
}


//////////////////////////////////////////////////

void loop(){
  /*
  // forward fast, left
  motor_control(30, -50);    // speed(-50~50), angle(-50~50)
  delay(3000);

  // forward slow, middle
  motor_control(5, 0);    // speed(-50~50), angle(-50~50)
  delay(3000);

  // backward slow, right
  motor_control(-15, 23);    // speed(-50~50), angle(-50~50)
  delay(3000);
  
  // backward fast, left
  motor_control(-50, -10);    // speed(-50~50), angle(-50~50)
  delay(3000);
  
  // stop, middle
  motor_control(0, 0);    // speed(-50~50), angle(-50~50)
  delay(3000);
  */
}
