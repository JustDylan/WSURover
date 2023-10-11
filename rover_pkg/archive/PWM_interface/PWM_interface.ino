#include <ros.h>
#include <rover_pkg/RoverData.h>
#include "Motor.h"
#include <Servo.h>

byte rb_fr_pin = 2;
byte rb_mr_pin = 3;
byte rb_rr_pin = 4;
byte rb_fl_pin = 5;
byte rb_ml_pin = 6;
byte rb_rl_pin = 7;
byte arm_zm_pin = 8;
byte arm_bm_pin = 9;
byte arm_am_pin = 10;
byte arm_wr_pin = 11;
byte awm_wp_pin = 12;


int maxAccel = 1;
int timeDelay = 50;

Motor FR_Drive;
Motor MR_Drive;
Motor RR_Drive;
Motor FL_Drive;
Motor ML_Drive;
Motor RL_Drive;

ros::NodeHandle nh;

void updateMotors(const rover_pkg::RoverData rover_data){
  FR_Drive.setTargetVel(rover_data.rb_fr);
  MR_Drive.setTargetVel(rover_data.rb_mr);
  RR_Drive.setTargetVel(rover_data.rb_rr);
  FL_Drive.setTargetVel(rover_data.rb_fl);
  ML_Drive.setTargetVel(rover_data.rb_ml);
  RL_Drive.setTargetVel(rover_data.rb_rl);
  nh.loginfo("working");



  
}

ros::Subscriber<rover_pkg::RoverData> sub("rover_control_stream", updateMotors);

void setup() {
  nh.initNode();
  nh.subscribe(sub);
  
  FR_Drive.motor_setup(rb_fr_pin);
  MR_Drive.motor_setup(rb_mr_pin);
  RR_Drive.motor_setup(rb_rr_pin);
  FL_Drive.motor_setup(rb_fl_pin);
  ML_Drive.motor_setup(rb_ml_pin);
  RL_Drive.motor_setup(rb_rl_pin);
  
}

void loop() {
  nh.spinOnce();
  FR_Drive.tick();
  MR_Drive.tick();
  RR_Drive.tick();
  FL_Drive.tick();
  ML_Drive.tick();
  RL_Drive.tick();
  
  delay(timeDelay);
  
}
