#include <ros.h>
#include <rover_pkg/UserInput.h>
// delete ~/Arduino/libraries/ros_lib
// make sure you're not root
// rosrun rosserial_arduino make_libraries.py ~/Arduino/libraries rover_pkg

byte ljx_pin = 0;
byte rjx_pin = 1;
byte but1_pin = 2;
byte but2_pin = 3;
byte but3_pin = 4;
byte led1_pin = 5;
byte led2_pin = 6;
byte led3_pin = 7;

bool but1LastRead = 0;
bool but2LastRead = 0;
bool but3LastRead = 0;

bool but1State = 0;
bool but2State = 0;
bool but3State = 0;

char controlMode[] = "basic_controller";

ros::NodeHandle nh;

rover_pkg::UserInput user_data;
ros::Publisher publisher("user_input_stream", &user_data);

void setup() {
  nh.initNode();
  nh.advertise(publisher);
  pinMode(but1_pin, INPUT_PULLUP);
  pinMode(but2_pin, INPUT_PULLUP);
  pinMode(but3_pin, INPUT_PULLUP);
  pinMode(led1_pin, OUTPUT);
  pinMode(led2_pin, OUTPUT);
  pinMode(led3_pin, OUTPUT);
  
}

float mapf(float x, float min1, float max1, float min2, float max2){
  float result;
  result = (x - min1)*(max2-min2)/(max1-min1)+min2;
  return result;
}

void loop() {
  user_data.controlMode = controlMode;
  
  float ljx = analogRead(ljx_pin);
  float rjx = analogRead(rjx_pin);  
  user_data.ljx = mapf(ljx, 0, 1023, -1, 1);
  user_data.rjx = mapf(rjx, 0, 1023, -1, 1);


  byte but1Read = digitalRead(but1_pin);
  if (!but1LastRead && but1Read){
    but1State = !but1State;
  }
  but1LastRead = but1Read;
  

  byte but2Read = digitalRead(but2_pin);
  if (!but2LastRead && but2Read){
    but2State = !but2State;
  }
  but2LastRead = but2Read;

  byte but3Read = digitalRead(but3_pin);
  if (!but3LastRead && but3Read){
    but3State = !but3State;
  }
  but3LastRead = but3Read;

  digitalWrite(led1_pin,but1State);
  digitalWrite(led2_pin,but2State);
  digitalWrite(led3_pin,but3State);

  user_data.but1 = but1State;
  user_data.but2 = but2State;
  user_data.but3 = but3State;

  publisher.publish( &user_data );
  
  nh.spinOnce();
  delay(50);
  
}
