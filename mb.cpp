#include <Arduino.h>
#include "ros.h"
#include "HardwareSerial.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/String.h"

#define ARRAY_LEN 4
#define MSG_LEN ARRAY_LEN*4 + 2

int incoming_byte;
int arm_incoming_byte;

String incoming_str;
String arm_incoming_str;

bool receive_cnt_flag;

bool direct_drive_flag;

bool drive_live_flag;

unsigned long last_time;

unsigned long millis_value;

String ardu_Str;

std_msgs::String millisStrig;

float torque_mode_float;

HardwareSerial DriveSerial(PC5, PC4);
HardwareSerial ArmSerial(PB11, PB10);


std_msgs::Float64MultiArray drive_published_feedback;
std_msgs::Float64MultiArray drive_joystick_array;

float joystick_float_array[ARRAY_LEN];

void multiArrayCallback(const std_msgs::Float64MultiArray &comingMultiArray);

void multiArrToArr(std_msgs::Float64MultiArray command_arr, float *commands_to_send);

int mapData(float x);
String getDirection(float coming_float);
String generateString(float x);
String generateMCUmessage(float *commands_to_send, int mode_torque);

void getThrustings(String encoderwDir);
void DriveFeedbackListener(void);

void armRead(void);

ros::NodeHandle nh;
ros::Publisher driveFeedbackPub("drive_feedback_topic", &drive_published_feedback);
ros::Subscriber <std_msgs::Float64MultiArray> joySub("/drive_system/wheel_speed", &multiArrayCallback);
//ros::Subscriber <std_msgs::Float64MultiArray> joySub("multiarray_topic", &multiArrayCallback);
ros::Publisher millispub("millis_topic", &millisStrig);

void setup() {
  nh.initNode();
  nh.advertise(driveFeedbackPub);
  nh.subscribe(joySub);
  nh.advertise(millispub);
  
  DriveSerial.begin(9600);
  ArmSerial.begin(9600);

  drive_published_feedback.data=(float *)malloc(sizeof(float)*ARRAY_LEN);  
  drive_published_feedback.data_length=ARRAY_LEN;

  drive_joystick_array.data=(float *)malloc(sizeof(float)*ARRAY_LEN);  
  drive_joystick_array.data_length=ARRAY_LEN;

  last_time = millis();
}

void loop() {
  
  nh.spinOnce();
  
  delay(1);

  unsigned long current_time = millis();
  if ((!nh.connected() && !direct_drive_flag) || ((current_time-last_time) > 500)) {
        DriveSerial.println("S00000000000000000F");
        delay(1);
  }
  
  DriveFeedbackListener();
  armRead();

  ardu_Str=String(current_time-last_time);  
  millisStrig.data=ardu_Str.c_str();

  millispub.publish(&millisStrig);
}

void multiArrayCallback(const std_msgs::Float64MultiArray &comingMultiArray){
  
  //unsigned long drive_callback_start_time=millis();
  last_time = millis();
  
  drive_joystick_array.data[0]=comingMultiArray.data[0];
  drive_joystick_array.data[1]=comingMultiArray.data[1];
  drive_joystick_array.data[2]=comingMultiArray.data[2];
  drive_joystick_array.data[3]=comingMultiArray.data[3];

  torque_mode_float = comingMultiArray.data[4];

  /*
  if(abs(millis()-drive_callback_start_time>=500)){
    drive_live_flag=false;
  }
  */

  /*
  thrust_up_flag = comingMultiArray.data[5];
  */

  //drive_message_start_time=millis();
  /*
  if(fabs(drive_message_control_time-drive_message_start_time>=500)){
    drive_live_flag=false;
  }
  */

  multiArrToArr(drive_joystick_array, joystick_float_array);
  DriveSerial.println(generateMCUmessage(joystick_float_array, int(torque_mode_float)));
  delay(1);

  
}

void multiArrToArr(std_msgs::Float64MultiArray command_arr, float *commands_to_send){
    for (int i = 0; i < ARRAY_LEN; i++){
        commands_to_send[i]=command_arr.data[i];
    }
}

int mapData(float coming_float){
  coming_float=255*coming_float;
  if(coming_float>255){
    coming_float=255;
  }
  if(coming_float<=255){
    coming_float=coming_float;
  }
  return coming_float;
}

String getDirection(float coming_float){
    String direction;
    if(coming_float<=0){
        direction='0';
    }
    if(coming_float>0){
        direction='1';
    }
    return direction;
}

String generateString(float x){
    String processedString = String(mapData(fabs(x)));
    while(processedString.length()<3){
        processedString = "0" + processedString;
    }
    return getDirection(x) + processedString;
}

String generateMCUmessage(float *commands_to_send, int mode_torque){
  String sentString = "S";
    for(int i=0;i<ARRAY_LEN;i++){
        sentString+=generateString(commands_to_send[i]);
    }
    sentString+=String(mode_torque);
    sentString+='F';
    return sentString;
}

void getThrustings(String encoderwDir){
  String str_buffer;
  char direction_char;
  int direction;
  for(int i=0;i<ARRAY_LEN;i++){
    direction_char=encoderwDir[4*i];
    if(direction_char=='0'){
      direction=-1;
    }
    if(direction_char=='1'){
      direction=1;
    }
    for(int j=i*4;j<(i*4)+3;j++){
      str_buffer+=encoderwDir[j+1];    
    }
    drive_published_feedback.data[i]=direction*str_buffer.toFloat()/255;
    str_buffer="";
    }
}

void DriveFeedbackListener(void){
  if (DriveSerial.available() > 0){
    incoming_byte = DriveSerial.read();
    if (incoming_byte == 'A'){
      incoming_str = "";
      receive_cnt_flag = true;
      return;
    }
    else if (receive_cnt_flag = true && incoming_byte != 'B'){
      incoming_str += (char) incoming_byte;
    }
    else if (incoming_byte == 'B'){
      getThrustings(incoming_str);
      driveFeedbackPub.publish(&drive_published_feedback);
      incoming_str = "";
      receive_cnt_flag = false;
    }   
  }
}

void armRead(void){
  static unsigned long arm_message_start_time=0;
  unsigned long arm_message_control_time=millis();

  if (ArmSerial.available() > 0){
    arm_incoming_byte = ArmSerial.read();
    
    if (arm_incoming_byte == 'S'){
      arm_message_start_time=millis();
      direct_drive_flag=true;
      arm_incoming_str = "";
      receive_cnt_flag = true;
      arm_incoming_str += (char) arm_incoming_byte;
      return;
    }
    else if (receive_cnt_flag = true && arm_incoming_byte != 'F'){
      arm_incoming_str += (char) arm_incoming_byte;
    }
    else if (arm_incoming_byte == 'F'){
      arm_incoming_str += (char) arm_incoming_byte;
      DriveSerial.println(arm_incoming_str);
      delay(1);
      arm_incoming_str = "";
      receive_cnt_flag = false;
    }   
  }
  if(fabs(arm_message_control_time-arm_message_start_time)>=500){
    direct_drive_flag=false;
  }
}
