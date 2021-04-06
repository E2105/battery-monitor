#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;
std_msgs::Float32MultiArray battery_msg;
ros::Publisher pub_volt("battery", &battery_msg);

float BIT_TO_VOLT = 4.75/1023.0;

/*
// Resistor values
float r1[] = {0, 9700, 22100, 22000};   // Ohms
float r2[] = {9900, 9800, 9900, 7400};  // Ohms
*/

float ratios[] = {1.0, (9.7+9.8)/9.8, (22.1+9.9)/9.9, (22.0+7.4)/7.4}; 


void setup() {
  nh.initNode();
  //nh.negotiateTopics();
  nh.advertise(pub_volt);
  
  battery_msg.data_length = 5;
  battery_msg.data = new float[5];
}


void loop() {

  //float batteryVoltage = 0.0;
  
  // Reading inputs
  float cell1 = analogRead(A3) * BIT_TO_VOLT;
  float cell2 = analogRead(A2) * BIT_TO_VOLT;
  float cell3 = analogRead(A1) * BIT_TO_VOLT;
  float cell4 = analogRead(A0) * BIT_TO_VOLT;

  // Calculating original cell voltage
  //cell1;
  cell2 = cell2 * ratios[1] - cell1;
  cell3 = cell3 * ratios[2] - cell1 - cell2;
  cell4 = cell4 * ratios[3] - cell1 - cell2 - cell3;
  float batteryVoltage = cell4 + cell3 + cell2 + cell1;

  // Updating ROS Topics
  battery_msg.data[0] = batteryVoltage;
  battery_msg.data[1] = cell1;
  battery_msg.data[2] = cell2;
  battery_msg.data[3] = cell3;
  battery_msg.data[4] = cell4;
  
  // Publish ROS Topic
  pub_volt.publish( &battery_msg );
  nh.spinOnce();
  delay(1000);
}
