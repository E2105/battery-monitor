// ROS implementation of BMS code

// ROS dependancies
#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/BatteryState.h>

// OLED Screen dependancies
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

ros::NodeHandle nh;
sensor_msgs::BatteryState battery_msg;
ros::Publisher battery("/battery_health", &battery_msg);

// Screen parameters
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);

// Resistor values
float r1[] = {0, 9700, 22100, 22000};   // Ohms
float r2[] = {9900, 9800, 9900, 7400};     // Ohms

// Pins
int lightPins[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
int cellPins[] = {A3, A2, A1, A0};


void setup() {
 Serial.begin(9600);
 
 // ROS initiation
 nh.initNode();
 nh.advertise(battery);

 // Initiating pins
 for (int inputs=0; inputs<sizeof(cellPins)+1; inputs++){
  pinMode(cellPins[inputs], INPUT);
 }
 
 for (int outputs=0; outputs<sizeof(lightPins)+1; outputs++){
  pinMode(lightPins[outputs], OUTPUT);
 }

 // Initiating the OLED screen
 display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
 display.display();
 delay(2000);

 // Clear the buffer.
 display.clearDisplay();
 display.setTextSize(1);
 display.setCursor(0, 0);
 display.setTextColor(WHITE);
 
}


void loop() {

  // Reading inputs
  int cell1 = analogRead(A3);
  int cell2 = analogRead(A2);
  int cell3 = analogRead(A1);
  int cell4 = analogRead(A0);

  // Calculating original cell voltage by multiplying with the reverse of the physical voltage divider ratio.
  // 4.75 V is used over 5.0 V because it is more accurate to the physical hardware.
  float volt1 = (cell1 * 4.75 / 1023.0);
  float volt2 = (cell2 * 4.75 / 1023.0) * ((r1[1] + r2[1]) / r2[1]) - volt1;
  float volt3 = (cell3 * 4.75 / 1023.0) * ((r1[2] + r2[2]) / r2[2]) - volt1 - volt2;
  float volt4 = (cell4 * 4.75 / 1023.0) * ((r1[3] + r2[3]) / r2[3]) - volt1 - volt2 - volt3;
  float batteryVoltage = volt1 + volt2 + volt3 + volt4;

  // Lighting up array based on battery capacity.
  //    Intervals of 0.35 V from 13.0 V and up.
  //    The measurement uncertainty is exploited in a way that a fully charged battery
  //    will show a higher voltage, and an undercharged one will show less to lessen
  //    the chance to kill the battery.
  if (batteryVoltage <= 17.5 && batteryVoltage > 16.15){
    lightUp(10);
  } else if (batteryVoltage <= 16.15 && batteryVoltage > 15.8){
    lightUp(9);
  } else if (batteryVoltage <= 15.8 && batteryVoltage > 15.45){
    lightUp(8);
  } else if (batteryVoltage <= 15.45 && batteryVoltage > 15.1){
    lightUp(7);
  } else if (batteryVoltage <= 15.1 && batteryVoltage > 14.75){
    lightUp(6);
  } else if (batteryVoltage <= 14.75 && batteryVoltage > 14.4){
    lightUp(5);
  } else if (batteryVoltage <= 16.1 && batteryVoltage > 15.9){
    lightUp(4);
  } else if (batteryVoltage <= 14.05 && batteryVoltage > 13.7){
    lightUp(3);
  } else if (batteryVoltage <= 13.7 && batteryVoltage > 13.35){
    lightUp(2);
  } else if (batteryVoltage <= 13.35 && batteryVoltage > 13.0){
    lightUp(1);
  } else if (batteryVoltage < 0.1 && batteryVoltage > -0.1){
    // When there is no battery attached given a small deadzone
    lightUp(0);
  } else {
    // If the voltage is something it shouldn't be; blink all lights!
    lightBlink();
  }

  // Displaying the measured voltages on the onboard screen
  display.setCursor(0, 0);
  display.print("Voltage: ");
  display.println(batteryVoltage);
  display.print("(");
  display.print(volt1);
  display.print(", ");
  display.print(volt2);
  display.println(", ");
  display.print(volt3);
  display.print(", ");
  display.print(volt4);
  display.println(")");
  display.display();
  delay(10);
  display.clearDisplay();
  
  // Publish ROS Topic
  battery.publish(&battery_msg);
  nh.spinOnce();
}


void lightUp(int num){
  // Lights up the amount of LEDs on a light array given by "num".
  
  for(int i=0; i<num; i++){
    digitalWrite(lightPins[i], HIGH);
  }
  for(int j=num; j<11; j++){
    digitalWrite(lightPins[j], LOW);
  }
}


void lightBlink(){
  // Blinks all the LEDs
  
  for(int i=0; i<11; i++){
    digitalWrite(lightPins[i], HIGH);
  }
  
  delay(250);
  
  for(int i=0; i<11; i++){
    digitalWrite(lightPins[i], LOW);
  }
  
  delay(250);
}
