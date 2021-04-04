
// OLED
/*
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
*/

#include <ros.h>
#include <sensor_msgs/BatteryState.h>

/*
#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);
*/

#define CELLS         4
#define BIT_TO_VOLT   4.8/1023.0

/*
// Resistor values
float r1[] = {0, 9700, 22100, 22000};   // Ohms
float r2[] = {9900, 9800, 9900, 7400};  // Ohms
*/

double ratios[] = {1.0, (9.7+9.8)/9.8, (22.1+9.9)/9.9, (22.0+7.4)/7.4}; 

//int lightPins[] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 12};

ros::NodeHandle nh;
sensor_msgs::BatteryState battery_msg;
ros::Publisher batteryState("battery", &battery_msg);

void setup() {

  //Serial.begin(9600);

  // ROS initiation
  nh.initNode();
  nh.advertise(batteryState);
  nh.negotiateTopics();

  // Populate battery parameters.
  battery_msg.power_supply_status = 2;     // discharging
  battery_msg.power_supply_health = 0;     // unknown
  battery_msg.power_supply_technology = 3; // LiPo
  battery_msg.present = 1;                 // battery present

  battery_msg.location = "Manta";        // unit location
  battery_msg.serial_number = "Sewial4bwekfast";  // unit serial number
  battery_msg.cell_voltage = new float[CELLS];

  /*
  for (int outputs=0; outputs<sizeof(lightPins)+1; outputs++){
  pinMode(lightPins[outputs], OUTPUT);
  }

  // Initiating the OLED screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.setTextColor(WHITE);
  */
}


void loop() {

  double batteryVoltage = 0.0;
  
  // Reading inputs
  double cell1 = analogRead(A3) * BIT_TO_VOLT;
  double cell2 = analogRead(A2) * BIT_TO_VOLT;
  double cell3 = analogRead(A1) * BIT_TO_VOLT;
  double cell4 = analogRead(A0) * BIT_TO_VOLT;

  // Calculating original cell voltage
  //cell1;
  cell2 = cell2 * ratios[1] - cell1;
  cell3 = cell3 * ratios[2] - cell1 - cell2;
  cell4 = cell4 * ratios[3] - cell1 - cell2 - cell3;
  batteryVoltage = cell4 + cell3 + cell2 + cell1;

  // Updating ROS Topics
  battery_msg.voltage = (float)batteryVoltage;
  battery_msg.cell_voltage[0] = (float)cell1;
  battery_msg.cell_voltage[1] = (float)cell2;
  battery_msg.cell_voltage[2] = (float)cell3;
  battery_msg.cell_voltage[3] = (float)cell4;

  /*
  // LIGHT ARRAY
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
    lightUp(0);
  }

  // OLED
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("V: ");
  display.println(batteryVoltage);
  display.setTextSize(1);
  display.print("(");
  display.print(cell1);
  display.print(", ");
  display.print(cell2);
  display.println(", ");
  display.print(cell3);
  display.print(", ");
  display.print(cell4);
  display.println(")");
  display.display();
  display.clearDisplay();
  */
  
  // Publish ROS Topic
  batteryState.publish( &battery_msg );
  nh.spinOnce();
  delay(1000);
}

/*
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
}*/
