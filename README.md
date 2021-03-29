# Battery Monitoring System

This circuit was made to fit a 4S LiPo battery and its battery cell connector. It can be expanded with another connector, or a simple pin header. Using a pin header is not recommended because of the higher risk of frying the Arduino and your PC (If it is connected), at least not without a diode to protect from opposite polarity.

The values of the battery cells are measured by simple voltage dividers. They cannot be measured independantly without shorting your circuit, and the order is as follows: the value of cell1, the value of cell1+cell2, and so on. Each cell can reach a voltage of about 4.2 V, which is just under the 5 V max input of the Arduino Nano. The voltage dividers are designed to measure the entire voltage drop of the first cell, half of the second, third of the third and a fourth of the four cells combined. [Add schematic].

This can be run as a virtual node in a ROS system with rosserial through USB. Here it will be connected to a Raspberry Pi 4, and publish the BatteryState message from the default sensor messages class. It measures the individual voltage on each battery cell and the total voltage. The absolute minimum voltage on each cell is 3.2 V, so to visualize a low charge an OLED screen and a light array has been installed.

![Image of circuit](https://github.com/E2105/battery-monitor/blob/main/fig/battery_monitor_front.jpg)
