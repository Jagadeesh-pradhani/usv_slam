# usv_slam



## Hardware Connections

<!-- ### Ultrasonic Sensors
Each ultrasonic sensor uses two GPIO pins (Trigger and Echo). The sensors are connected as follows (using BCM numbering):

- **front_left:** Trigger = GPIO 21, Echo = GPIO 23  
- **front_right:** Trigger = GPIO 26, Echo = GPIO 19  
- **back_left:** Trigger = GPIO 5, Echo = GPIO 0  
- **back_right:** Trigger = GPIO 11, Echo = GPIO 9  
- **left_front:** Trigger = GPIO 27, Echo = GPIO 17  
- **left_back:** Trigger = GPIO 10, Echo = GPIO 22  
- **right_front:** Trigger = GPIO 13, Echo = GPIO 6  
- **right_back:** Trigger = GPIO 3, Echo = GPIO 4   -->

### Motor Control
The USV’s motor driver is controlled via digital outputs. The motor connections are:

- **front_left:** GPIO 20  
- **front_right:** GPIO 16  
- **back_left:** GPIO 12  
- **back_right:** GPIO 1  
- **left_front:** GPIO 7  
- **left_back:** GPIO 8  
- **right_front:** GPIO 24  
- **right_back:** GPIO 25  

*Note:* This package uses simple on/off control. For finer speed control, consider using PWM.

### Pickup Mechanism
The ball pickup mechanism is controlled by the following GPIO pins:

- **motor1_in1:** GPIO 14  
- **motor1_in2:** GPIO 24  
- **motor2_in3:** GPIO 18  
- **motor2_in4:** GPIO 23  
