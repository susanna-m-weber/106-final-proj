# Fuusball Bot: EE C106A Group 3 Final Project

![A gif demonstrating gameplay](imgs/example.gif)


### To run the code used at the prjoect showcase: 
 - Upload demo_arduino.ino to ESP
 - Run demo.py 
 - A pop-up window showing the color detection should appear, and the goalie should be able to respond to the ball's movement
 
### To blink LED based on color detection 
 - Upload color_dection_actuation.ino to ESP 
 - Run color_detection_actuation.py 
 - LED should turn off if the camera detects red, turn off if it detects green, and flicker if it it detects both

### To test if ESP and computer are communicating correctly: 
 - Upload com_test.ino to ESP32
 - Run com_test.py 
 - Check that output matches LED in code 
 
### To test if computer can actuate ESP: 
  - Upload led_test.ino to ESP32 
  - Run led_test.py 
  - LED should turn on when user enters 1, and off when user enters 0 

