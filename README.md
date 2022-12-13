# 106-final-proj
EE C106A Group 3 Final Project
### Demo Files
- Arduino: led_baby 
- Python: regress.py
#### Demo Notes to Self: 
- 5 jenga block stack 
- make sure green ball is visible at star

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
### To test color detection CV code:
  - Run color_detect_test.py or color_decection.ipynb
