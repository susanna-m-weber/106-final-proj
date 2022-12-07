//#include <ESP32Encoder.h>
#define BIN_1 26
#define BIN_2 25

volatile float cur_pos; //Current position
volatile float des_pos; //Desired position
volatile int D = 0; //PWM vslue to send to motor
const int ledPin = 13;


// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 150;
const int MIN_PWM_VOLTAGE = 200;
volatile float pos_error = 0;; //Current error in position
volatile float sum_error = 0;
volatile float last_error = 0;
volatile float rate_error = 0;

volatile float last_time = 0;
volatile float delta_time = 0;

volatile float goalie_data[2];

int Kp = 1;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
int Ki = 1;
int Kd = 1;

float error_max = 1400; //Maximum error for anti-windup

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

   // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);

  millis();
}

void loop() {
  float goalie_data[2];
  Serial.flush();
  
  if (Serial.available()) {
    while (Serial.available()) {
      for (int i = 0; i < 2; i++) {
        goalie_data[i] = float((Serial.read()));
      }
      Serial.write(byte(abs(goalie_data[1] - goalie_data[0])));
      Serial.flush();
      if (abs(goalie_data[1] - goalie_data[0]) < 20) {
        digitalWrite(ledPin, HIGH);
      }
      else {
        digitalWrite(ledPin, LOW);
      }
    }
  }
  
  cur_pos = goalie_data[0];
  des_pos = goalie_data[1];

  delta_time = millis() - last_time;
  last_time = millis();

  pos_error = abs(cur_pos - des_pos);
  sum_error += pos_error;
  rate_error = abs(last_error - cur_pos)/delta_time;

  if(sum_error > error_max){
    sum_error = 1400; //TUNE THIS
  }

  if(sum_error < -error_max) {
    sum_error = -1400; //TUNE THIS TOO
  }

  D = Kp * pos_error + Ki * sum_error + Kd * rate_error;

  //Ensure that you don't go past the maximum possible command
  if (D > MAX_PWM_VOLTAGE) {
      D = MAX_PWM_VOLTAGE;
  }
  else if (D < -MAX_PWM_VOLTAGE) {
      D = -MAX_PWM_VOLTAGE;
  }

  //Ensure you send enough info to actute on motor
  if (D > -MIN_PWM_VOLTAGE && D<0) {
      D = -MIN_PWM_VOLTAGE;
  }
  else if (D < MIN_PWM_VOLTAGE && D>0) {
      D = MIN_PWM_VOLTAGE;
  }  

  //Map the D value to motor directionality
  //FLIP ENCODER PINS SO SPEED AND D HAVE SAME SIGN
  if (D > 0) {
      ledcWrite(ledChannel_1, LOW);
      ledcWrite(ledChannel_2, D);
  }
  else if (D < 0) {
      ledcWrite(ledChannel_1, -D);
      ledcWrite(ledChannel_2, LOW);
  }
  else {
      ledcWrite(ledChannel_1, LOW);
      ledcWrite(ledChannel_2, LOW);
  }
}
