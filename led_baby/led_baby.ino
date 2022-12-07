#define BIN_1 32
#define BIN_2 33

const int ledPin = 13;
volatile float cur_pos; //Current position
volatile float des_pos; //Desired position
volatile int D = 0; //PWM vslue to send to motor
volatile int dir = 0;

// setting PWM properties ----------------------------
const int freq = 5000;
const int ledChannel_1 = 1;
const int ledChannel_2 = 2;
const int resolution = 8;
const int MAX_PWM_VOLTAGE = 255;
const int NOM_PWM_VOLTAGE = 175;
volatile float pos_error = 0;; //Current error in position
volatile float sum_error = 0;
volatile float last_error = 0;
volatile float rate_error = 0;

volatile float last_time = 0;
volatile float delta_time = 0;

volatile float goalie_data[2];

volatile int get_outta_here = 1;

int Kp = 1;   // TUNE THESE VALUES TO CHANGE CONTROLLER PERFORMANCE
int Ki = 1;
int Kd = 1;

float error_max = 1400; //Maximum error for anti-windup

void setup() {
 Serial.begin(115200);
 pinMode(ledPin, OUTPUT);
 Serial.setTimeout(1);

  // configure LED PWM functionalitites
  ledcSetup(ledChannel_1, freq, resolution);
  ledcSetup(ledChannel_2, freq, resolution);

  // attach the channel to the GPIO to be controlled
  ledcAttachPin(BIN_1, ledChannel_1);
  ledcAttachPin(BIN_2, ledChannel_2);
}

void loop() {
 while (!Serial.available());
  //Serial.print(Serial.readString().toInt());
  dir = Serial.readString().toInt();
  if (dir == 1) {
      digitalWrite(ledPin, HIGH);
      D = -250;
    }
  else if (dir == 2) {
      digitalWrite(ledPin, LOW);
      D = 250;
    }
  else {
      digitalWrite(ledPin, LOW);
      D = 0;
  }

  //Ensure that you don't go past the maximum possible command
  if (D > MAX_PWM_VOLTAGE) {
      D = MAX_PWM_VOLTAGE;
  }
  else if (D < -MAX_PWM_VOLTAGE) {
      D = -MAX_PWM_VOLTAGE;
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

