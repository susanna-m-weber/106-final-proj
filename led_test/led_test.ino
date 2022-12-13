const int ledPin = 13;

void setup() {
 Serial.begin(115200);
 pinMode(ledPin, OUTPUT);
 Serial.setTimeout(1);
}

void loop() {
 float goalie_data[2];
  Serial.flush();
  
  if (Serial.available()) {
    
    while (Serial.available()) {
      for (int i = 0; i < 2; i++) {
        goalie_data[i] = float((Serial.read()));
      }
      //Serial.write(byte(abs(goalie_data[1] - goalie_data[0])));
      Serial.flush();
      if (abs(goalie_data[1] - goalie_data[0]) < 20) {
        digitalWrite(ledPin, HIGH);
      }
      else {
        digitalWrite(ledPin, LOW);
      }
    }
  }
}



