void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float goalie_data[2];
  Serial.flush();
  for (int i = 0; i < 2; i++) {
    // Wait for data at the port
    while (!Serial.available());
    goalie_data[i] = float((Serial.read()));
    // delay(100);
    // Serial.write(byte(goalie_data[i]));
    Serial.flush(); 
  }
}
