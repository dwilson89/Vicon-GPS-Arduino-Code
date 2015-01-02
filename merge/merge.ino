void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  Serial1.begin(57600);
  Serial2.begin(57600);
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long time = millis();
  
  while(Serial1.available()>0)
  {
    uint8_t c = Serial1.read();
    //Serial2.write(c);
    Serial.write(c);
  }
  
  while(Serial2.available()>0)
  {
    uint8_t c = Serial2.read();
    Serial1.write(c);
  }
  
  while(Serial.available()>0)
  {
    uint8_t c = Serial.read();
    Serial1.write(c);
  }  
  
  delay(1);
}
