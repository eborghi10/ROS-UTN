const int PIN = A2;

uint16_t oldState;
uint16_t newState;
double count = 0.0;

void setup() {
  Serial.begin(9600);
  pinMode(PIN, INPUT);
  oldState = digitalRead(PIN);
}

void loop() {
  newState = digitalRead(PIN);
  if(oldState != newState)
  {
    if(newState)
    {
      count += 22.5;
      count = fmod(fmod(count, 360) + 360, 360);
      Serial.println(count);
    }
    oldState = newState;
  }
  else {
    Serial.println(count);
  }
  delay(200);
}
