const int pinHall = A1;
#define Hall_Sensor_Gnd_Pin_0 5

void setup() {
  pinMode(pinHall, INPUT);
  Serial.begin(9600);
  pinMode(Hall_Sensor_Gnd_Pin_0, OUTPUT);
  digitalWrite(Hall_Sensor_Gnd_Pin_0, LOW);
}
void loop() {
  // we measure 10 times adn make the mean
  long measure = 0;
  for (int i = 0; i < 10; i++) {
    
      measure += analogRead(pinHall);
  }
  measure /= 10;
  // voltage in mV
  float outputV = measure * 5000.0 / 1023;
  Serial.print("Output Voltaje = ");
  Serial.print(outputV);
  Serial.print(" mV   ");
  Serial.print(" \n");

  // flux density
  float magneticFlux = outputV * 53.33 - 133.3;
  Serial.print("Magnetic Flux Density = ");
  Serial.print(magneticFlux);
  Serial.print(" mT");
  Serial.print(" \n");
  delay(2000);
}