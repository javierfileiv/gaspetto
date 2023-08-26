#define Hall_Sensor_Pin_A0 A0
#define Hall_Sensor_Pin_A1 A1
#define Hall_Sensor_Pin_A2 A2
#define Hall_Sensor_Pin_A3 A3
#define SENSOR_PER_BLOCK 4

#define HALL_THRESHOLD 250.0
#define MEAN_VALUES_N 100

#define _MAKEDATA(n) data##n
#define MAKEDATA(n) _MAKEDATA(n)

uint8_t hall_sensor_input[SENSOR_PER_BLOCK];
float voltage[SENSOR_PER_BLOCK];

void setup() {
  hall_sensor_input[0] = Hall_Sensor_Pin_A0;  // AO sensor to LEFT
  hall_sensor_input[1] = Hall_Sensor_Pin_A1;  // A1 sensor to UP
  hall_sensor_input[2] = Hall_Sensor_Pin_A2;  // A2 sensor to RIGTH
  hall_sensor_input[3] = Hall_Sensor_Pin_A3;  // A3 sensor to DOWN

  for (int i = 0; i < SENSOR_PER_BLOCK; i++) {
    pinMode(hall_sensor_input[i], INPUT);
  }

  Serial.begin(9600);
}

void loop() {
  int i = 0, mean_values;
  for (i = 0; i < SENSOR_PER_BLOCK; i++) {
    for (mean_values = 0; mean_values < MEAN_VALUES_N; mean_values++) {
      voltage[i] += analogRead(hall_sensor_input[i]);
    }
    voltage[i] /= MEAN_VALUES_N;
    Serial.print("Sensor" + String(i) + " value:");
    Serial.println(voltage[i]);
  }


  Serial.println("///////////////////");
  i = 0;
  if (hall_sensor_input[i] > HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD) {
    Serial.println("LEFT");
  }

  i = 0;
  if (hall_sensor_input[i] < HALL_THRESHOLD && hall_sensor_input[i++] > HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD) {
    Serial.println("UP");
  }

  i = 0;
  if (hall_sensor_input[i] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] > HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD) {
    Serial.println("RIGTH");
  }

  i = 0;
  if (hall_sensor_input[i] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] > HALL_THRESHOLD) {
    Serial.println("DOWN");
  }

  i = 0;
  if (hall_sensor_input[i] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD && hall_sensor_input[i++] < HALL_THRESHOLD) {
    Serial.println("NO DETECTION");
  }

  Serial.println("+++++++++++++");

  delay(50);
}