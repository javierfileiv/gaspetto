#define Hall_Sensor_Pin_A0 A0
#define Hall_Sensor_Pin_A1 A1
#define Hall_Sensor_Pin_A2 A2
#define Hall_Sensor_Pin_A3 A3

#define Hall_Sensor_Gnd_Pin_0 4
#define Hall_Sensor_Gnd_Pin_1 5
#define Hall_Sensor_Gnd_Pin_2 2
#define Hall_Sensor_Gnd_Pin_3 3

#define GND_ON LOW
#define GND_OFF HIGH

#define SENSOR_PER_BLOCK 4

#define HALL_THRESHOLD_1 3500.0
#define HALL_THRESHOLD_2 1900.0
#define MEAN_VALUES_N 50

#define _MAKEDATA(n) data##n
#define MAKEDATA(n) _MAKEDATA(n)

#define PRINT_CSV 0

uint8_t hall_sensor_input[SENSOR_PER_BLOCK];
uint8_t hall_sensor_gnd[SENSOR_PER_BLOCK];
float voltage[SENSOR_PER_BLOCK];

static inline void enable_sensor(uint8_t i) {
  digitalWrite(hall_sensor_gnd[i], GND_ON);
  delay(50);
}

static inline void disable_sensor(uint8_t i) {
  digitalWrite(hall_sensor_gnd[i], GND_OFF);
  // delay(150);
}

void setup() {

  Serial.begin(9600);
  delay(150);

  Serial.print("setup\n");
  hall_sensor_input[0] = Hall_Sensor_Pin_A0;  // AO sensor to LEFT
  hall_sensor_input[1] = Hall_Sensor_Pin_A1;  // A1 sensor to DOWN
  hall_sensor_input[2] = Hall_Sensor_Pin_A2;  // A2 sensor to RIGTH
  hall_sensor_input[3] = Hall_Sensor_Pin_A3;  // A3 sensor to UP

  hall_sensor_gnd[0] = Hall_Sensor_Gnd_Pin_0;  // GND O sensor to LEFT
  hall_sensor_gnd[1] = Hall_Sensor_Gnd_Pin_1;  // GND 1 sensor to DOWN
  hall_sensor_gnd[2] = Hall_Sensor_Gnd_Pin_2;  // GND 2 sensor to RIGTH
  hall_sensor_gnd[3] = Hall_Sensor_Gnd_Pin_3;  // GND 3 sensor to UP

  for (int i = 0; i < SENSOR_PER_BLOCK; i++) {

    pinMode(hall_sensor_input[i], INPUT);
    pinMode(hall_sensor_gnd[i], OUTPUT);
    /* All sensor OFF. */
    digitalWrite(hall_sensor_gnd[i], GND_OFF);
    /*Write CSV header.*/
    // Serial.print("Sensor");
    // Serial.print(i);
    // if (i != SENSOR_PER_BLOCK - 1)
    // {
    // 	Serial.print(",");
    // }
    // else
    // {
    // 	Serial.print("\n");
    // }
  }
}

void loop() {
  uint8_t i = 0;
  uint16_t mean_values = 0;
  long measure = 0;

  for (i = 0; i < SENSOR_PER_BLOCK; i++) {
    enable_sensor(i);
    for (mean_values = 0; mean_values < MEAN_VALUES_N; mean_values++) {
      measure += analogRead(hall_sensor_input[i]);
    }
    disable_sensor(i);

    measure /= MEAN_VALUES_N;
    voltage[i] = measure * (5000.0 / 1023);

#if PRINT_CSV

    Serial.print(voltage[i]);
    if (i != SENSOR_PER_BLOCK - 1)
      Serial.print(", ");
    else
      Serial.print("\n");
#endif
    // Serial.print("\n");
  }

#if !PRINT_CSV

//   Serial.println("///////////////////");
  i = 0;
  if (voltage[0] > HALL_THRESHOLD_1 && voltage[1] > HALL_THRESHOLD_2 && voltage[2] < HALL_THRESHOLD_1 && voltage[3] > HALL_THRESHOLD_2) {
    Serial.println("LEFT");
  }

  i = 0;
  if (voltage[0] < HALL_THRESHOLD_1 && voltage[1] < HALL_THRESHOLD_2 && voltage[2] < HALL_THRESHOLD_1 && voltage[3] > HALL_THRESHOLD_2) {
    Serial.println("UP");
  }

  i = 0;
  if (voltage[0] < HALL_THRESHOLD_1 && voltage[1] > HALL_THRESHOLD_2 && voltage[2] > HALL_THRESHOLD_1 && voltage[3] > HALL_THRESHOLD_2) {
    Serial.println("RIGTH");
  }

  i = 0;
  if (voltage[0] < HALL_THRESHOLD_1 && voltage[1] > HALL_THRESHOLD_2 && voltage[2] < HALL_THRESHOLD_1 && voltage[3] < HALL_THRESHOLD_2) {
    Serial.println("DOWN");
  }

  i = 0;
  if (voltage[0] < HALL_THRESHOLD_1 && voltage[1] > HALL_THRESHOLD_2 && voltage[2] < HALL_THRESHOLD_1 && voltage[3] > HALL_THRESHOLD_2) {
//     Serial.println("NO DETECTION");
  }

//   Serial.println("+++++++++++++");
#endif

  delay(100);
}