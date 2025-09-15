// Este código lee el estado de 80 sensores Hall utilizando 10 chips multiplexores 74HC4051
// en un microcontrolador STM32F103C8T6 (Blue Pill).

// --- Definición de pines para el Blue Pill ---
// Pin ADC para leer la salida de los multiplexores
const int SENSOR_ADC_PIN = PA0;

// Pines GPIO para seleccionar el canal (S0, S1, S2)
// Estos pines controlan los 8 canales de todos los multiplexores simultáneamente
const int MUX_S0_PIN = PA1;
const int MUX_S1_PIN = PA2;
const int MUX_S2_PIN = PA3;

// Pines GPIO para habilitar cada uno de los 10 multiplexores
// Solo uno de estos pines estará en LOW a la vez para seleccionar el chip activo
const int MUX_ENABLE_PINS[10] = { PA4, PA5, PA6, PA7, PB0, PB1, PB2, PB3, PB4, PB5 };

// --- Constantes del sistema ---
const int NUM_HOLES = 20;
const int SENSORS_PER_HOLE = 4;
const int TOTAL_SENSORS = NUM_HOLES * SENSORS_PER_HOLE;
const int NUM_MULTIPLEXERS = 10;
const int CHANNELS_PER_MUX = 8;
const int ADC_THRESHOLD = 512; // Umbral para convertir de analógico a digital. Ajusta este valor.

// --- Variables para almacenar los estados de los sensores ---
// Se almacena el estado de cada uno de los 80 sensores
uint8_t sensor_states[TOTAL_SENSORS];

// --- Configuración inicial del Arduino ---
void setup()
{
    Serial.begin(115200);

    // Configurar los pines de selección del multiplexor como salidas
    pinMode(MUX_S0_PIN, OUTPUT);
    pinMode(MUX_S1_PIN, OUTPUT);
    pinMode(MUX_S2_PIN, OUTPUT);

    // Configurar los pines de habilitación de los multiplexores como salidas
    for (int i = 0; i < NUM_MULTIPLEXERS; i++) {
        pinMode(MUX_ENABLE_PINS[i], OUTPUT);
        digitalWrite(MUX_ENABLE_PINS[i], HIGH); // Deshabilitar todos los chips (E = HIGH)
    }

    Serial.println("Inicialización del controlador de la caja completa.");
    Serial.println("Comenzando a escanear los 80 sensores.");
}

// --- Bucle principal del Arduino ---
void loop()
{
    readAllSensors();
    printSensorStates();
    delay(1000); // Esperar 1 segundo antes de la siguiente lectura
}

// --- Función para leer todos los 80 sensores ---
void readAllSensors()
{
    int sensor_index = 0;

    // Bucle a través de los 10 multiplexores
    for (int mux_id = 0; mux_id < NUM_MULTIPLEXERS; mux_id++) {
        // Deshabilitar todos los chips para evitar conflictos antes de habilitar el correcto
        for (int i = 0; i < NUM_MULTIPLEXERS; i++) {
            digitalWrite(MUX_ENABLE_PINS[i], HIGH);
        }

        // Habilitar el multiplexor actual (poner su pin E en LOW)
        digitalWrite(MUX_ENABLE_PINS[mux_id], LOW);

        // Bucle a través de los 8 canales de cada multiplexor
        for (int channel = 0; channel < CHANNELS_PER_MUX; channel++) {
            // Configurar los pines de selección para elegir el canal actual
            digitalWrite(MUX_S0_PIN, bitRead(channel, 0));
            digitalWrite(MUX_S1_PIN, bitRead(channel, 1));
            digitalWrite(MUX_S2_PIN, bitRead(channel, 2));

            // Esperar un breve momento para que la señal se estabilice
            delayMicroseconds(50);

            // Leer el valor del sensor a través del ADC
            int analog_value = analogRead(SENSOR_ADC_PIN);

            // Convertir la lectura analógica en un estado digital (0 o 1)
            if (analog_value > ADC_THRESHOLD) {
                sensor_states[sensor_index] = 1; // Sensor activo
            } else {
                sensor_states[sensor_index] = 0; // Sensor inactivo
            }

            sensor_index++;
        }

        // Deshabilitar el multiplexor actual antes de la siguiente iteración del bucle
        digitalWrite(MUX_ENABLE_PINS[mux_id], HIGH);
    }
}

// --- Función para imprimir los estados de los sensores en el Monitor Serie ---
void printSensorStates()
{
    Serial.println("--- Estados de los 80 sensores ---");
    for (int i = 0; i < TOTAL_SENSORS; i++) {
        Serial.print(sensor_states[i]);
        Serial.print("\t");
        // Salto de línea cada 4 sensores (para representar cada orificio)
        if ((i + 1) % SENSORS_PER_HOLE == 0) {
            Serial.print("  (Orificio ");
            Serial.print((i + 1) / SENSORS_PER_HOLE);
            Serial.println(")");
        }
    }
    Serial.println("----------------------------------");
}
