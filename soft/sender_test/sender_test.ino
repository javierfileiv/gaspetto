#include <RF24.h>
#include <RF24_config.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <printf.h>

enum E_Command { MOTOR_STOP, MOTOR_FORWARD, MOTOR_BACKWARD, MOTOR_RIGHT, MOTOR_LEFT };

#define CE_PIN PB_15
#define CSN_PIN PIN_A4
#define DATA_RATE RF24_250KBPS

// Pines para el NRF24L01+ (ajusta según tu conexión en el transmisor)
RF24 radio(CE_PIN, CSN_PIN);
const uint64_t address[2] = { "gaBox", "gaCar" };

// Estructura para los comandos a enviar (debe coincidir con el receptor)
struct Command {
    int speed; // Valor PWM para la velocidad (0-255)
    E_Command direction; // 0: parar, 1: adelante, 2: atrás
};

Command commandToSend;

void setup()
{
    Serial.begin(115200);
    Serial.println("Iniciando Transmisor NRF24L01+");

    // initialize the transceiver on the SPI bus
    if (!radio.begin()) {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {
        } // hold in infinite loop
    }
    radio.openWritingPipe(address);
    radio.setPALevel(RF24_PA_LOW);
    radio.setDataRate(DATA_RATE);
    radio.setPayloadSize(sizeof(Command)); // float datatype occupies 4 bytes
    radio.stopListening(); // El transmisor no necesita escuchar
}

void loop()
{
    // Ejemplo de secuencia de comandos para probar el movimiento

    Serial.println("Enviando comando: Adelante, Velocidad 150");
    commandToSend.speed = 150;
    commandToSend.direction = MOTOR_FORWARD; // Adelante
    radio.write(&commandToSend, sizeof(commandToSend));
    delay(2000); // Esperar 2 segundos

    Serial.println("Enviando comando: Parar");
    commandToSend.speed = 0;
    commandToSend.direction = MOTOR_STOP; // Parar
    radio.write(&commandToSend, sizeof(commandToSend));
    delay(1000); // Esperar 1 segundo

    Serial.println("Enviando comando: Atrás, Velocidad 100");
    commandToSend.speed = 100;
    commandToSend.direction = MOTOR_BACKWARD; // Atrás
    radio.write(&commandToSend, sizeof(commandToSend));
    delay(2000); // Esperar 2 segundos

    Serial.println("Enviando comando: Parar");
    commandToSend.speed = 0;
    commandToSend.direction = MOTOR_STOP; // Parar
    radio.write(&commandToSend, sizeof(commandToSend));
    delay(1000); // Esperar 1 segundo
}
