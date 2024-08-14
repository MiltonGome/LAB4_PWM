#include <Arduino.h>
#include "driver/ledc.h"

// Definiciones de pines y canales
#define canalServoPWM 4
#define freqServoPWM 50
#define resServoPWM 10

#define canalLed1PWM 0
#define canalLed2PWM 1
#define canalLed3PWM 2
#define freqLedPWM 5000
#define resLedPWM 10

#define B1pin 16
#define B2pin 17
#define B3pin 18
#define B4pin 5

#define PinServoPWM 23
#define ledPin1 22
#define ledPin2 21
#define ledPin3 19

// Arreglo de posiciones del servo
float posiciones[] = {51, 64, 89, 102};  // Valores correctos para PWM del servo

// Arreglo de intensidades de los LEDs (en escala de 0 a 1023 para 10 bits)
int intensidades[] = {0, 256, 512, 768, 1023};

// Variables para rastrear la posición actual en el arreglo y el LED seleccionado
volatile int servoIndex = 0;
volatile int ledIndex = 0;
volatile int intensidadIndex[] = {0, 0, 0}; // Índices de intensidad para cada LED
volatile bool modoEspecial = false;  // Indica si el modo especial está activo

// Inicialización de las funciones
void initPWM();
void IRAM_ATTR B1ISR();
void IRAM_ATTR B2ISR();
void IRAM_ATTR B3ISR();
void IRAM_ATTR B4ISR();
void actualizarModoEspecial();

void setup() {
  // Inicializar el PWM
  initPWM();

  // Configurar los pines de los botones como entradas con resistencia pull-up
  pinMode(B1pin, INPUT_PULLUP);
  pinMode(B2pin, INPUT_PULLUP);
  pinMode(B3pin, INPUT_PULLUP);
  pinMode(B4pin, INPUT_PULLUP);

  // Configurar los pines de los LEDs como salida
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);

  // Adjuntar las interrupciones a los botones
  attachInterrupt(digitalPinToInterrupt(B1pin), B1ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(B2pin), B2ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(B3pin), B3ISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(B4pin), B4ISR, FALLING);

  Serial.begin(115200);
}

void loop() {
  // El bucle principal se mantiene vacío
}

void IRAM_ATTR B1ISR() {
  // Incrementar el índice para avanzar en el arreglo
  servoIndex++;
  // Verificar si el índice supera el tamaño del arreglo y reiniciarlo a 0
  if (servoIndex >= sizeof(posiciones) / sizeof(posiciones[0])) {
    servoIndex = 0;
  }
  // Escribir la nueva posición al PWM del servo
  ledcWrite(canalServoPWM, posiciones[servoIndex]); // Mapear posición a 10 bits

  // Actualizar los LEDs si el modo especial está activo
  if (modoEspecial) {
    actualizarModoEspecial();
  }
}

void IRAM_ATTR B2ISR() {
  // Decrementar el índice para retroceder en el arreglo
  servoIndex--;
  // Verificar si el índice es menor que 0 y establecerlo al último índice
  if (servoIndex < 0) {
    servoIndex = (sizeof(posiciones) / sizeof(posiciones[0])) - 1;
  }
  // Escribir la nueva posición al PWM del servo
  ledcWrite(canalServoPWM, posiciones[servoIndex]); // Mapear posición a 10 bits

  // Actualizar los LEDs si el modo especial está activo
  if (modoEspecial) {
    actualizarModoEspecial();
  }
}

void IRAM_ATTR B3ISR() {
  // Cambiar al siguiente modo
  ledIndex++;
  // Verificar si el índice supera el número de LEDs (tres LEDs más modo especial)
  if (ledIndex > 3) {
    ledIndex = 0;
  }

  // Verificar si estamos en el modo especial
  if (ledIndex == 3) {
    modoEspecial = true;
    actualizarModoEspecial();
  } else {
    modoEspecial = false;
  }
}

void IRAM_ATTR B4ISR() {
  // Incrementar la intensidad del LED seleccionado
  intensidadIndex[ledIndex]++;
  // Verificar si el índice de intensidad supera el tamaño del arreglo y reiniciarlo a 0
  if (intensidadIndex[ledIndex] >= sizeof(intensidades) / sizeof(intensidades[0])) {
    intensidadIndex[ledIndex] = 0;
  }

  // Aplicar la nueva intensidad al LED seleccionado
  if (!modoEspecial) {  // Solo ajustar intensidades si no estamos en el modo especial
    switch (ledIndex) {
      case 0:
        ledcWrite(canalLed1PWM, intensidades[intensidadIndex[ledIndex]]);
        break;
      case 1:
        ledcWrite(canalLed2PWM, intensidades[intensidadIndex[ledIndex]]);
        break;
      case 2:
        ledcWrite(canalLed3PWM, intensidades[intensidadIndex[ledIndex]]);
        break;
    }
  }
}

void actualizarModoEspecial() {
  // Enciende los LEDs dependiendo de la posición del servo
  switch (servoIndex) {
    case 0:  // Primera posición
      digitalWrite(ledPin1, HIGH);
      digitalWrite(ledPin2, LOW);
      digitalWrite(ledPin3, LOW);
      break;
    case 1:  // Segunda y tercera posición
    case 2:
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, HIGH);
      digitalWrite(ledPin3, LOW);
      break;
    case 3:  // Cuarta posición
      digitalWrite(ledPin1, LOW);
      digitalWrite(ledPin2, LOW);
      digitalWrite(ledPin3, HIGH);
      break;
  }
}

void initPWM() {
  // Configurar el canal PWM para el servo
  ledcSetup(canalServoPWM, freqServoPWM, resServoPWM);
  // Asignar el pin del servo al canal PWM
  ledcAttachPin(PinServoPWM, canalServoPWM);
  // Inicializar la posición del servo
  ledcWrite(canalServoPWM, posiciones[servoIndex]);

  // Configurar los canales PWM para los LEDs
  ledcSetup(canalLed1PWM, freqLedPWM, resLedPWM);
  ledcSetup(canalLed2PWM, freqLedPWM, resLedPWM);
  ledcSetup(canalLed3PWM, freqLedPWM, resLedPWM);

  // Asignar los pines de los LEDs a sus canales PWM correspondientes
  ledcAttachPin(ledPin1, canalLed1PWM);
  ledcAttachPin(ledPin2, canalLed2PWM);
  ledcAttachPin(ledPin3, canalLed3PWM);

  // Inicializar las intensidades de los LEDs a cero
  ledcWrite(canalLed1PWM, intensidades[intensidadIndex[0]]);
  ledcWrite(canalLed2PWM, intensidades[intensidadIndex[1]]);
  ledcWrite(canalLed3PWM, intensidades[intensidadIndex[2]]);
}