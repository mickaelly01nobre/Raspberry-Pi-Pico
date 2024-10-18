#ifndef ultrasonic_h
#define ultrasonic_h

#include "pico/stdlib.h"

// Define o timeout para a leitura do eco em microssegundos
#define ECHO_TIMEOUT_US (500 * 2 * 30)

// Inicializa o sensor HC-SR04 com os pinos fornecidos
void hcsr04_init(uint trigger_pin, uint echo_pin);

// Obtém o tempo do pulso do sensor HC-SR04
uint32_t hcsr04_send_pulse_and_wait(uint echo_pin);

// Calcula a distância em milímetros
float hcsr04_distance_mm(void);

// Calcula a distância em centímetros
float hcsr04_distance_cm(void);

#endif
