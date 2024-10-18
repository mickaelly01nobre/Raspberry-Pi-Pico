#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#define SERVO_PIN 0 // Defina o pino conectado ao servo motor

/*
* PWM (Pulse Width Modulation): O sinal PWM é usado para controlar a rotação contínua do servo. 
* A largura do pulso (duty_us) é ajustada com base no valor de speed.
*
*  - duty_us = 1500 + (speed * 5);: Calcula o tempo em microssegundos durante o qual o pino GPIO 
*   deve estar em nível alto. O valor central é 1500 us, que pode ser ajustado para mais ou menos com base na velocidade.
*
*  - gpio_put(gpio_pin, 1);: Define o pino como alto.
*
*  - sleep_us(duty_us);: Mantém o pino alto pelo tempo calculado.
*
*  - gpio_put(gpio_pin, 0);: Define o pino como baixo.
*
*  - sleep_us(20000 - duty_us);: Mantém o pino baixo pelo restante do período de 20 ms do ciclo PWM.
*
*/
void set_servo_speed(uint gpio_pin, int speed) {
    // O valor de 'speed' vai de -200 (velocidade máxima em uma direção) a 200 (velocidade máxima na outra direção)
    uint duty_us = 1500 + (speed * 5);  // PWM centralizado em 1500us (neutro), com ajuste de +/- 500us
    gpio_put(gpio_pin, 1);
    sleep_us(duty_us);
    gpio_put(gpio_pin, 0);
    sleep_us(20000 - duty_us);  // Completa o período de 20ms
}

// Tarefa do FreeRTOS para controlar o servo de rotação contínua
void vServoTask(void *pvParameters) {
    gpio_init(SERVO_PIN);
    gpio_set_dir(SERVO_PIN, GPIO_OUT);

    while (1) {
        // Gira o servo continuamente em uma direção rápida
        set_servo_speed(SERVO_PIN, 200);
        vTaskDelay(pdMS_TO_TICKS(500)); // Gira por 1 segundo

        // Inverte a direção (opcional)
        set_servo_speed(SERVO_PIN, -200);
        vTaskDelay(pdMS_TO_TICKS(500)); // Gira por 1 segundo
    }
}

int main() {
    stdio_init_all();

    // Cria a tarefa para controlar o servo
    xTaskCreate(vServoTask, "ServoTask", 256, NULL, 1, NULL);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    // Loop infinito
    while (1) {}
}

