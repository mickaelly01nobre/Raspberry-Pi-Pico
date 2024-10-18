#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// Defina o pino do buzzer
#define BUZZER_PIN 1

void buzzer_task(void *pvParameters) {
    // Configura o pino do buzzer como saída
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    while (true) {
        // Alterna o estado do buzzer
        gpio_put(BUZZER_PIN, 1); // Liga o buzzer
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500 ms

        gpio_put(BUZZER_PIN, 0); // Desliga o buzzer
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500 ms
    }
}

int main() {
    stdio_init_all();

    // Cria a tarefa do buzzer
    xTaskCreate(buzzer_task, "Buzzer Task", 256, NULL, 1, NULL);

    // Inicia o agendador do FreeRTOS
    vTaskStartScheduler();

    // Nunca deve chegar aqui
    while (true) {}
}

