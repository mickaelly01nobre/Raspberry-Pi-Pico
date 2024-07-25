#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include <stdio.h>
#include "task.h"

// Definição dos pinos dos LEDs
#define LED1 15
#define LED2 16

TaskHandle_t xTaskLED1;
TaskHandle_t xTaskLED2;

// Função para piscar o LED 1
void vTaskBlinkLED1(void *pvParameter) {
    for (;;) {
        gpio_put(LED1, 1);
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500ms
        gpio_put(LED1, 0);
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500ms
    }
}

// Função para piscar o LED 2
void vTaskBlinkLED2(void *pvParameter) {
    int cont = 0;
    for (;;) {
        gpio_put(LED2, 0);
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500ms
        gpio_put(LED2, 1);
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500ms
        cont++;
         if(cont == 4){
        gpio_put(LED2, 0);
    	vTaskDelete(NULL);

    	}
    }

}

int main() {
    stdio_init_all();

    // Inicializa o sistema de GPIO
    gpio_init(LED1);
    gpio_init(LED2);

    // Configura os pinos dos LEDs como saída
    gpio_set_dir(LED1, GPIO_OUT);
    gpio_set_dir(LED2, GPIO_OUT);

    // Cria as tarefas para piscar os LEDs
    xTaskCreate(vTaskBlinkLED1, "Blink LED 1", 256, NULL, 1, &xTaskLED1);
    xTaskCreate(vTaskBlinkLED2, "Blink LED 2", 256, NULL, 1, &xTaskLED2);
    
    // Após o agendador iniciar, defina a prioridade


    // Inicializa o scheduler do FreeRTOS
    vTaskStartScheduler();



    // Para evitar que o código continue, use um loop infinito
    for (;;);

    return 0;
}

