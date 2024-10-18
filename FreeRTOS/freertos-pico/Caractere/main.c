#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#define LED_PIN 15

// Função para inicializar o LED
void setup_led() {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

// Função para ler um caractere do terminal
char read_char() {
    while (!stdio_usb_connected()) {
        // Aguarde até que a conexão USB esteja disponível
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return getchar(); // Lê um caractere do terminal USB
}

// Tarefa FreeRTOS para ler caracteres e controlar o LED
void uart_task(void *pvParameters) {
    setup_led();
    while (1) {
        char input = read_char(); // Lê um caractere do terminal
        if (input == '1') {
            gpio_put(LED_PIN, true); // Acende o LED
            printf("LED ON\n");
        } else if (input == '0') {
            gpio_put(LED_PIN, false); // Apaga o LED
            printf("LED OFF\n");
        } else {
            printf("Invalid input: %c\n", input);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para evitar sobrecarga
    }
}

int main() {
    stdio_init_all(); // Inicializa a comunicação padrão via USB

    xTaskCreate(uart_task, "UART Task", 256, NULL, 1, NULL);
    vTaskStartScheduler();
    
    while (1) {
        // O código nunca deve chegar aqui
    }
}

