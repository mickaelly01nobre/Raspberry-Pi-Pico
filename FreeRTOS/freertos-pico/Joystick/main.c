#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "FreeRTOS.h"
#include "task.h"

// Definindo os pinos do joystick
#define VRX_PIN 27
#define VRY_PIN 26
#define SW_PIN  22

// Função para leitura dos valores dos eixos e do botão
void joystick_task(void *pvParameters) {
    // Inicializa o ADC
    adc_init();
    adc_gpio_init(VRX_PIN);
    adc_gpio_init(VRY_PIN);

    // Inicializa o pino do botão com Pull-up
    gpio_init(SW_PIN);
    gpio_set_dir(SW_PIN, GPIO_IN);
    gpio_pull_up(SW_PIN);

    while (1) {
        // Seleciona o canal para leitura do eixo X
        adc_select_input(1); // Canal 1 -> GPIO27 (VRX)
        uint16_t xAxis = adc_read();

        // Seleciona o canal para leitura do eixo Y
        adc_select_input(0); // Canal 0 -> GPIO26 (VRY)
        uint16_t yAxis = adc_read();

        // Leitura do valor do botão
        uint8_t switch_state = gpio_get(SW_PIN);

        // Exibe os valores
        printf("X-axis: %d, Y-axis: %d, Switch: %d\n", xAxis, yAxis, switch_state);

        if (switch_state == 0) {
            printf("Botão de pressão pressionado!\n");
        }

        printf("\n");

        // Delay de 1 segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Função principal
int main() {
    stdio_init_all();

    // Cria a tarefa para o joystick
    xTaskCreate(joystick_task, "Joystick Task", 256, NULL, 1, NULL);

    // Inicia o agendador do FreeRTOS
    vTaskStartScheduler();

    // Loop infinito caso o agendador falhe (não deve chegar aqui)
    while (1);
}

