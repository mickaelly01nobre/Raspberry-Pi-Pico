#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Pinos dos LEDs e botões
#define LED_PIN_1 20
#define BUTTON_PIN_1 22
#define LED_PIN_2 21
#define BUTTON_PIN_2 26

// Identificadores para as tarefas de botões e LEDs
TaskHandle_t taskButtonHandles[2];
TaskHandle_t taskLedHandles[2];

//Identificador para o semáforo que controla o acesso aos LEDs
SemaphoreHandle_t semaphoreLed;

// Identificadores para as filas de comunicação entre as tarefas de botões e LEDs.
QueueHandle_t queueLed[2];


// Tempo de debounce para evitar múltiplas interrupções rápidas
#define DEBOUNCE_DELAY_MS 150


// Estrutura que armazena a configuração de cada LED e botão
typedef struct {
    uint32_t ledPin;
    uint32_t buttonPin;
    uint32_t taskIndex;
} LedButtonConfig;

// Array com as configurações para dois LEDs e botões
LedButtonConfig ledButtonConfigs[2] = {
    {LED_PIN_1, BUTTON_PIN_1, 0},
    {LED_PIN_2, BUTTON_PIN_2, 1}
};

/*
 * A função isrButtonHandler é chamada quando ocorre uma interrupção em um pino GPIO configurado. 
 * Ela usa um mecanismo de debounce para garantir que apenas interrupções válidas sejam processadas. 
 * Se a interrupção for válida, a função identifica qual botão causou a interrupção e notifica a tarefa 
 * associada para que ela possa processar o evento. Se uma tarefa de maior prioridade foi despertada, o 
 * sistema pode realizar uma troca de contexto para garantir que a tarefa com maior prioridade seja executada imediatamente.
 */

// uint gpio: O pino GPIO que gerou a interrupção
// uint32_t events: Máscara de eventos que descreve o tipo de interrupção (por exemplo, borda de subida, borda de descida).
void isrButtonHandler(uint gpio, uint32_t events) {
// Variável estática que armazena o tempo da última interrupção
    static uint32_t lastInterruptTime = 0;
// Armazena o tempo atual em milissegundos desde o início do boot.
    uint32_t currentTime = to_ms_since_boot(get_absolute_time());

    if (currentTime - lastInterruptTime > DEBOUNCE_DELAY_MS) {
        for (int i = 0; i < 2; i++) {
            if (gpio == ledButtonConfigs[i].buttonPin) {
                // Variável usada para indicar se uma troca de contexto de tarefa é necessária
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                //Notifica a tarefa associada ao botão pressionado que ocorreu uma interrupção.
                vTaskNotifyGiveFromISR(taskButtonHandles[i], &xHigherPriorityTaskWoken);
                // Se xHigherPriorityTaskWoken foi configurado como pdTRUE, 
                // a função portYIELD_FROM_ISR força uma troca de contexto para que uma 
                // tarefa de prioridade mais alta possa ser executada imediatamente.
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
                break;
            }
        }
    }
    lastInterruptTime = currentTime;
}
//  Espera por notificações da interrupção do botão e envia comandos para a fila correspondente ao LED
void taskButton(void *params) {
    // contém a configuração do LED e do botão associado.
    LedButtonConfig *config = (LedButtonConfig *)params;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint32_t ledCommand = 1; // Comando para alternar o LED
        if (xQueueSend(queueLed[config->taskIndex], &ledCommand, portMAX_DELAY) == pdPASS) {
            
        }
    }
}
/*
 * A função utiliza um semáforo para garantir que apenas uma tarefa pode acender um LED por vez 
 * e alterna o estado do LED baseado em comandos recebidos através da fila. Mensagens de depuração 
 * são usadas para fornecer feedback sobre o estado do semáforo e do LED.
 */
void taskLed(void *params) {
    LedButtonConfig *config = (LedButtonConfig *)params;
    uint32_t ledState = 0;
    while (1) {
        uint32_t ledCommand;
        if (xQueueReceive(queueLed[config->taskIndex], &ledCommand, portMAX_DELAY)) {
            if (ledState == 0) {
                if (xSemaphoreTake(semaphoreLed, 0) == pdTRUE) {
                    printf("Semáforo ocupado. Adquirido pelo LED %d\n", config->taskIndex);
                    ledState = 1;
                    gpio_put(config->ledPin, ledState);
                } else {
                    printf("Semáforo se encontra no limite da sua capacidade. LED %d não pode adquiri-lo\n", config->taskIndex);
                }
            } else {
                ledState = 0;
                gpio_put(config->ledPin, ledState);
                xSemaphoreGive(semaphoreLed);
                printf("Semáforo liberado pelo LED %d\n", config->taskIndex);
            }
        }
    }
}

int main() {
    stdio_init_all();
    for (int i = 0; i < 2; i++) {
        gpio_init(ledButtonConfigs[i].ledPin);
        gpio_set_dir(ledButtonConfigs[i].ledPin, GPIO_OUT);
        gpio_put(ledButtonConfigs[i].ledPin, 0);
        gpio_init(ledButtonConfigs[i].buttonPin);
        gpio_set_dir(ledButtonConfigs[i].buttonPin, GPIO_IN);
        gpio_pull_up(ledButtonConfigs[i].buttonPin);
    }

    semaphoreLed = xSemaphoreCreateCounting(1, 1);

    for (int i = 0; i < 2; i++) {
        queueLed[i] = xQueueCreate(1, sizeof(uint32_t));
    }

    if (semaphoreLed != NULL) {
        for (int i = 0; i < 2; i++) {
            gpio_set_irq_enabled_with_callback(ledButtonConfigs[i].buttonPin, GPIO_IRQ_EDGE_FALL, true, &isrButtonHandler);

            xTaskCreate(taskButton, "Task Botão", 256, &ledButtonConfigs[i], 2, &taskButtonHandles[i]);
            xTaskCreate(taskLed, "Task LED", 256, &ledButtonConfigs[i], 2, &taskLedHandles[i]);
        }

        vTaskStartScheduler();
    } else {
        printf("Falha ao criar o semáforo ou filas\n");
    }

    while (1) {
    }
    return 0;
}

