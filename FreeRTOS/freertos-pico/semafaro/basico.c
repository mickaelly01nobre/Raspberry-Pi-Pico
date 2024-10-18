#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

// Definições dos pinos do DHT11, LED RGB e botão
#define DHT11_PIN 2
#define LED_R_PIN 17  // Pino do LED vermelho

#define LED_B_PIN 18  // Pino do LED azul
#define BUTTON_PIN 16  // Pino do botão

// Define os códigos de erro e sucesso
#define DHT11_OK 0
#define DHT11_ERROR_TIMEOUT 1
#define DHT11_ERROR_CHECKSUM 2

// Tamanho da fila para comunicação entre as tarefas
#define QUEUE_LENGTH 1

// Definições para a fila e o comando do LED verde
#define QUEUE_SIZE 1
#define SENSOR_CONTROL_QUEUE_LENGTH 1


// Semáforo binário
SemaphoreHandle_t buttonSemaphore;

// Filas para enviar comandos para as tarefas do LED
QueueHandle_t tempQueue;
QueueHandle_t sensorControlQueue;

// Função para esperar microssegundos
void delay_us(uint32_t us) {
    sleep_us(us);
}

// Realiza a leitura de 1 byte do DHT11
static char DHT11_Get_Byte(uint gpio) {
    char i, byte = 0;
    uint32_t timeOut;

    for (i = 0b10000000; i; i >>= 1) {
        timeOut = 0xFFFF;
        while (!gpio_get(gpio))
            if (!--timeOut) return 0;

        delay_us(40);

        if (gpio_get(gpio)) {
            byte |= i;
            timeOut = 0xFFFF;
            while (gpio_get(gpio))
                if (!--timeOut) return 0;
        }
    }

    return byte;
}

// Função para ler a umidade e temperatura do DHT11
char DHT11_Read(uint gpio, char *umidade, char *temperatura) {
    char checksum = 0;
    uint32_t timeOut = 0xFFFF;

    gpio_init(gpio);
    gpio_set_dir(gpio, GPIO_OUT);
    gpio_put(gpio, 0);
    vTaskDelay(pdMS_TO_TICKS(20)); // Sinal de start

    gpio_put(gpio, 1);
    gpio_set_dir(gpio, GPIO_IN);
    delay_us(60);

    while (!gpio_get(gpio))
        if (!--timeOut) return DHT11_ERROR_TIMEOUT;
    timeOut = 0xFFFF;
    while (gpio_get(gpio))
        if (!--timeOut) return DHT11_ERROR_TIMEOUT;

    umidade[0] = DHT11_Get_Byte(gpio);
    umidade[1] = DHT11_Get_Byte(gpio);
    temperatura[0] = DHT11_Get_Byte(gpio);
    temperatura[1] = DHT11_Get_Byte(gpio);
    checksum = DHT11_Get_Byte(gpio);

    if ((char)(umidade[0] + umidade[1] + temperatura[0] + temperatura[1]) != checksum)
        return DHT11_ERROR_CHECKSUM;

    return DHT11_OK;
}

// Função para configurar os pinos do LED RGB
void setup_led_pins() {
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);

    // Inicialmente, apaga todas as cores
    gpio_put(LED_R_PIN, 0);
    gpio_put(LED_B_PIN, 0);
}

// Função para acender o LED com a cor adequada
void set_led_color(bool red, bool blue) {
    gpio_put(LED_R_PIN, red);
    gpio_put(LED_B_PIN, blue);
}

// Função de interrupção para o botão
void button_isr(uint gpio, uint32_t events) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

// Tarefa do botão para reiniciar o sensor DHT11
void button_task(void *pvParameters) {
    while (1) {
        // Espera pelo semáforo binário
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            // Envia um comando para reiniciar o sensor DHT11
            bool restart_sensor = true;
            xQueueSend(sensorControlQueue, &restart_sensor, portMAX_DELAY);
        }
    }
}

// Tarefa do FreeRTOS para ler o DHT11 e enviar a temperatura pela fila
void vTaskDHT11(void *pvParameters) {
    char umidade[2], temperatura[2];
    char result;
    int temp;
    bool sensor_stable = true;

    while (true) {
        // Verifica se há um comando para reiniciar o sensor
        bool restart_sensor;
        if (xQueueReceive(sensorControlQueue, &restart_sensor, portMAX_DELAY)) {
            if (restart_sensor) {
                // Espera 2 segundos após reiniciar para estabilizar o sensor
                vTaskDelay(pdMS_TO_TICKS(2000));
                
                // Período de pré-leitura para garantir que o sensor esteja estável
                int stable_readings = 0;
                for (int i = 0; i < 25; i++) { // Realiza 10 leituras
                    result = DHT11_Read(DHT11_PIN, umidade, temperatura);
                    if (result == DHT11_OK) {
                        temp = temperatura[0];
                        stable_readings++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundos entre as leituras
                }
                
                sensor_stable = (stable_readings >= 4);
            }
        }

        if (sensor_stable) {
            result = DHT11_Read(DHT11_PIN, umidade, temperatura);
            if (result == DHT11_OK) {
                // Exibe os dados no terminal
                printf("Umidade: %d.%d%%\n", umidade[0], umidade[1]);
                printf("Temperatura: %d.%d°C\n", temperatura[0], temperatura[1]);

                // Envia a temperatura para a fila após a leitura bem-sucedida
                temp = temperatura[0];
                xQueueSend(tempQueue, &temp, portMAX_DELAY);
            } else if (result == DHT11_ERROR_TIMEOUT) {
                printf("Erro: Timeout na leitura do sensor DHT11\n");
            } else if (result == DHT11_ERROR_CHECKSUM) {
                printf("Erro: Checksum inválido na leitura do sensor DHT11\n");
            }
        } else {
            printf("Sensor DHT11 não está estável. Aguardando...\n");
            vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda mais 2 segundos antes da próxima leitura
        }

        vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundos para próxima leitura
    }
}

// Tarefa do FreeRTOS para controlar o LED RGB com base na temperatura
void vTaskLED(void *pvParameters) {
    int temp;

    while (true) {
        // Recebe a temperatura da fila
        if (xQueueReceive(tempQueue, &temp, portMAX_DELAY)) {
            // Acende o LED em azul se a temperatura for abaixo de 27°C
            // Acende o LED em vermelho se a temperatura for 27°C ou mais
            if (temp < 27) {
                set_led_color(false, true); // Azul
            } else {
                set_led_color(true, false); // Vermelho
            }
        }
    }
}

int main() {
    stdio_init_all();

    // Configura os pinos do LED RGB
    setup_led_pins();

    // Inicializa o hardware do botão e do LED verde
    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_set_irq_enabled_with_callback(BUTTON_PIN, GPIO_IRQ_EDGE_FALL, true, button_isr);

    // Cria o semáforo binário para o botão
    buttonSemaphore = xSemaphoreCreateBinary();
    if (buttonSemaphore == NULL) {
        printf("Erro ao criar o semáforo.\n");
        return 1;
    }

    // Cria as filas para comunicação entre as tarefas
    tempQueue = xQueueCreate(QUEUE_LENGTH, sizeof(int));
    sensorControlQueue = xQueueCreate(SENSOR_CONTROL_QUEUE_LENGTH, sizeof(bool));
    if ( tempQueue == NULL || sensorControlQueue == NULL) {
        printf("Erro ao criar a fila.\n");
        return 1;
    }

    // Cria as tarefas do FreeRTOS
    xTaskCreate(vTaskDHT11, "TaskDHT11", 1024, NULL, 1, NULL);
    xTaskCreate(vTaskLED, "TaskLED", 1024, NULL, 1, NULL);
    xTaskCreate(button_task, "ButtonTask", 256, NULL, 1, NULL);

    // Inicia o agendador do FreeRTOS
    vTaskStartScheduler();

    // O código não deve chegar aqui
    while (true) {
    }

    return 0;
}

