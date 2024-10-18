#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "ultrasonic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// Definições dos pinos
#define TRIG_PIN 2
#define ECHO_PIN 3
#define RED_PIN 17
#define GREEN_PIN 18
#define BLUE_PIN 19
#define BUZZER_PIN 21

// Constantes para controle de cor
#define DEFAULT_MAX_DISTANCE_CM 30
#define DISTANCE_RED_THRESHOLD 10
#define DISTANCE_GREEN_THRESHOLD 20
#define DISTANCE_BLUE_THRESHOLD 30
#define DISTANCE_CYAN_THRESHOLD 30
#define DISTANCE_MAGENTA_THRESHOLD 50
#define DISTANCE_YELLOW_THRESHOLD 60

// Estrutura para dados do sensor
typedef struct {
    float distance_cm;
    bool valid;
} SensorData;

// Estrutura para a distância máxima
typedef struct {
    float max_distance_cm;
    bool valid;
} MaxDistanceData;

// Criação das filas e do semáforo contador
QueueHandle_t sensor_queue;
QueueHandle_t max_distance_queue;
SemaphoreHandle_t sem_counter;

void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
    gpio_put(RED_PIN, red);
    gpio_put(GREEN_PIN, green);
    gpio_put(BLUE_PIN, blue);
}

void sensor_task(void *pvParameters) {
    MaxDistanceData max_distance_data;
    
    if (xQueueReceive(max_distance_queue, &max_distance_data, portMAX_DELAY)) {
        float max_distance_cm = max_distance_data.max_distance_cm;
        
        while (true) {
            uint32_t pulse_time = hcsr04_send_pulse_and_wait(ECHO_PIN);

            SensorData data;
            if (pulse_time >= ECHO_TIMEOUT_US) {
                data.distance_cm = -1; // Fora do alcance
                data.valid = false;
            } else {
                data.distance_cm = (pulse_time / 2.0f) / 29.1f;
                data.valid = (data.distance_cm <= max_distance_cm);
            }

            xQueueSend(sensor_queue, &data, portMAX_DELAY);

            vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo
        }
    }
}

void led_rgb_task(void *pvParameters) {
    SensorData data;
    while (true) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY)) {
            if (xSemaphoreTake(sem_counter, portMAX_DELAY)) {
                if (data.valid) {
                    float distance_cm = data.distance_cm;
                    printf("LED RGB: Distance: %.2f cm\n", distance_cm);

                    if (distance_cm < DISTANCE_RED_THRESHOLD) {
                        set_rgb_color(1, 0, 0); // Vermelho
                    } else if (distance_cm < DISTANCE_GREEN_THRESHOLD) {
                        set_rgb_color(0, 1, 0); // Verde
                    } else if (distance_cm < DISTANCE_BLUE_THRESHOLD) {
                        set_rgb_color(0, 0, 1); // Azul
                    } else if (distance_cm < DISTANCE_CYAN_THRESHOLD) {
                        set_rgb_color(0, 1, 1); // Ciano
                    } else if (distance_cm < DISTANCE_MAGENTA_THRESHOLD) {
                        set_rgb_color(1, 0, 1); // Magenta
                    } else if (distance_cm < DISTANCE_YELLOW_THRESHOLD) {
                        set_rgb_color(1, 1, 0); // Amarelo
                    } else {
                        set_rgb_color(0, 0, 0); // Desliga o LED RGB
                    }
                } else {
                    set_rgb_color(0, 0, 0); // Desliga o LED RGB
                }
                vTaskDelay(pdMS_TO_TICKS(500)); // Mantém o LED RGB aceso por 500 ms
                set_rgb_color(0, 0, 0); // Desliga o LED RGB após o tempo
                xSemaphoreGive(sem_counter); // Libera o semáforo
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo antes de tentar novamente
        }
    }
}

void buzzer_task(void *pvParameters) {
    SensorData data;
    while (true) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY)) {
            if (xSemaphoreTake(sem_counter, portMAX_DELAY)) {
                if (data.valid) {
                    float distance_cm = data.distance_cm;
                    printf("Buzzer: Distance: %.2f cm\n", distance_cm);
                    gpio_put(BUZZER_PIN, 1); // Ativa o buzzer
                    vTaskDelay(pdMS_TO_TICKS(500)); // Mantém o buzzer ativo por 500 ms
                    gpio_put(BUZZER_PIN, 0); // Desativa o buzzer
                }
                xSemaphoreGive(sem_counter); // Libera o semáforo
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo antes de tentar novamente
        }
    }
}

char read_char() {
    while (!stdio_usb_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return getchar(); // Lê um caractere do terminal USB
}

void uart_task(void *pvParameters) {
    char input_buffer[10]; 
    int index = 0;

    while (1) {
        char input = read_char(); 

        if (input == '\n' || input == '\r') {
            if (index > 0) {
                input_buffer[index] = '\0'; 
                index = 0; 

                float new_max_distance = atof(input_buffer);
                if (new_max_distance > 0) {
                    MaxDistanceData max_distance_data = { new_max_distance, true };
                    xQueueSend(max_distance_queue, &max_distance_data, portMAX_DELAY);
                    printf("Max distance updated to %.2f cm\n", new_max_distance);
                } else {
                    printf("Invalid distance value: %.2f\n", new_max_distance);
                }
            }
        } else if (input >= '0' && input <= '9') {
            if (index < sizeof(input_buffer) - 1) {
                input_buffer[index++] = input;
            }
        } else {
            printf("Invalid input: %c\n", input);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

int main() {
    stdio_init_all();
    hcsr04_init(TRIG_PIN, ECHO_PIN);

    // Configuração inicial do LED RGB
    gpio_init(RED_PIN);
    gpio_init(GREEN_PIN);
    gpio_init(BLUE_PIN);

    gpio_set_dir(RED_PIN, GPIO_OUT);
    gpio_set_dir(GREEN_PIN, GPIO_OUT);
    gpio_set_dir(BLUE_PIN, GPIO_OUT);

    // Configuração do buzzer
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);

    // Criação das filas e do semáforo contador
    sensor_queue = xQueueCreate(10, sizeof(SensorData));
    max_distance_queue = xQueueCreate(1, sizeof(MaxDistanceData));
    sem_counter = xSemaphoreCreateCounting(1, 1); // Criação do semáforo contador

    // Verificação se as filas e o semáforo foram criados com sucesso
    if (sensor_queue == NULL || max_distance_queue == NULL || sem_counter == NULL) {
        printf("Failed to create queue or semaphore.\n");
        return 1;
    }

    // Criação das tarefas
    xTaskCreate(sensor_task, "SensorTask", 256, NULL, 1, NULL);
    xTaskCreate(led_rgb_task, "LEDTask", 256, NULL, 1, NULL);
    xTaskCreate(buzzer_task, "BuzzerTask", 256, NULL, 1, NULL);
    xTaskCreate(uart_task, "UARTTask", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {
        // Loop principal não faz nada, o FreeRTOS gerencia as tarefas
    }
}

