#include <stdio.h>
#include <stdlib.h> // Inclua este cabeçalho para usar atof()
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
#define BUZZER_PIN 21 // Novo pino para o buzzer

// Constantes para controle de cor
#define DEFAULT_MAX_DISTANCE_CM 30          // Distância máxima padrão em cm
#define DISTANCE_RED_THRESHOLD 5    // Distância em cm para cor vermelha
#define DISTANCE_GREEN_THRESHOLD 10  // Distância em cm para cor verde
#define DISTANCE_BLUE_THRESHOLD 15   // Distância em cm para cor azul
#define DISTANCE_CYAN_THRESHOLD 20   // Distância em cm para cor ciano
#define DISTANCE_MAGENTA_THRESHOLD 25 // Distância em cm para cor magenta
#define DISTANCE_YELLOW_THRESHOLD 30 // Distância em cm para cor amarela

// Estrutura para dados do sensor
typedef struct {
    float distance_cm;
    bool valid;
} SensorData;

// Estrutura para a distância máxima
typedef struct {
    float max_distance_cm;
    bool valid; // Indica se a distância máxima foi configurada
} MaxDistanceData;

// Criação das filas e do mutex
QueueHandle_t sensor_queue;
QueueHandle_t max_distance_queue;
SemaphoreHandle_t mutex;

void set_rgb_color(uint8_t red, uint8_t green, uint8_t blue) {
    // Ajusta o LED RGB para a cor desejada
    gpio_put(RED_PIN, red);
    gpio_put(GREEN_PIN, green);
    gpio_put(BLUE_PIN, blue);
}
/*
 *	A função sensor_task realiza as seguintes tarefas:
 *
 *	1.Recebe a distância máxima configurada de uma fila.
 *	2.Mede a distância usando um sensor ultrassônico em um loop infinito.
 *	3.Processa a medição:
 *		-Se o pulso do sensor é inválido, define a distância como -1 e marca os dados como inválidos.
 *		-Caso contrário, calcula a distância e verifica se está dentro do limite máximo.
 *	4.Envia os dados do sensor para uma fila.
 *	5.Aguarda 1 segundo antes de realizar a próxima medição.
 *
*/
void sensor_task(void *pvParameters) {
    MaxDistanceData max_distance_data;
    
    // Recebe a distância máxima configurada
    if (xQueueReceive(max_distance_queue, &max_distance_data, portMAX_DELAY)) {
        float max_distance_cm = max_distance_data.max_distance_cm;
        
        while (true) {
            /* Envia um pulso ao sensor ultrassônico e aguarda a resposta. Esta função 
            deve retornar o tempo que o pulso levou para retornar, o que é usado para 
            calcular a distância. ECHO_PIN é o pino que recebe o sinal de eco do sensor.
            */
            uint32_t pulse_time = hcsr04_send_pulse_and_wait(ECHO_PIN);

            SensorData data;
            /*  Verifica se o tempo de pulso é maior que um valor de timeout (ECHO_TIMEOUT_US).
             Se for, significa que o eco não foi recebido a tempo, indicando que o objeto está fora do alcance.
            */
            if (pulse_time >= ECHO_TIMEOUT_US) {
             // Define a distância como -1, indicando que a medição está fora do alcance.
                data.distance_cm = -1; 
                //Marca os dados como inválidos
                data.valid = false;
            // Se o pulso foi recebido a tempo, calcula a distância com base no tempo do pulso
            } else {
                data.distance_cm = (pulse_time / 2.0f) / 29.1f;
                data.valid = (data.distance_cm <= max_distance_cm); // Verifica se a distância está dentro do limite
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
            if (xSemaphoreTake(mutex, portMAX_DELAY)) { // Tenta obter o mutex
                if (data.valid) {
                    float distance_cm = data.distance_cm;
                    printf("LED RGB: Distance: %.2f cm\n", distance_cm);

                    // Define a cor com base na distância
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
                        set_rgb_color(0, 0, 0); // Apaga o LED RGB se a distância for maior que o limite
                    }
                } else {
                    set_rgb_color(0, 0, 0); // Apaga o LED RGB
                }
                vTaskDelay(pdMS_TO_TICKS(500)); // Mantém o LED RGB aceso por 500 ms
                set_rgb_color(0, 0, 0); // Apaga o LED RGB após o tempo
                xSemaphoreGive(mutex); // Libera o mutex
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo antes de tentar novamente
        }
    }
}

void buzzer_task(void *pvParameters) {
    SensorData data;
    while (true) {
        if (xQueueReceive(sensor_queue, &data, portMAX_DELAY)) {
            if (xSemaphoreTake(mutex, portMAX_DELAY)) { // Tenta obter o mutex
                if (data.valid) {
                    float distance_cm = data.distance_cm;
                    printf("Buzzer: Distance: %.2f cm\n", distance_cm);
                    gpio_put(BUZZER_PIN, 1); // Ativa o buzzer
                    vTaskDelay(pdMS_TO_TICKS(500)); // Mantém o buzzer ativo por 500 ms
                    gpio_put(BUZZER_PIN, 0); // Desativa o buzzer
                }
                xSemaphoreGive(mutex); // Libera o mutex
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // Aguarda 1 segundo antes de tentar novamente
        }
    }
}

char read_char() {
    while (!stdio_usb_connected()) {
        // Aguarde até que a conexão USB esteja disponível
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    return getchar(); // Lê um caractere do terminal USB
}

void uart_task(void *pvParameters) {
    char input_buffer[10]; // Buffer para armazenar a entrada do usuário
    int index = 0; //  Um índice para rastrear a posição atual no buffer.
    // Inicia um loop infinito para que a tarefa continue a processar entradas enquanto o sistema estiver ativo.
    while (1) {
        char input = read_char(); // Lê um caractere do terminal

        // Se o caractere for '\n' (nova linha), processa a entrada
        if (input == '\n' || input == '\r') {
            // Garante que há algo no buffer para processar
            if (index > 0) {
                input_buffer[index] = '\0'; // Finaliza a string
                index = 0; // Reseta o índice para a próxima entrada

                // Converte o conteúdo do buffer (que deve ser uma string representando um número) para um valor float usando atof.
                float new_max_distance = atof(input_buffer);
                // Verifica se o valor convertido é positivo.
                if (new_max_distance > 0) {
                    // Cria uma estrutura MaxDistanceData com a nova distância máxima e marca como válida.
                    MaxDistanceData max_distance_data = { new_max_distance, true };
                    xQueueSend(max_distance_queue, &max_distance_data, portMAX_DELAY);
                    printf("Max distance updated to %.2f cm\n", new_max_distance);
                } else {
                    printf("Invalid distance value: %.2f\n", new_max_distance);
                }
            }
         // Verifica se o caractere lido é um dígito numérico.
        } else if (input >= '0' && input <= '9') {
            // Garante que o índice não ultrapasse o tamanho do buffer.
            if (index < sizeof(input_buffer) - 1) {
            	// Adiciona o dígito ao buffer e incrementa o índice.
                input_buffer[index++] = input;
            }
        } else {
            printf("Invalid input: %c\n", input);
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // Delay para evitar sobrecarga
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

    // Criação das filas e do mutex
    sensor_queue = xQueueCreate(10, sizeof(SensorData)); // Cria uma fila com 10 elementos
    max_distance_queue = xQueueCreate(1, sizeof(MaxDistanceData)); // Fila para a distância máxima
    mutex = xSemaphoreCreateMutex(); // Cria o mutex

    // Verifica se as filas e o mutex foram criados com sucesso
    if (sensor_queue == NULL || max_distance_queue == NULL || mutex == NULL) {
        printf("Failed to create queue or mutex.\n");
        return 1;
    }

    // Criação das tarefas
    xTaskCreate(sensor_task, "SensorTask", 256, NULL, 1, NULL);
    xTaskCreate(led_rgb_task, "LEDTask", 256, NULL, 1, NULL);
    xTaskCreate(buzzer_task, "BuzzerTask", 256, NULL, 1, NULL);
    xTaskCreate(uart_task, "UARTTask", 256, NULL, 1, NULL);

    vTaskStartScheduler();

    while (true) {
        // Main loop does nothing, FreeRTOS handles tasks
    }
}

