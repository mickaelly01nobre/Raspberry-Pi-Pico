#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include <stdio.h>
#include "task.h"

// Definição dos pinos dos LEDs
#define LED1_PIN 15
#define LED2_PIN 16
#define BUZZER_PIN 17
#define RGB1_PIN 18
#define RGB2_PIN 19
#define RGB3_PIN 20

TaskHandle_t xTaskLED1;
TaskHandle_t xTaskLED2;

volatile uint32_t ulIdleTicks = 0U;
volatile uint32_t ulPreviousIdleTicks = 0U;
volatile float cpu_usage = 0.0f;

// Contadores específicos para cada tarefa
volatile uint32_t ulLED1Counter = 0;
volatile uint32_t ulLED2Counter = 0;
volatile uint32_t ulbuzzerCounter = 0;

// Função Idle Hook
void vApplicationIdleHook(void) {
    ulIdleTicks++;
}

// Função para calcular e atualizar o uso da CPU
void update_cpu_usage(void) {
    uint32_t ulCurrentIdleTicks = ulIdleTicks;
    cpu_usage = ((float)(ulCurrentIdleTicks - ulPreviousIdleTicks) / ulCurrentIdleTicks) * 100;
    ulPreviousIdleTicks = ulCurrentIdleTicks;
}

// Função para piscar o LED 1
void vTaskBlinkLED1(void *pvParameter) {

    // Configura o pino GPIO como saída
    gpio_init(LED1_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    int cont = 0;

    for (;;) {
        gpio_put(LED1_PIN, 1);
        vTaskDelay(500);
        gpio_put(LED1_PIN, 0);
        vTaskDelay(500);
        update_cpu_usage();
        ulLED1Counter++;
        printf("LED1 Task is running. Counter: %lu, CPU Usage: %.2f%%\n", ulLED1Counter, cpu_usage);
        cont++;
        if(cont == 15){
		gpio_put(LED2_PIN, 0);
	    	vTaskDelete(NULL);

    	}
    }
    }


// Função para piscar o LED 2
void vTaskBlinkLED2(void *pvParameter) {

    // Configura o pino GPIO como saída
    gpio_init(LED2_PIN);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    int cont = 0;
    
    for (;;) {
        gpio_put(LED2_PIN, 0);
        vTaskDelay(500);
        gpio_put(LED2_PIN, 1);
        vTaskDelay(500);
        ulLED2Counter++;
        printf("LED2 Task is running. Counter: %lu, CPU Usage: %.2f%%\n", ulLED2Counter, cpu_usage);
        cont++;
        if(cont == 10){
		gpio_put(LED2_PIN, 0);
	    	vTaskDelete(NULL);

    	}
    }
}

void buzzer_task(void *pvParameters) {
    // Configure o pino do buzzer como saída
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    int cont = 0;


    for (;;) {
        // Ative o buzzer
        gpio_put(BUZZER_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500)); // Buzzer ligado por 500 ms

        // Desative o buzzer
        gpio_put(BUZZER_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500)); // Buzzer desligado por 500 ms
        update_cpu_usage();
        ulbuzzerCounter++;
        printf("BUZZER Task is running. Counter: %lu, CPU Usage: %.2f%%\n", ulbuzzerCounter, cpu_usage);
        cont++; 
        if(cont == 5){
		gpio_put(BUZZER_PIN, 0);
	    	vTaskDelete(NULL);

    	}
    }
}
void vTaskBlinkRGB1(void *pvParameter) {

    // Configura o pino GPIO como saída
    gpio_init(RGB1_PIN);
    gpio_set_dir(RGB1_PIN, GPIO_OUT);
    int cont = 0;
    
    for (;;) {
        gpio_put(RGB1_PIN, 0);
        vTaskDelay(500);
        gpio_put(RGB1_PIN, 1);
        vTaskDelay(500);
        ulLED2Counter++;
        printf("RGB1 Task is running. Counter: %lu, CPU Usage: %.2f%%\n", ulLED2Counter, cpu_usage);
        cont++;

    }
}
void vTaskBlinkRGB2(void *pvParameter) {

    // Configura o pino GPIO como saída
    gpio_init(RGB2_PIN);
    gpio_set_dir(RGB2_PIN, GPIO_OUT);
    int cont = 0;
    
    for (;;) {
        gpio_put(RGB2_PIN, 0);
        vTaskDelay(500);
        gpio_put(RGB2_PIN, 1);
        vTaskDelay(500);
        ulLED2Counter++;
        printf("RGB2 Task is running. Counter: %lu, CPU Usage: %.2f%%\n", ulLED2Counter, cpu_usage);
        cont++;

    }
}
void vTaskBlinkRGB3(void *pvParameter) {

    // Configura o pino GPIO como saída
    gpio_init(RGB3_PIN);
    gpio_set_dir(RGB3_PIN, GPIO_OUT);
    int cont = 0;
    
    for (;;) {
        gpio_put(RGB3_PIN, 0);
        vTaskDelay(500);
        gpio_put(RGB3_PIN, 1);
        vTaskDelay(500);
        ulLED2Counter++;
        printf("RGB3 Task is running. Counter: %lu, CPU Usage: %.2f%%\n", ulLED2Counter, cpu_usage);
        cont++;

    }
}




int main() {
    stdio_init_all();

    // Cria as tarefas para piscar os LEDs
    xTaskCreate(vTaskBlinkLED1, "Blink LED1 Task", 256, NULL, 1, &xTaskLED1);
    xTaskCreate(vTaskBlinkLED2, "Blink LED2 Task", 256, NULL, 1, &xTaskLED2);
    xTaskCreate(buzzer_task, "Buzzer Task", 256, NULL, 1, NULL);
    xTaskCreate(vTaskBlinkRGB1, "RGB 1", 256, NULL, 1, NULL);
    xTaskCreate(vTaskBlinkRGB2, "RGB 1", 256, NULL, 1, NULL);
    xTaskCreate(vTaskBlinkRGB3, "RGB 1", 256, NULL, 1, NULL);
    

    // Inicializa o scheduler do FreeRTOS
    vTaskStartScheduler();
    



    // Nunca deve chegar aqui
    for (;;);

    return 0;
}

