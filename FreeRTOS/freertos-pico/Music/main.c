#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "pico/time.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Defina os pinos do buzzer e LEDs
#define BUZZER_PIN 15 // Define o pino do buzzer como 15
#define POT_PIN 26 // Define o pino do potenciômetro como 26

// Frequências para sons graves
const float graves[] = {
    60.00, 80.00, 100.00, 120.00, 140.00, 150.00, 160.00
};
const uint32_t num_graves = sizeof(graves) / sizeof(graves[0]);
// Calcula o número de notas na música "Happy Birthday"


// Frequências para sons agudos
const float agudos[] = {
    2000, 4000, 6000, 8000, 12000, 14000, 16000
};
const uint32_t num_agudos = sizeof(agudos) / sizeof(agudos[0]);
// Calcula o número de notas na música "Ode to Joy"

// Defina os pinos dos LEDs
const uint32_t led_pins[] = { 2, 3, 4, 5, 6, 7, 8 };
// Define os pinos que os LEDs estão conectados

// Defina uma estrutura para a mensagem da fila
typedef struct {
    float frequency; // Frequência da nota
    uint32_t led_pin; // Pino do LED associado à nota
} NoteMessage;

// Crie uma fila para enviar mensagens entre as tarefas
QueueHandle_t note_queue; // Fila para mensagens de notas
QueueHandle_t adc_queue; // Fila para valores do ADC

// Tarefa de controle da música
TaskHandle_t music_task_handle = NULL; // Handle para a tarefa atual da música

// Função para inicializar o PWM manualmente
void setup_pwm() {
    gpio_init(BUZZER_PIN); // Inicializa o pino do buzzer
    gpio_set_dir(BUZZER_PIN, GPIO_OUT); // Define o pino do buzzer como saída
}

// Função para inicializar os LEDs
void setup_leds() {
    for (uint32_t i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); ++i) {
        gpio_init(led_pins[i]); // Inicializa cada pino de LED
        gpio_set_dir(led_pins[i], GPIO_OUT); // Define o pino do LED como saída
        gpio_put(led_pins[i], 0); // Inicializa os LEDs apagados
    }
}

/*
A função set_PWM gera um tom de uma frequência específica no buzzer por um determinado tempo usando 
PWM manualmente. Ela calcula o período e meio período da onda com base na frequência fornecida, alterna 
o estado do pino do buzzer entre ligado e desligado, e usa a função busy_wait_us para esperar o tempo 
necessário para criar a onda desejada. A função continua a gerar o tom até que o tempo especificado para a nota tenha passado.
*/

void set_PWM(float frequency, uint32_t duration_ms) {
    // O período do sinal é o inverso da frequência. Multiplicando a frequência por 
    // um milhão (para converter Hz em microssegundos), obtemos o período em microssegundos.
    uint32_t period = (uint32_t)(1000000 / frequency);  // Calcula o período do sinal PWM em microssegundos
    uint32_t half_period = period / 2;      // Calcula meio período em microssegundos
    // time_us_32() : ajuda a calcula a duracao de uma nota musical
    uint32_t start_time = time_us_32();     // Obtém a hora de início da nota

    while (time_us_32() - start_time < duration_ms * 1000) {
        gpio_put(BUZZER_PIN, 1);  // Liga o buzzer
        //A função busy_wait_us(half_period) é usada para gerar uma pausa precisa no código, em microsegundos. Usada no PWM
        busy_wait_us(half_period);  // Espera meio período
        gpio_put(BUZZER_PIN, 0);  // Desliga o buzzer
        busy_wait_us(half_period);  // Espera meio período
    }
}

// Função para acender e apagar o LED
void control_led(uint32_t led_pin, bool state) {
    gpio_put(led_pin, state ? 1 : 0); // Define o estado do LED
    if (state) {
        printf("LED no pino %d aceso\n", led_pin); // Imprime uma mensagem se o LED estiver aceso
    }
}

// Tarefa para ler o valor do ADC
void potenciometro(void *pvParameters) {
    uint16_t adc_value;

    while (1) {
        adc_select_input(0); // Seleciona o canal do ADC 0
        adc_value = adc_read(); // Lê o valor do ADC

        xQueueSend(adc_queue, &adc_value, portMAX_DELAY); // Envia o valor do ADC para a fila

        vTaskDelay(pdMS_TO_TICKS(50)); // Aguarda 50 ms
    }
}

// Função para tocar a música com sons graves
void graves_task(void *pvParameters) {
    for (uint32_t i = 0; i < num_graves; ++i) {
        NoteMessage message = {graves[i], led_pins[i % (sizeof(led_pins) / sizeof(led_pins[0]))]}; // Cria a mensagem com a nota e o LED associado
        xQueueSend(note_queue, &message, portMAX_DELAY); // Envia a mensagem para a fila
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500 ms antes de enviar a próxima nota
    }
    vTaskDelete(NULL); // Finaliza a tarefa quando terminar a música
}

// Função para tocar a música agudos
void agudos_task(void *pvParameters) {
    for (uint32_t i = 0; i < num_agudos; ++i) {
        NoteMessage message = {agudos[i], led_pins[i % (sizeof(led_pins) / sizeof(led_pins[0]))]}; // Cria a mensagem com a nota e o LED associado
        xQueueSend(note_queue, &message, portMAX_DELAY); // Envia a mensagem para a fila
        vTaskDelay(pdMS_TO_TICKS(500)); // Aguarda 500 ms antes de enviar a próxima nota
    }
    vTaskDelete(NULL); // Finaliza a tarefa quando terminar a música
}

/*
A função led_task gerencia o controle de um LED específico e a reprodução de um tom 
associado com base nas mensagens recebidas de uma fila. Ela desliga todos os outros LEDs, 
acende o LED específico, toca o tom associado e, finalmente, desliga o LED após a reprodução do tom
*/
void led_task(void *pvParameters) {
    uint32_t led_pin = *(uint32_t*)pvParameters; // Obtém o pino do LED a partir dos parâmetros
    NoteMessage message;
    
    while (true) {
        if (xQueueReceive(note_queue, &message, portMAX_DELAY)) { // Recebe uma mensagem da fila
            // Desliga todos os LEDs antes de acender o novo
            for (uint32_t i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); ++i) {
                if (led_pins[i] != led_pin) {
                    control_led(led_pins[i], false); // Desliga LEDs que não são o LED atual
                }
            }

            // Acende o LED específico
            control_led(led_pin, true);
            set_PWM(message.frequency, 400); // Toca a nota correspondente
            control_led(led_pin, false); // Desliga o LED após tocar a nota
        }
    }
}

/*
A função music_selector_task é responsável por selecionar e alternar entre diferentes músicas com base 
na leitura de um potenciômetro. Ela faz isso monitorando mudanças no valor do ADC, comparando-o com um limiar 
predefinido. Quando o valor do ADC ultrapassa ou cai abaixo desse limiar, a tarefa atual é interrompida, e a tarefa 
correspondente à nova música é criada. 
*/
void music_selector_task(void *pvParameters) {
    // Armazena o valor atual lido do potenciômetro ou sensor ADC.
    uint16_t adc_value;
    // Armazena o valor do ADC da última leitura para comparar se houve uma mudança significativa.
    static uint16_t last_adc_value = 0;
    // Define um limite (CHANGE_THRESHOLD) que será usado para determinar se a leitura do ADC 
    // ultrapassou um ponto de mudança. Esse valor define a sensibilidade para troca de música.
    const uint16_t CHANGE_THRESHOLD = 20; // Limite para troca de música

    while (true) {
        if (xQueueReceive(adc_queue, &adc_value, portMAX_DELAY)) { // Recebe o valor do ADC da fila
        // Se o valor do ADC ultrapassou o limiar (CHANGE_THRESHOLD) e o valor anterior estava abaixo do limiar, 
        // a tarefa atual (music_task_handle) é deletada (se existir), e uma nova tarefa (graves_task) é criada.
            if (adc_value > CHANGE_THRESHOLD && last_adc_value <= CHANGE_THRESHOLD) {
                // Troca para "sons graves"
                if (music_task_handle != NULL) {
                    vTaskDelete(music_task_handle); // Para a música atual
                }
                xTaskCreate(graves_task, "Graves", 256, NULL, 1, &music_task_handle); // Cria a tarefa para "sons graves"
                
                // Se o valor do ADC caiu abaixo do limiar e o valor anterior estava acima, a tarefa atual é deletada 
                // e uma nova tarefa (agudos_task) é criada.
            } else if (adc_value <= CHANGE_THRESHOLD && last_adc_value > CHANGE_THRESHOLD) {
                // Troca para agudos
                if (music_task_handle != NULL) {
                    vTaskDelete(music_task_handle); // Para a música atual
                }
                xTaskCreate(agudos_task, "Agudos", 256, NULL, 1, &music_task_handle); // Cria a tarefa para "sons agudos"
            }
	    // Atualiza last_adc_value com o valor atual do ADC para a próxima comparação
            last_adc_value = adc_value; 
        }
    }
}

int main() {
    stdio_init_all(); // Inicializa a comunicação serial
    setup_pwm(); // Inicializa o PWM
    setup_leds(); // Inicializa os LEDs
    adc_init(); // Inicializa o ADC
    adc_gpio_init(POT_PIN); // Inicializa o pino do potenciômetro para o ADC

    // Crie a fila para mensagens
    note_queue = xQueueCreate(10, sizeof(NoteMessage)); // Cria a fila para mensagens de notas
    adc_queue = xQueueCreate(10, sizeof(uint16_t)); // Cria a fila para valores do ADC

    if (note_queue == NULL || adc_queue == NULL) {
        printf("Falha ao criar a fila\n"); // Imprime uma mensagem de erro se não conseguir criar as filas
        return 1;
    }

    // Cria uma tarefa para cada LED
    for (uint32_t i = 0; i < sizeof(led_pins) / sizeof(led_pins[0]); ++i) {
        xTaskCreate(led_task, "LED Task", 256, &led_pins[i], 1, NULL); // Cria uma tarefa para cada LED
    }

    xTaskCreate(potenciometro, "Potenciometro", 256, NULL, 1, NULL); // Cria a tarefa para ler o ADC
    xTaskCreate(music_selector_task, "Music Selector Task", 256, NULL, 1, NULL); // Cria a tarefa para selecionar a música
    
    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();
    
    while (true) {
        // O loop principal permanece vazio
    }

    return 0; // O main nunca deve retornar aqui
}

