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
#define LED_R_PIN 18  // Pino do LED vermelho

#define LED_B_PIN 17  // Pino do LED azul
#define BUTTON_PIN 16  // Pino do botão
#define SERVO_PIN 0    // Pino do servo motor

// Constantes para indicar o status da leitura do DHT11
#define DHT11_OK 0
#define DHT11_ERROR_TIMEOUT 1
#define DHT11_ERROR_CHECKSUM 2

// Tamanho da fila para comunicação entre as tarefas
#define QUEUE_LENGTH 1

// Definições para a fila 
#define QUEUE_SIZE 1
#define SENSOR_CONTROL_QUEUE_LENGTH 1


// Semáforo binário para gerenciar interrupções do botão
SemaphoreHandle_t buttonSemaphore;

// Filas para comunicação entre as tarefas do LED, sensor de temperatura e controle do sensor.

QueueHandle_t tempQueue;
QueueHandle_t sensorControlQueue;

// Função para esperar microssegundos
void delay_us(uint32_t us) {
    sleep_us(us);
}

// Realiza a leitura de 1 byte do DHT11
// Le um byte de dados do sensor DHT11.
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
// Realiza a leitura da umidade e temperatura do DHT11, verificando se a leitura foi bem-sucedida (cálculo de checksum).
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

// Função para configurar os pinos do LED vemelho e azul
// Configura os pinos dos LEDs para saída e desliga-os inicialmente
void setup_led_pins() {
    gpio_init(LED_R_PIN);
    gpio_set_dir(LED_R_PIN, GPIO_OUT);
    gpio_init(LED_B_PIN);
    gpio_set_dir(LED_B_PIN, GPIO_OUT);

    // Inicialmente, apaga todas as cores
    gpio_put(LED_R_PIN, 0);
    gpio_put(LED_B_PIN, 0);
}

// Controla o estado dos LEDs vermelho e azul com base nos parâmetros
void set_led_color(bool red, bool blue) {
    gpio_put(LED_R_PIN, red);
    gpio_put(LED_B_PIN, blue);
}

/* Função de interrupção chamada quando o botão é pressionado. Ela libera um semáforo para notificar a tarefa correspondente.

 * Linha 1: BaseType_t higherPriorityTaskWoken = pdFALSE;
	- BaseType_t: Um tipo de dado definido pelo FreeRTOS, geralmente utilizado para indicar o status de uma operação 
	ou para controlar fluxos 				 		de execução.
       - higherPriorityTaskWoken: Variável que será utilizada para indicar se uma tarefa de prioridade mais alta foi 
       acordada como resultado da chamada a uma função de serviço de interrupção (ISR). Inicialmente, é definida como 
       pdFALSE, indicando que nenhuma tarefa de maior prioridade foi acordada.
       
  * Linha 2: xSemaphoreGiveFromISR(buttonSemaphore, &higherPriorityTaskWoken);
  
	- xSemaphoreGiveFromISR: Esta função libera (ou "dá") o semáforo buttonSemaphore de dentro de uma ISR. 
	Isso sinaliza que o evento esperado (neste caso, a interrupção do botão) ocorreu, e qualquer tarefa esperando
 	por esse semáforo poderá ser liberada.
 	-buttonSemaphore: O semáforo binário que está sendo liberado. Esse semáforo é utilizado em outra parte do 
 	código para sincronizar a tarefa que reinicia o sensor DHT11.
	-&higherPriorityTaskWoken: Endereço da variável que será atualizada pela função. Se a liberação do semáforo 
	resultar em uma tarefa de prioridade mais alta sendo acordada, essa variável será definida como pdTRUE.
	
  * Linha 3: portYIELD_FROM_ISR(higherPriorityTaskWoken);
  
	-portYIELD_FROM_ISR: Esta macro força uma mudança de contexto de dentro da ISR, caso a variável 
	higherPriorityTaskWoken tenha sido definida como pdTRUE. Em outras palavras, se a liberação do semáforo 
	acordou uma tarefa de maior prioridade, essa função garante que o sistema troque para essa tarefa imediatamente 
	após a ISR ser concluída.
	-higherPriorityTaskWoken: A variável que indica se uma tarefa de maior prioridade foi acordada. Se for pdTRUE,
	 o sistema realiza a troca de contexto para essa tarefa.
	

*/
void button_isr(uint gpio, uint32_t events) {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(buttonSemaphore, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

//  Tarefa que aguarda a interrupção do botão e reinicia o sensor DHT11 quando o botão é pressionado.
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

/* Tarefa que lê periodicamente a temperatura e umidade do DHT11. Se o botão for pressionado, 
 * a tarefa aguarda 2 segundos para estabilizar o sensor antes de realizar novas leituras.
 */
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
                for (int i = 0; i < 6; i++) { // Realiza 25 leituras
                    result = DHT11_Read(DHT11_PIN, umidade, temperatura);
                    if (result == DHT11_OK) {
                        temp = temperatura[0];
                        stable_readings++;
                    }
                    vTaskDelay(pdMS_TO_TICKS(2000)); // Aguarda 2 segundos entre as leituras
                }
                
                sensor_stable = (stable_readings >= 2);
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
        xSemaphoreGive(semaphoreLed);
    }
}
/*
* PWM (Pulse Width Modulation): O sinal PWM é usado para controlar a rotação contínua do servo. 
* A largura do pulso (duty_us) é ajustada com base no valor de speed.
*
*  - duty_us = 1500 + (speed * 5);: Calcula o tempo em microssegundos durante o qual o pino GPIO 
*   deve estar em nível alto. O valor central é 1500 us, que pode ser ajustado para mais ou menos com base na velocidade.
*
*  - gpio_put(gpio_pin, 1);: Define o pino como alto.
*
*  - sleep_us(duty_us);: Mantém o pino alto pelo tempo calculado.
*
*  - gpio_put(gpio_pin, 0);: Define o pino como baixo.
*
*  - sleep_us(20000 - duty_us);: Mantém o pino baixo pelo restante do período de 20 ms do ciclo PWM.
*
*/
void set_servo_speed(uint gpio_pin, int speed) {
    uint duty_us = 1500 + (speed * 5);
    gpio_put(gpio_pin, 1);
    sleep_us(duty_us);
    gpio_put(gpio_pin, 0);
    sleep_us(20000 - duty_us);
}


/* 
 *Tarefa que controla os LEDs com base na temperatura lida do DHT11 
 *(LED azul se a temperatura for abaixo de 26°C, LED vermelho se for 26°C ou mais) e acionar um servo motor e um buzzer.
 */

void vTaskLEDServo(void *pvParameters) {
    int temp;
    while (true) {
        if (xQueueReceive(tempQueue, &temp, portMAX_DELAY)) {
            if (temp < 20) {
                set_led_color(false, true); // Azul
            } else {
            	while(temp > 20){
		        set_led_color(true, false); // Vermelho
		        set_servo_speed(SERVO_PIN, 200);  // Aciona o servo motor
		        
		        vTaskDelay(pdMS_TO_TICKS(500));  // Mantém o movimento por meio segundo
		        set_servo_speed(SERVO_PIN, -200);   // Para o servo motor
		  
		        vTaskDelay(pdMS_TO_TICKS(500));  // Mantém o movimento por meio segundo
		}
            }
        }
    }
}

int main() {
    stdio_init_all();

    // Configura os pinos do LED RGB
    setup_led_pins();
    
    gpio_init(SERVO_PIN);
    gpio_set_dir(SERVO_PIN, GPIO_OUT);


    // Inicializa o hardware do botão
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
    xTaskCreate(vTaskLEDServo, "TaskLEDServo", 1024, NULL, 1, NULL);
    //xTaskCreate(vTaskLED, "TaskLED", 1024, NULL, 1, NULL);
    xTaskCreate(button_task, "ButtonTask", 256, NULL, 1, NULL);

    // Inicia o agendador do FreeRTOS
    vTaskStartScheduler();

    // O código não deve chegar aqui
    while (true) {
    }

    return 0;
}

