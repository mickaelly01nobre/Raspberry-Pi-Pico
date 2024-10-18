#include "mfrc522.h"
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define HEAP_THRESHOLD_PERCENTAGE 90  // 90% do heap ainda livre (queremos usar 10%)
#define LED_PIN 22                    // LED indicador de uso de memória
#define LED_GREEN_PIN 26              // LED verde para indicar desalocação
#define HEAP_TOTAL 120832             // Total do heap livre, conforme especificado
#define ALLOCATION_SIZE 3020          // Memória alocada por leitura (10% do total em 4 leituras)
#define MAX_READINGS 4                // Número máximo de leituras antes de desalocar

uint8_t *allocatedBlocks[MAX_READINGS];  // Armazenar os blocos alocados
int readingCount = 0; // Controlar o número de leituras

// Task to monitor heap status every 1 second
void vTaskHeapMonitor(void *pvParameters) {
    const size_t totalHeap = HEAP_TOTAL;
    const size_t threshold = (totalHeap * HEAP_THRESHOLD_PERCENTAGE) / 100;

    while (1) {
        size_t freeHeap = xPortGetFreeHeapSize();

        // Enviar o tamanho livre do heap para o terminal
        printf("Tamanho livre do heap: %u bytes\n\n", freeHeap);

        // Verifica se o tamanho livre está abaixo do limite de 90% (ou seja, 10% utilizado)
        if (freeHeap <= threshold) {
            printf("Heap atingiu 10%% do tamanho total alocado. \n\n");
            gpio_put(LED_PIN, 1);  // Acender o LED
        } else {
            gpio_put(LED_PIN, 0);  // Apagar o LED se a memória for maior que 90%
        }

        // Esperar 1 segundo
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Task to handle RFID card reading and allocate memory for each card detected
void vTaskRFIDHandler(void *pvParameters) {
    MFRC522Ptr_t mfrc = MFRC522_Init();
    PCD_Init(mfrc, spi0);
    

    while (1) {
        // Verificar se a leitura atual é a quinta leitura
        if (readingCount >= MAX_READINGS + 1) {
            printf(" Desalocando memória.\n\n");

            // Desalocar todos os blocos de memória
            for (int i = 0; i < MAX_READINGS; i++) {
                if (allocatedBlocks[i] != NULL) {
                    vPortFree(allocatedBlocks[i]);
                    allocatedBlocks[i] = NULL;
                }
            }

            // Acender o LED verde
            gpio_put(LED_GREEN_PIN, 1);

            // Esperar um tempo para mostrar a desalocação
            vTaskDelay(pdMS_TO_TICKS(2000));

            // Apagar o LED verde
            gpio_put(LED_GREEN_PIN, 0);

            // Bloquear novas leituras após a quinta
            continue;
        }

        // Esperar por um novo cartão
        printf("Aguardando cartão\n\n\r");
        while (!PICC_IsNewCardPresent(mfrc));

        // Selecionar o cartão
        printf("Selecionando cartão\n\n\r");
        PICC_ReadCardSerial(mfrc);

        // Verificar se a leitura é a quinta
        if (readingCount == MAX_READINGS) {
            // A quinta leitura, o sistema não aloca, mas espera para desalocar
            printf("Memoria não sera alocada.\n\n");
            readingCount++;  // Incrementar o contador de leituras para acionar a desalocação
            continue;
        }

        // Alocar memória (3020 bytes por leitura)
        uint8_t *memoryBlock = pvPortMalloc(ALLOCATION_SIZE);
        if (memoryBlock == NULL) {
            printf("Falha na alocação de memória\n\n\r");
            continue;
        }

        // Armazenar o bloco alocado
        allocatedBlocks[readingCount] = memoryBlock;

        // Incrementar o contador de leituras
        readingCount++;

        // Mostrar o UID no terminal
        printf("UID do cartão: ");
        for (int i = 0; i < mfrc->uid.size; i++) {
            printf("%x ", mfrc->uid.uidByte[i]);
        }
        printf("\n\r");

        // Delay entre leituras de cartões
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void main() {
    stdio_init_all();
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);  // Começar com o LED desligado

    gpio_init(LED_GREEN_PIN);
    gpio_set_dir(LED_GREEN_PIN, GPIO_OUT);
    gpio_put(LED_GREEN_PIN, 0);  // Começar com o LED verde desligado

    // Inicializar os blocos de memória como NULL
    for (int i = 0; i < MAX_READINGS; i++) {
        allocatedBlocks[i] = NULL;
    }

    // Criar a tarefa de monitoramento do heap
    xTaskCreate(vTaskHeapMonitor, "Heap Monitor", 256, NULL, 1, NULL);

    // Criar a tarefa de manipulação do RFID
    xTaskCreate(vTaskRFIDHandler, "RFID Handler", 512, NULL, 1, NULL);

    // Iniciar o agendador do FreeRTOS
    vTaskStartScheduler();

    // O programa nunca deve chegar aqui
    while (1) {
    }
}

