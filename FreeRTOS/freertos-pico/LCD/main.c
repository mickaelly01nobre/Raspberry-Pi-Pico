#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "lcd1602_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "pico/binary_info.h"  // Incluído para bi_decl e bi_2pins_with_func

#define I2C_ADDRESS 0x27  // Certifique-se de que este é o endereço correto do LCD
#define I2C_BAUDRATE 100000

// Função da tarefa que escreve mensagens no LCD
void lcd_task(void *pvParameters) {
    static char *message[] = {
        "RP2040 by", "Raspberry Pi",
        "A brand new", "microcontroller",
        "Twin core M0", "Full C SDK",
        "More power in", "your product",
        "More beans", "than Heinz!"
    };

    while (1) {
        for (uint m = 0; m < sizeof(message) / sizeof(message[0]); m += MAX_LINES) {
            for (int line = 0; line < MAX_LINES; line++) {
                lcd_set_cursor(line, (MAX_CHARS / 2) - strlen(message[m + line]) / 2);
                lcd_string(message[m + line]);
            }
            vTaskDelay(pdMS_TO_TICKS(2000));  // Delay de 2 segundos
            lcd_clear();
        }
    }
}

int main() {
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/lcd_1602_i2c example requires a board with I2C pins
#else
    // Inicialização do I2C
    i2c_init(i2c_default, I2C_BAUDRATE);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    
    // Declaração dos pinos para o uso com picotool
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));

    // Inicializa o LCD
    lcd_init();

    // Cria a tarefa do LCD no FreeRTOS
    xTaskCreate(lcd_task, "LCD Task", 256, NULL, 1, NULL);

    // Inicia o agendador do FreeRTOS
    vTaskStartScheduler();

    // Caso o agendador falhe, o programa entra neste loop infinito
    while (1);
#endif
}

