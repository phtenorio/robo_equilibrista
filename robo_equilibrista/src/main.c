#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "hardware_init.h"
#include "mpu6050.h"

#define PICO_LED_PIN 12

void vBlinkTask(void *pvParameters) {
    (void) pvParameters;
    gpio_init(PICO_LED_PIN);
    gpio_set_dir(PICO_LED_PIN, GPIO_OUT);

    while (true) {
        gpio_put(PICO_LED_PIN, 1); // Liga LED
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_put(PICO_LED_PIN, 0); // Desliga LED
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


// --- Função Principal (main) ---
int main() {
    stdio_init_all();
    printf("Iniciando Robô Equilibrista com FreeRTOS...\n");


    // --- Inicialização do Hardware  ---
    i2c_bus_init(MPU6050_I2C_INSTANCE, MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN, 400 * 1000);

    // --- Inicialização e Configuração dos Sensores ---
    mpu6050_configure_sensor(ACCEL_FS_2G, GYRO_FS_250DPS);
    
    // Cria a tarefa para o MPU6050
    xTaskCreate(vMPU6050Task,
                "MPU6050_Reader",
                configMINIMAL_STACK_SIZE * 2,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    xTaskCreate(vBlinkTask, "Blink", 
                configMINIMAL_STACK_SIZE, 
                NULL, 
                tskIDLE_PRIORITY + 1,
                NULL);

    vTaskStartScheduler();

    while (true) {
       // Em caso de falha no scheduler
    }

    return 0;
}