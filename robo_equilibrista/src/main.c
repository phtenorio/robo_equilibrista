#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "hardware_init.h"
#include "mpu6050.h"
#include "motor.h"

#define PICO_LED_PIN 12

// Para que main e as tarefas possam acessá-la
QueueHandle_t xAccelQueue;

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

    // --- Criação da Fila ---
    // Cria uma fila que pode armazenar 5 valores float
    xAccelQueue = xQueueCreate(5, sizeof(float));
    if (xAccelQueue == NULL) {
        printf("ERRO: Falha ao criar a fila do acelerômetro!\n");
        while(1); // Trava se a fila não puder ser criada
    }

    // --- Inicialização do Hardware ---
    i2c_bus_init(MPU6050_I2C_INSTANCE, MPU6050_I2C_SDA_PIN, MPU6050_I2C_SCL_PIN, 400 * 1000);

    // --- Inicialização e Configuração dos Sensores ---
    mpu6050_configure_sensor(ACCEL_FS_2G, GYRO_FS_250DPS);
    mpu6050_set_data_queue(xAccelQueue); // Passa o handle da fila para o módulo MPU6050


    // Inicializa os motores
    motor_init(&motor_left); // Inicializa o motor esquerdo
    motor_init(&motor_right); // Inicializa o motor direito
    motor_set_control_queue(xAccelQueue); // Passa o handle da fila para o módulo Motor

    // Tarefa para o MPU6050
    xTaskCreate(vMPU6050Task,
                "MPU6050_Reader",
                configMINIMAL_STACK_SIZE * 2,
                NULL,
                tskIDLE_PRIORITY + 2,
                NULL);

    // Tarefa para piscar LED
    xTaskCreate(vBlinkTask, "Blink",
                configMINIMAL_STACK_SIZE,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);

    // Tarefa de controle do motor
    xTaskCreate(vMotorControlTask,
                "Motor_Control",
                configMINIMAL_STACK_SIZE * 2,
                NULL,
                tskIDLE_PRIORITY + 1,
                NULL);


    // // Cria a tarefa de demonstração do motor com PWM
    // xTaskCreate(vMotorDemoTask,
    //             "Motor_Demo",
    //             configMINIMAL_STACK_SIZE * 2,
    //             NULL,
    //             tskIDLE_PRIORITY + 1,
    //             NULL);

    vTaskStartScheduler();

    while (true) {
       // Em caso de falha no scheduler
    }

    return 0;
}