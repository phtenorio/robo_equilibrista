// include/mpu6050.h

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "filter.h"

// --- Constantes Físicas ---
#define GRAVITY 9.81f // Aceleração da gravidade em m/s^2

// --- Definições do MPU6050 ---
#define MPU6050_ADDR 0x68 // Endereço I2C do MPU6050 (AD0 baixo)

// Registradores do MPU6050
#define MPU6050_PWR_MGMT_1 0x6B    // Gerenciamento de Energia 1
#define MPU6050_GYRO_CONFIG 0x1B   // Configuração do Giroscópio
#define MPU6050_ACCEL_CONFIG 0x1C  // Configuração do Acelerômetro
#define MPU6050_ACCEL_XOUT_H 0x3B  // Registrador de saída do acelerômetro X (MSB)

// --- Pinos I2C para I2C1 (se forem fixos para este módulo) ---
#define MPU6050_I2C_SDA_PIN 2
#define MPU6050_I2C_SCL_PIN 3
#define MPU6050_I2C_INSTANCE i2c1 // Define qual instância I2C será usada

// --- Configurações de Fundo de Escala (Full Scale Range - FSR) ---
// Para Acelerômetro (MPU6050_ACCEL_CONFIG)
typedef enum {
    ACCEL_FS_2G = 0x00,  // ±2g  (Sensibilidade: 16384 LSB/g)
    ACCEL_FS_4G = 0x08,  // ±4g  (Sensibilidade: 8192 LSB/g)
    ACCEL_FS_8G = 0x10,  // ±8g  (Sensibilidade: 4096 LSB/g)
    ACCEL_FS_16G = 0x18  // ±16g (Sensibilidade: 2048 LSB/g)
} mpu6050_accel_fsr_t;

// Para Giroscópio (MPU6050_GYRO_CONFIG)
typedef enum {
    GYRO_FS_250DPS = 0x00,   // ±250 °/s (Sensibilidade: 131 LSB/°/s)
    GYRO_FS_500DPS = 0x08,   // ±500 °/s (Sensibilidade: 65.5 LSB/°/s)
    GYRO_FS_1000DPS = 0x10,  // ±1000 °/s (Sensibilidade: 32.8 LSB/°/s)
    GYRO_FS_2000DPS = 0x18   // ±2000 °/s (Sensibilidade: 16.4 LSB/°/s)
} mpu6050_gyro_fsr_t;

// --- Fatores de Sensibilidade (LSB/unidade) ---
// Estes são os valores inversos da sensibilidade, para converter raw para unidade real
// Eles devem corresponder às opções de FSR acima!
#define ACCEL_SENSITIVITY_2G   16384.0f
#define ACCEL_SENSITIVITY_4G   8192.0f
#define ACCEL_SENSITIVITY_8G   4096.0f
#define ACCEL_SENSITIVITY_16G  2048.0f

#define GYRO_SENSITIVITY_250DPS  131.0f
#define GYRO_SENSITIVITY_500DPS  65.5f
#define GYRO_SENSITIVITY_1000DPS 32.8f
#define GYRO_SENSITIVITY_2000DPS 16.4f

// --- Protótipos de Funções ---
/**
 * @brief Configura o sensor MPU6050 com o fundo de escala especificado para acelerômetro e giroscópio.
 * Esta função assume que o barramento I2C já foi inicializado.
 * @param accel_fsr Fundo de escala desejado para o acelerômetro.
 * @param gyro_fsr Fundo de escala desejado para o giroscópio.
 */
void mpu6050_configure_sensor(mpu6050_accel_fsr_t accel_fsr, mpu6050_gyro_fsr_t gyro_fsr); // <--- NOME NOVO!

/**
 * @brief Lê os dados brutos do acelerômetro e giroscópio do MPU6050.
 * @param accel Array de 3 int16_t para armazenar os dados do acelerômetro (X, Y, Z).
 * @param gyro Array de 3 int16_t para armazenar os dados do giroscópio (X, Y, Z).
 */
void mpu6050_read_raw_data(int16_t accel[3], int16_t gyro[3]);

/**
 * @brief Seta o handle da fila para a tarefa do MPU6050.
 * @param queue_handle O handle da fila para enviar os dados do acelerômetro.
 */
void mpu6050_set_data_queue(QueueHandle_t queue_handle);

/**
 * @brief Tarefa FreeRTOS para leitura e impressão dos dados do MPU6050.
 * Esta tarefa também realiza a conversão para unidades reais e envia o Z-accel para a fila.
 */
void vMPU6050Task(void *pvParameters);

#endif // MPU6050_H