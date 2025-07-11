#include "mpu6050.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"

// Variáveis globais
static float current_accel_scale_factor = ACCEL_SENSITIVITY_2G;
static float current_gyro_scale_factor = GYRO_SENSITIVITY_250DPS;

// --- Implementação da Tarefa MPU6050 ---
void vMPU6050Task(void *pvParameters) {
    (void) pvParameters;

    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    float accel_g[3];
    float gyro_dps[3];

    while (true) {
        mpu6050_read_raw_data(accel_raw, gyro_raw);

        // Converte para unidades reais usando os fatores de escala atuais
        accel_g[0] = (float)accel_raw[0] / current_accel_scale_factor;
        accel_g[1] = (float)accel_raw[1] / current_accel_scale_factor;
        accel_g[2] = (float)accel_raw[2] / current_accel_scale_factor;

        gyro_dps[0] = (float)gyro_raw[0] / current_gyro_scale_factor;
        gyro_dps[1] = (float)gyro_raw[1] / current_gyro_scale_factor;
        gyro_dps[2] = (float)gyro_raw[2] / current_gyro_scale_factor;

        printf("Acel: X=%7.2f g, Y=%7.2f g, Z=%7.2f g | ", accel_g[0], accel_g[1], accel_g[2]);
        printf("Giro: X=%7.2f dps, Y=%7.2f dps, Z=%7.2f dps\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// --- Implementação da Função de Configuração do MPU6050 ---
void mpu6050_configure_sensor(mpu6050_accel_fsr_t accel_fsr, mpu6050_gyro_fsr_t gyro_fsr) { // <--- NOME NOVO!
    uint8_t buf[2];

    // "Acorda" o MPU6050 e seleciona o clock do giroscópio X
    buf[0] = MPU6050_PWR_MGMT_1;
    buf[1] = 0x01; // PLL com giroscópio X
    i2c_write_blocking(MPU6050_I2C_INSTANCE, MPU6050_ADDR, buf, 2, false);
    printf("MPU6050: Acordado e clock configurado.\n");

    // Configuração do Fundo de Escala do Giroscópio
    buf[0] = MPU6050_GYRO_CONFIG;
    buf[1] = gyro_fsr;
    i2c_write_blocking(MPU6050_I2C_INSTANCE, MPU6050_ADDR, buf, 2, false);
    printf("MPU6050: Giroscópio configurado.\n");

    // Configuração do Fundo de Escala do Acelerômetro
    buf[0] = MPU6050_ACCEL_CONFIG;
    buf[1] = accel_fsr;
    i2c_write_blocking(MPU6050_I2C_INSTANCE, MPU6050_ADDR, buf, 2, false);
    printf("MPU6050: Acelerômetro configurado.\n");

    // Atualiza os fatores de escala para que a tarefa possa usá-los
    switch (accel_fsr) {
        case ACCEL_FS_2G:  current_accel_scale_factor = ACCEL_SENSITIVITY_2G; break;
        case ACCEL_FS_4G:  current_accel_scale_factor = ACCEL_SENSITIVITY_4G; break;
        case ACCEL_FS_8G:  current_accel_scale_factor = ACCEL_SENSITIVITY_8G; break;
        case ACCEL_FS_16G: current_accel_scale_factor = ACCEL_SENSITIVITY_16G; break;
        default: current_accel_scale_factor = ACCEL_SENSITIVITY_2G; break;
    }
    switch (gyro_fsr) {
        case GYRO_FS_250DPS:  current_gyro_scale_factor = GYRO_SENSITIVITY_250DPS; break;
        case GYRO_FS_500DPS:  current_gyro_scale_factor = GYRO_SENSITIVITY_500DPS; break;
        case GYRO_FS_1000DPS: current_gyro_scale_factor = GYRO_SENSITIVITY_1000DPS; break;
        case GYRO_FS_2000DPS: current_gyro_scale_factor = GYRO_SENSITIVITY_2000DPS; break;
        default: current_gyro_scale_factor = GYRO_SENSITIVITY_250DPS; break;
    }
}

void mpu6050_read_raw_data(int16_t accel[3], int16_t gyro[3]) {
    uint8_t buffer[14];

    uint8_t reg = MPU6050_ACCEL_XOUT_H;
    i2c_write_blocking(MPU6050_I2C_INSTANCE, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(MPU6050_I2C_INSTANCE, MPU6050_ADDR, buffer, 14, false);

    accel[0] = (int16_t)(buffer[0] << 8 | buffer[1]);
    accel[1] = (int16_t)(buffer[2] << 8 | buffer[3]);
    accel[2] = (int16_t)(buffer[4] << 8 | buffer[5]);

    gyro[0] = (int16_t)(buffer[8] << 8 | buffer[9]);
    gyro[1] = (int16_t)(buffer[10] << 8 | buffer[11]);
    gyro[2] = (int16_t)(buffer[12] << 8 | buffer[13]);
}