#ifndef MOTOR_H
#define MOTOR_H

#include "pico/stdlib.h"
#include "hardware/pwm.h"

// Definição dos pinos GPIO para a ponte H L298N
#define MOTOR_DEFAULT_LEFT_IN1_PIN  8
#define MOTOR_DEFAULT_LEFT_IN2_PIN  9
#define MOTOR_DEFAULT_LEFT_ENA_PIN  4

#define MOTOR_DEFAULT_RIGHT_IN1_PIN 16
#define MOTOR_DEFAULT_RIGHT_IN2_PIN 18
#define MOTOR_DEFAULT_RIGHT_ENA_PIN 19


// Estrutura para representar um motor
typedef struct {
    uint in1_pin;
    uint in2_pin;
    uint ena_pin;
    uint slice_num; 
    uint channel;   
    const char *name;
} motor_t;

// Declaração das instâncias dos motores
extern motor_t motor_left;
extern motor_t motor_right;

/**
 * @brief Inicializa um motor específico.
 * Configura os pinos INx como saídas e o pino ENA para PWM.
 * @param motor Ponteiro para a estrutura do motor a ser inicializado.
 */
void motor_init(motor_t *motor);

/**
 * @brief Seta a direção de um motor específico.
 * @param motor Ponteiro para a estrutura do motor.
 * @param forward Se true, gira em uma direção. Se false, gira na direção oposta.
 */
void motor_set_direction(motor_t *motor, bool forward);

/**
 * @brief Para um motor específico (seta velocidade para 0 e desliga a direção).
 * @param motor Ponteiro para a estrutura do motor.
 */
void motor_stop(motor_t *motor);

/**
 * @brief Seta a velocidade de um motor específico usando PWM.
 * @param motor Ponteiro para a estrutura do motor.
 * @param speed Valor do duty cycle (0-255, onde 255 é velocidade máxima).
 */
void motor_set_speed(motor_t *motor, uint8_t speed);

void vMotorDemoTask(void *pvParameters);

#endif // MOTOR_H