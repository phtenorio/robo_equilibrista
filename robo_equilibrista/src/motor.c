#include "motor.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdio.h>
#include <math.h>

// Definição das instâncias dos motores
motor_t motor_left = {
    .in1_pin = MOTOR_DEFAULT_LEFT_IN1_PIN,
    .in2_pin = MOTOR_DEFAULT_LEFT_IN2_PIN,
    .ena_pin = MOTOR_DEFAULT_LEFT_ENA_PIN,
    .name = "Motor Esquerdo"
};

motor_t motor_right = {
    .in1_pin = MOTOR_DEFAULT_RIGHT_IN1_PIN,
    .in2_pin = MOTOR_DEFAULT_RIGHT_IN2_PIN,
    .ena_pin = MOTOR_DEFAULT_RIGHT_ENA_PIN,
    .name = "Motor Direito"
};

// Variável para armazenar o handle da fila
static QueueHandle_t motor_control_queue = NULL; 

// --- Implementação da Função para Setar a Fila ---
void motor_set_control_queue(QueueHandle_t queue_handle) {
    motor_control_queue = queue_handle;
}

void motor_set_direction(motor_t *motor, bool forward) {
    if (forward) {
        gpio_put(motor->in1_pin, 1);
        gpio_put(motor->in2_pin, 0);
        printf("Motor '%s': Direção: Frente\n", motor->name);
    } else {
        gpio_put(motor->in1_pin, 0);
        gpio_put(motor->in2_pin, 1);
        printf("Motor '%s': Direção: Trás\n", motor->name);
    }
}

void motor_stop(motor_t *motor) {
    gpio_put(motor->in1_pin, 0);
    gpio_put(motor->in2_pin, 0);
    motor_set_speed(motor, 0);
    // printf("Motor '%s': Parado\n", motor->name);
}

void motor_init(motor_t *motor) {
    // Configura os pinos INx como saídas
    gpio_init(motor->in1_pin);
    gpio_set_dir(motor->in1_pin, GPIO_OUT);
    gpio_init(motor->in2_pin);
    gpio_set_dir(motor->in2_pin, GPIO_OUT);

    // Configura o pino ENA para PWM
    gpio_set_function(motor->ena_pin, GPIO_FUNC_PWM);
    motor->slice_num = pwm_gpio_to_slice_num(motor->ena_pin);
    motor->channel = pwm_gpio_to_channel(motor->ena_pin);
    pwm_set_clkdiv_int_frac(motor->slice_num, 1, 0);
    pwm_set_wrap(motor->slice_num, 255);
    pwm_set_enabled(motor->slice_num, true);

    // Garante que o motor esteja parado inicialmente
    motor_stop(motor);
    motor_set_speed(motor, 0);

    // printf("Motor '%s' inicializado nos pinos IN1=%d, IN2=%d, ENA=%d\n",
    //        motor->name, motor->in1_pin, motor->in2_pin, motor->ena_pin);
}

void motor_set_speed(motor_t *motor, uint8_t speed) {
    pwm_set_chan_level(motor->slice_num, motor->channel, speed);
    // printf("Motor '%s': Velocidade = %d\n", motor->name, speed); // Comentado para reduzir a saída
}

// --- Tarefa de Controle do Motor ---
void vMotorControlTask(void *pvParameters) { // Renomeado de vMotorDemoTask
    (void) pvParameters;

    float z_accel_value;
    uint8_t current_speed = 0;

    // Limiares de sensibilidade para acionamento
    const float FORWARD_THRESHOLD = -0.1f; // Se Z < -0.1, tombando para frente
    const float BACKWARD_THRESHOLD = 0.1f; // Se Z > 0.1, tombando para trás
    const float MAX_ACCEL_FOR_SPEED = 1.0f; // Aceleração máxima esperada para mapear para velocidade total

    while (true) {
        // Tenta receber um valor da fila. Espera por 10ms se a fila estiver vazia.
        if (motor_control_queue != NULL && xQueueReceive(motor_control_queue, &z_accel_value, pdMS_TO_TICKS(10)) == pdPASS) {

            // Lógica de controle simples baseada no eixo Z do acelerômetro
            if (z_accel_value < FORWARD_THRESHOLD) { // Tombando para frente
                motor_set_direction(&motor_left, true);
                motor_set_direction(&motor_right, true);

                // Mapeia a magnitude da inclinação para a velocidade
                // Limita o valor para evitar velocidades excessivas com leituras extremas
                float normalized_accel = fmin(fabs(z_accel_value), MAX_ACCEL_FOR_SPEED);
                current_speed = (uint8_t)(normalized_accel / MAX_ACCEL_FOR_SPEED * 255);
                current_speed = fmax(current_speed, 10); // Garante uma velocidade mínima para mover

                motor_set_speed(&motor_left, current_speed);
                motor_set_speed(&motor_right, current_speed);
                printf("Tombando p/ FRENTE (Z=%.2f). Motores FRENTE com velocidade %d\n", z_accel_value, current_speed);

            } else if (z_accel_value > BACKWARD_THRESHOLD) { // Tombando para trás
                motor_set_direction(&motor_left, false);
                motor_set_direction(&motor_right, false);

                // Mapeia a magnitude da inclinação para a velocidade
                float normalized_accel = fmin(fabs(z_accel_value), MAX_ACCEL_FOR_SPEED);
                current_speed = (uint8_t)(normalized_accel / MAX_ACCEL_FOR_SPEED * 255);
                current_speed = fmax(current_speed, 10); // Garante uma velocidade mínima para mover

                motor_set_speed(&motor_left, current_speed);
                motor_set_speed(&motor_right, current_speed);
                printf("Tombando p/ TRÁS (Z=%.2f). Motores TRÁS com velocidade %d\n", z_accel_value, current_speed);

            } else { // Robô está relativamente equilibrado
                motor_stop(&motor_left);
                motor_stop(&motor_right);
                // printf("Equilibrado (Z=%.2f). Motores PARADOS.\n", z_accel_value); // Comentado para não poluir o console
            }
        } else {
            // Se não recebeu nada da fila, aguarda um pouco antes de tentar novamente
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// void vMotorDemoTask(void *pvParameters) {
//     (void) pvParameters;

//     uint8_t speed = 0;
//     const uint8_t speed_step = 5;
//     const TickType_t delay_ms_per_step = pdMS_TO_TICKS(300);

//     while (true) {
//         // --- Demonstração de aumento de velocidade do Motor Direito (Motor B) para frente ---
//         printf("\nMotor Direito: Aumentando velocidade para frente...\n");
//         motor_set_direction(&motor_right, true);
//         motor_set_direction(&motor_left, true);
//         for (speed = 0; speed < 255; speed += speed_step) {
//             motor_set_speed(&motor_right, speed);
//             motor_set_speed(&motor_left, speed);
//             vTaskDelay(delay_ms_per_step);
//         }
//         printf("Motor Direito: Velocidade máxima (255) alcançada (Frente).\n");
//         vTaskDelay(pdMS_TO_TICKS(1500));

//         // --- Demonstração de diminuição de velocidade do Motor Direito para frente ---
//         printf("\nMotor Direito: Diminuindo velocidade para frente...\n");
//         for (speed = 255; speed > speed_step; speed -= speed_step) {
//             motor_set_speed(&motor_right, speed);
//             motor_set_speed(&motor_left, speed);
//             vTaskDelay(delay_ms_per_step);
//         }
//         motor_stop(&motor_right);
//         printf("Motor Direito: Parado (0) (Frente).\n");
//         vTaskDelay(pdMS_TO_TICKS(3000));

//         // --- Demonstração de aumento de velocidade do Motor Direito para trás ---
//         printf("\nMotor Direito: Aumentando velocidade para trás...\n");
//         motor_set_direction(&motor_right, false);
//         motor_set_direction(&motor_left, false);
//         for (speed = 0; speed <= 255; speed += speed_step) {
//             motor_set_speed(&motor_right, speed);
//             motor_set_speed(&motor_left, speed);
//             vTaskDelay(delay_ms_per_step);
//         }
//         printf("Motor Direito: Velocidade máxima (255) alcançada (Trás).\n");
//         vTaskDelay(pdMS_TO_TICKS(1500));

//         // --- Demonstração de diminuição de velocidade do Motor Direito para trás ---
//         printf("\nMotor Direito: Diminuindo velocidade para trás...\n");
//         for (speed = 255; speed >= speed_step; speed -= speed_step) {
//             motor_set_speed(&motor_right, speed);
//             motor_set_speed(&motor_left, speed);
//             vTaskDelay(delay_ms_per_step);
//         }
//         motor_stop(&motor_right);
//         printf("Motor Direito: Parado (0) (Trás).\n");
//         vTaskDelay(pdMS_TO_TICKS(3000));
//     }
// }