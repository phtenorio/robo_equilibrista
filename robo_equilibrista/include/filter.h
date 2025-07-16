#ifndef FILTER_H
#define FILTER_H

#include <stdint.h>
#include <stdbool.h>

// Estrutura para os dados do filtro
typedef struct {
    float angle_pitch; // Ângulo de inclinação (frente/trás) em graus
    float angle_roll;  // Ângulo de rolagem (lateral) em graus
    float dt;          // Intervalo de tempo entre as amostras em segundos
    float alpha;       // Fator alfa do filtro complementar
} balancer_filter_t;

/**
 * @brief Inicializa o filtro complementar.
 * @param filter Ponteiro para a estrutura do filtro.
 * @param dt O delta de tempo em segundos entre as leituras do sensor.
 * @param alpha O fator alfa para o filtro complementar (ex: 0.98).
 */
void filter_init(balancer_filter_t *filter, float dt, float alpha);

/**
 * @brief Atualiza a estimativa de ângulo usando o filtro complementar.
 * @param filter Ponteiro para a estrutura do filtro.
 * @param accel_g Array de acelerações em G (X, Y, Z).
 * @param gyro_dps Array de velocidades angulares em graus/segundo (X, Y, Z).
 */
void filter_update(balancer_filter_t *filter, float accel_g[3], float gyro_dps[3]);

#endif // FILTER_H