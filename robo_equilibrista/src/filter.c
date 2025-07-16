#include "filter.h"
#include <math.h>
#include <stdio.h>

// Definir M_PI se não estiver definido por <math.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

void filter_init(balancer_filter_t *filter, float dt, float alpha) {
    filter->angle_pitch = 0.0f;
    filter->angle_roll = 0.0f;
    filter->dt = dt;
    filter->alpha = alpha;
    printf("Filtro Complementar inicializado com dt=%.4f e alpha=%.2f\n", dt, alpha);
}

void filter_update(balancer_filter_t *filter, float accel_g[3], float gyro_dps[3]) {
    // 1. Calcular o ângulo a partir do acelerômetro
    float angle_accel_pitch = atan2f(accel_g[1], accel_g[2]) * 180.0f / M_PI; // Converte radianos para graus

    // 2. Calcular o ângulo a partir do giroscópio (integração)
    float gyro_pitch_rate = gyro_dps[0];

    // Integra o giroscópio para obter a mudança de ângulo
    filter->angle_pitch = filter->alpha * (filter->angle_pitch + gyro_pitch_rate * filter->dt) +
                          (1.0f - filter->alpha) * angle_accel_pitch;

}