#ifndef HARDWARE_INIT_H
#define HARDWARE_INIT_H

#include "hardware/i2c.h"

// Protótipos de funções de inicialização de hardware
/**
 * @brief Inicializa a instância I2C especificada com os pinos e velocidade definidos.
 * @param i2c_instance A instância I2C (ex: i2c0, i2c1).
 * @param sda_gpio O pino GPIO para SDA.
 * @param scl_gpio O pino GPIO para SCL.
 * @param baud_rate A taxa de baud desejada para o barramento I2C (em Hz).
 */
void i2c_bus_init(i2c_inst_t *i2c_instance, uint sda_gpio, uint scl_gpio, uint32_t baud_rate);

#endif // HARDWARE_INIT_H