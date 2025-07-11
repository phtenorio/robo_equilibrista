#include "hardware_init.h"
#include "pico/stdlib.h"
#include <stdio.h> // Para printf

void i2c_bus_init(i2c_inst_t *i2c_instance, uint sda_gpio, uint scl_gpio, uint32_t baud_rate) {
    i2c_init(i2c_instance, baud_rate);
    gpio_set_function(sda_gpio, GPIO_FUNC_I2C);
    gpio_set_function(scl_gpio, GPIO_FUNC_I2C);
    gpio_pull_up(sda_gpio);
    gpio_pull_up(scl_gpio);
    printf("Hardware I2C %s inicializado nos pinos SDA=%d, SCL=%d a %d Hz\n",
           i2c_instance == i2c0 ? "i2c0" : "i2c1", sda_gpio, scl_gpio, baud_rate);
}