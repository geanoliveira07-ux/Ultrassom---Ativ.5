#ifndef ULTRASSOM_H

#define ULTRASSOM_H
#include <zephyr/kernel.h>

void ultrassom_init(void);                  // Inicializa os pinos e interrupções do HC-SR04
void ultrassom_trigger(void);               // Envia o pulso de 10us para iniciar a leitura (assíncrono)
uint32_t ultrassom_get_distancia(void);     // Retorna a última distância calculada em centímetros

#endif