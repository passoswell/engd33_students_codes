/*
 * @file  semaforos.h
 * @date  31-November-2021
 * @brief Aplicação par aimplementação de semáforos.
 */

#ifndef INC_SEMAFOROS_H_
#define INC_SEMAFOROS_H_

#include "semphr.h"

void criar_semaforos(void);
extern SemaphoreHandle_t i2c_Semphr;

#endif /* INC_SEMAFOROS_H_ */
