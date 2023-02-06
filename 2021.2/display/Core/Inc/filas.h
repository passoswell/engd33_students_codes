/*
 * @file  filas_rtos.h
 * @date  29-November-2021
 * @brief Cabeçalho para Aplicação para comunicação em tarefas.
 */

#ifndef INC_FILAS_H_
#define INC_FILAS_H_

/* Declaração das estrutura para coleta de dados */
typedef  struct
{
	uint32_t w1;
	uint32_t w2;
	uint32_t w3;
} vEixo;

typedef  struct
{
	int32_t x;
	int32_t y;
	int32_t angulo_teta;
} GPS;

typedef  struct
{
	uint32_t vx;
	uint32_t vy;
	uint32_t w;
} vBase;

void criar_filas(void);
int8_t readDadosVEixo (vEixo dados, TickType_t tempo, UBaseType_t uxPriority);
int8_t readDadosVBase (vBase dados, TickType_t tempo, UBaseType_t uxPriority);
int8_t readDadosGps (GPS dados, TickType_t tempo, UBaseType_t uxPriority);
int8_t writeDadosVEixo (vEixo dados, TickType_t tempo);
int8_t writeDadosVBase (vBase dados, TickType_t tempo);
int8_t writeDadosGps (GPS dados, TickType_t tempo);

#endif /* INC_FILAS_H_ */
