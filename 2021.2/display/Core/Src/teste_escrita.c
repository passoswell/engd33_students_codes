/*
 * @file  teste_escrita.c
 * @date  29-November-2021
 * @brief Código teste a aplicação de comunicação entre tarefas.
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"
#include "queue.h"
#include "filas.h"
#include "teste_escrita.h"

vEixo v_eixo;
GPS gps;
vBase v_base;

TaskHandle_t hwTask;

void teste_escrita(void) {

	xTaskCreate(wTask, "wTask", 128,   NULL,  3,  &hwTask);

	vTaskStartScheduler();

	while(1); /* Execution will never reach this line */

}
void wTask(void *arg){

	while(1){

		v_eixo.w1 = rand();
		v_eixo.w2 = rand();
		v_eixo.w3 = rand();
		gps.x = rand();
		gps.y = rand();
		gps.angulo_teta = rand();
		v_base.vx = rand();
		v_base.vy = rand();
		v_base.w = rand();

		writeDadosGps (gps, (TickType_t)10);
		writeDadosVEixo (v_eixo, (TickType_t)10);
		writeDadosVBase (v_base, (TickType_t)10);
	}

}

