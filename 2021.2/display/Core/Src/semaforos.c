/*
 * @file  semaforos.c
 * @date  31-November-2021
 * @brief Aplicação par aimplementação de semáforos.
 */

#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "semphr.h"
#include "semaforos.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

SemaphoreHandle_t i2c_Semphr;

void criar_semaforos(void) {

	i2c_Semphr = xSemaphoreCreateBinary ();

	if (i2c_Semphr == NULL)
	{
		ssd1306_SetCursor(1, 2);
		ssd1306_WriteString("Erro: SemaphoreCreate", FONTE, White);
		ssd1306_UpdateScreen();
		while(1);
	}
}

