//Alunos:
//Claudelino Oliveira
//Daniel Carneiro
//Matheus Lessa

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include "main.h"
#include "cmsis_os.h"


UART_HandleTypeDef huart2;

osThreadId taskUARTListenHandle;

/* uint8_t int8_t uint16_t int16_t uint32_t int32_t*/

//Dados de entrada para o sistema UART - Obtidos dos outros grupos:
//TODO: Os dados a seguir devem ser substituidos pela forma com os outros subsistemas fornecem os dados.
int16_t correnteMotor[3];
int16_t rotMotor[3];
int16_t accLinear[3];
int16_t vAngular[3];
int16_t campoMag[3];
int16_t aControleTracao[3];
int16_t gControleTracao[9];
int16_t aControleVelocidade[3];
int16_t SetpointVelocidade[3];
int16_t gControleVelocidade[9];
int16_t AngRotBase[3];
int16_t GPS[3];
int16_t gControlePos[3];
int16_t aControlePos[3];


//Estruturas de dados que integram o buffer UART:
typedef struct
{
	int16_t correnteM1[100];
	int16_t correnteM2[100];
	int16_t correnteM3[100];

}correnteMotorStruct;

typedef struct
{
	int16_t rotM1[10];
	int16_t rotM2[10];
	int16_t rotM3[10];

}rotMotorStruct;

typedef struct
{
	int16_t accLinearM1[10];
	int16_t accLinearM2[10];
	int16_t accLinearM3[10];

}accLinearStruct;

typedef struct
{
	int16_t vAngularM1[10];
	int16_t vAngularM2[10];
	int16_t vAngularM3[10];

}vAngularStruct;

typedef struct
{
	int16_t campoMagM1[10];
	int16_t campoMagM2[10];
	int16_t campoMagM3[10];

}campoMagStruct;

typedef struct
{
	int16_t aControleTracaoM1[100];
	int16_t aControleTracaoM2[100];
	int16_t aControleTracaoM3[100];

}aControleTracaoStruct;

typedef struct
{
	int16_t gControleTracaoM1Kp[100];
	int16_t gControleTracaoM1Ki[100];
	int16_t gControleTracaoM1Kd[100];
	int16_t gControleTracaoM2Kp[100];
	int16_t gControleTracaoM2Ki[100];
	int16_t gControleTracaoM2Kd[100];
	int16_t gControleTracaoM3Kp[100];
	int16_t gControleTracaoM3Ki[100];
	int16_t gControleTracaoM3Kd[100];

}gControleTracaoStruct;

typedef struct
{
	int16_t aControleVelocidadeM1[10];
	int16_t aControleVelocidadeM2[10];
	int16_t aControleVelocidadeM3[10];

}aControleVelocidadeStruct;

typedef struct
{
	int16_t SetpointVelocidadeM1[10];
	int16_t SetpointVelocidadeM2[10];
	int16_t SetpointVelocidadeM3[10];

}SetpointVelocidadeStruct;

typedef struct
{
	int16_t gControleVelocidadeM1Kp[10];
	int16_t gControleVelocidadeM1Ki[10];
	int16_t gControleVelocidadeM1Kd[10];
	int16_t gControleVelocidadeM2Kp[10];
	int16_t gControleVelocidadeM2Ki[10];
	int16_t gControleVelocidadeM2Kd[10];
	int16_t gControleVelocidadeM3Kp[10];
	int16_t gControleVelocidadeM3Ki[10];
	int16_t gControleVelocidadeM3Kd[10];

}gControleVelocidadeStruct;

typedef struct
{
	int16_t Roll[10];
	int16_t Pitch[10];
	int16_t Yaw[10];

}AngRotBaseStruct;

typedef struct
{
	int16_t GPSX;
	int16_t GPSY;
	int16_t GPSZ;

}GPSStruct;

typedef struct
{
	int16_t gControlePosKp;
	int16_t gControlePosKi;
	int16_t gControlePosKd;

}gControlePosStruct;

typedef struct
{
	int16_t aControlePos1;
	int16_t aControlePos2;
	int16_t aControlePos3;

}aControlePosStruct;

//Buffer UART juntando todas as estruturas

typedef struct
{
	correnteMotorStruct correnteMotorBuffer;
	rotMotorStruct rotMotorBuffer;
	accLinearStruct accLinearBuffer;
	vAngularStruct vAngularBuffer;
	campoMagStruct campoMagBuffer;
	aControleTracaoStruct aControleTracaoBuffer;
	gControleTracaoStruct gControleTracaoBuffer;
	aControleVelocidadeStruct aControleVelocidadeBuffer;
	SetpointVelocidadeStruct SetpointVelocidadeBuffer;
	gControleVelocidadeStruct gControleVelocidadeBuffer;
	AngRotBaseStruct AngRotBaseBuffer;
	GPSStruct GPSBuffer;
	gControlePosStruct gControlePosBuffer;
	aControlePosStruct aControlePosBuffer;

}BufferUARTStruct;

BufferUARTStruct BufferUart; //Inicialização da Struct do Buffer

//Index dos buffers de transmissão:
uint8_t indiceBuffer = 0;
uint8_t indiceBuffer2 = 0;


//Structs para recepção de dados:
typedef struct
{
	int16_t gControleTracaoM1Kp;
	int16_t gControleTracaoM1Ki;
	int16_t gControleTracaoM1Kd;
	int16_t gControleTracaoM2Kp;
	int16_t gControleTracaoM2Ki;
	int16_t gControleTracaoM2Kd;
	int16_t gControleTracaoM3Kp;
	int16_t gControleTracaoM3Ki;
	int16_t gControleTracaoM3Kd;

}gControleTracaoRXStruct;

typedef struct
{
	int16_t gControleVelocidadeM1Kp;
	int16_t gControleVelocidadeM1Ki;
	int16_t gControleVelocidadeM1Kd;
	int16_t gControleVelocidadeM2Kp;
	int16_t gControleVelocidadeM2Ki;
	int16_t gControleVelocidadeM2Kd;
	int16_t gControleVelocidadeM3Kp;
	int16_t gControleVelocidadeM3Ki;
	int16_t gControleVelocidadeM3Kd;

}gControleVelocidadeRXStruct;

typedef struct
{
	int16_t gControlePosKp;
	int16_t gControlePosKi;
	int16_t gControlePosKd;

}gControlePosRXStruct;

//Inicialização das structs de recepção
gControleTracaoRXStruct gControleTracaoRX;
gControleVelocidadeRXStruct gControleVelocidadeRX;
gControlePosRXStruct gControlePosRX;

uint8_t rxtype = 0; //Flag de identificação do tipo de operação de recepção

TaskHandle_t xTaskToNotifyatUARTRx;



void TX1Task(void); //Tasks de Transmissão UART
void TX2Task(void);
void TX3Task(void);
void TaskUARTListen(void *argument);

void start_rtos(void)
{
//Cria semaforo para evitar divisão de uso da porta TX da UART:

  SemaphoreHandle_t xSemaforo_UART;
  xSemaforo_UART = xSemaphoreCreateMutex();


  xTaskCreate(TaskUARTListen,
		  "TaskUARTListen", //nome
		  128, //stack size
		  NULL, //parametros passados
		  osPriorityLow, //prioridade
		  NULL);

//Criação das TASKS de Transmissão:
  xTaskCreate(TX1Task,
		  "TX1", //nome
		  128, //stack size
		  NULL, //parametros passados
		  tskIDLE_PRIORITY+1, //prioridade
		  NULL);

  xTaskCreate(TX2Task,
  		  "TX2",
  		  128,
  		  NULL,
		  tskIDLE_PRIORITY+2,
  		  NULL);

  xTaskCreate(TX3Task,
  		  "TX3",
  		  128,
  		  NULL,
		  tskIDLE_PRIORITY+3,
  		  NULL);

	vTaskStartScheduler();

	while(1); /* Execution will never reach this line */
}

void TX1Task(void){
	//Preenche o buffer com os dados com período igual a 1ms:
	BufferUART.correnteMotorBuffer.correnteM1[indiceBuffer] = correnteMotor[0];
	BufferUART.correnteMotorBuffer.correnteM2[indiceBuffer] = correnteMotor[1];
	BufferUART.correnteMotorBuffer.correnteM3[indiceBuffer] = correnteMotor[2];
	BufferUART.aControleTracaoBuffer.aControleTracaoM1[indiceBuffer] = aControleTracao[0];
	BufferUART.aControleTracaoBuffer.aControleTracaoM2[indiceBuffer] = aControleTracao[1];
	BufferUART.aControleTracaoBuffer.aControleTracaoM3[indiceBuffer] = aControleTracao[2];
	BufferUART.gControleTracaoBuffer.gControleTracaoM1Kp[indiceBuffer] = gControleTracao[0];
	BufferUART.gControleTracaoBuffer.gControleTracaoM1Ki[indiceBuffer] = gControleTracao[1];
	BufferUART.gControleTracaoBuffer.gControleTracaoM1Kd[indiceBuffer] = gControleTracao[2];
	BufferUART.gControleTracaoBuffer.gControleTracaoM2Kp[indiceBuffer] = gControleTracao[3];
	BufferUART.gControleTracaoBuffer.gControleTracaoM2Ki[indiceBuffer] = gControleTracao[4];
	BufferUART.gControleTracaoBuffer.gControleTracaoM2Kd[indiceBuffer] = gControleTracao[5];
	BufferUART.gControleTracaoBuffer.gControleTracaoM3Kp[indiceBuffer] = gControleTracao[6];
	BufferUART.gControleTracaoBuffer.gControleTracaoM3Ki[indiceBuffer] = gControleTracao[7];
	BufferUART.gControleTracaoBuffer.gControleTracaoM3Kd[indiceBuffer] = gControleTracao[8];

	//Incrementa o indice do buffer e espera para a próxima janela:
	indiceBuffer = indiceBuffer+1;

	if (indiceBuffer > 99){ //Segurança para evitar overflow do buffer caso a TX3 não seja executada
		indiceBuffer = 0;
	}
	vTaskDelay (pdMS_TO_TICKS(1)); //Delay until
}


void TX2Task(void){
	//Preenche o buffer com os dados com período igual a 10ms:
	BufferUART.rotMotorBuffer.rotM1[indiceBuffer2] = rotMotor[0];
	BufferUART.rotMotorBuffer.rotM2[indiceBuffer2] = rotMotor[1];
	BufferUART.rotMotorBuffer.rotM3[indiceBuffer2] = rotMotor[2];
	BufferUART.accLinearBuffer.accLinearM1[indiceBuffer2] = accLinear[0];
	BufferUART.accLinearBuffer.accLinearM2[indiceBuffer2] = accLinear[1];
	BufferUART.accLinearBuffer.accLinearM3[indiceBuffer2] = accLinear[2];
	BufferUART.vAngularBuffer.vAngularM1[indiceBuffer2] = vAngular[0];
	BufferUART.vAngularBuffer.vAngularM2[indiceBuffer2] = vAngular[1];
	BufferUART.vAngularBuffer.vAngularM3[indiceBuffer2] = vAngular[2];
	BufferUART.campoMagBuffer.campoMagM1[indiceBuffer2] = campoMag[0];
	BufferUART.campoMagBuffer.campoMagM2[indiceBuffer2] = campoMag[1];
	BufferUART.campoMagBuffer.campoMagM3[indiceBuffer2] = campoMag[2];
	BufferUART.aControleVelocidadeBuffer.aControleVelocidadeM1[indiceBuffer2] = aControleVelocidade[0];
	BufferUART.aControleVelocidadeBuffer.aControleVelocidadeM2[indiceBuffer2] = aControleVelocidade[1];
	BufferUART.aControleVelocidadeBuffer.aControleVelocidadeM3[indiceBuffer2] = aControleVelocidade[2];
	BufferUART.SetpointVelocidadeBuffer.SetpointVelocidadeM1[indiceBuffer2] = SetpointVelocidade[0];
	BufferUART.SetpointVelocidadeBuffer.SetpointVelocidadeM2[indiceBuffer2] = SetpointVelocidade[1];
	BufferUART.SetpointVelocidadeBuffer.SetpointVelocidadeM3[indiceBuffer2] = SetpointVelocidade[2];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM1Kp[indiceBuffer2] = gControleVelocidade[0];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM1Ki[indiceBuffer2] = gControleVelocidade[1];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM1Kd[indiceBuffer2] = gControleVelocidade[2];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM2Kp[indiceBuffer2] = gControleVelocidade[3];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM2Ki[indiceBuffer2] = gControleVelocidade[4];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM2Kd[indiceBuffer2] = gControleVelocidade[5];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM3Kp[indiceBuffer2] = gControleVelocidade[6];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM3Ki[indiceBuffer2] = gControleVelocidade[7];
	BufferUART.gControleVelocidadeBuffer.gControleVelocidadeM3Kd[indiceBuffer2] = gControleVelocidade[8];
	BufferUART.AngRotBaseBuffer.Roll[indiceBuffer2] = AngRotBase[0];
	BufferUART.AngRotBaseBuffer.Pitch[indiceBuffer2] = AngRotBase[1];
	BufferUART.AngRotBaseBuffer.Yaw[indiceBuffer2] = AngRotBase[2];

	//Incrementa o indice do buffer e espera para a próxima janela:
	indiceBuffer2 = indiceBuffer2+1;

	if (indiceBuffer > 9){ //Segurança para evitar overflow do buffer caso a TX3 não seja executada
		indiceBuffer = 0;
	}

	vTaskDelay (pdMS_TO_TICKS(10));
}

void TX3Task(void){
	//Preenche o buffer com os dados com período igual a 100ms:
	BufferUART.GPSBuffer.GPSX = GPS[0];
	BufferUART.GPSBuffer.GPSY = GPS[1];
	BufferUART.GPSBuffer.GPSZ = GPS[2];
	BufferUART.gControlePosBuffer.gControlePosKp = gControlePos[0];
	BufferUART.gControlePosBuffer.gControlePosKi = gControlePos[1];
	BufferUART.gControlePosBuffer.gControlePosKd = gControlePos[2];
	BufferUART.aControlePosBuffer.aControlePos1 = aControlePos[0];
	BufferUART.aControlePosBuffer.aControlePos2 = aControlePos[1];
	BufferUART.aControlePosBuffer.aControlePos3 = aControlePos[2];

	//Zera os buffers anteriores:
	indiceBuffer = 0;
	indiceBuffer2 = 0;

	//Assume o semáforo, envia a dataStruct e então libera o semáforo:
	if( xSemaphoreTake( xSemaforo_UART, ( TickType_t ) pdMS_TO_TICKS(100) ) == pdTRUE )
		{

		HAL_UART_Transmit_IT(&huart2, BufferUART, sizeof(BufferUART));
		xSemaphoreGive( xSemaforo_UART );
	}
	//Espera a próxima janela de tempo:
	vTaskDelay (pdMS_TO_TICKS(100));
}

void TaskUARTListen(void *argument)
{
  configASSERT( xTaskToNotifyatUARTRx == NULL );
  xTaskToNotifyatUARTRx = xTaskGetCurrentTaskHandle();

  uint8_t idtRX;
  for(;;)
  {
	if(rxtype ==0){
		HAL_UART_Receive_IT(&huart2, (int16_t *)idtRX, sizeof(int16_t));
		uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
		if( ulNotificationValue == 1 ) {
			if( idtRX ==1){
				rxtype=1;
			}
			if( idtRX ==2){
				rxtype=2;
			}
			if( idtRX ==3){
				rxtype=3;
			}
			else{
				rxtype=0;
			}
		}
	    else
	    {

	    }
	}
	if(rxtype ==1){
		HAL_UART_Receive_IT(&huart2, gControleTracaoRX, sizeof(gControleTracaoRX));
		uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		if( ulNotificationValue == 1 )
		{
			gControleTracaoRX.gControleTracaoM1Kp = gControleTracao[0];
			gControleTracaoRX.gControleTracaoM1Ki = gControleTracao[1];
			gControleTracaoRX.gControleTracaoM1Kd = gControleTracao[2];
			gControleTracaoRX.gControleTracaoM2Kp = gControleTracao[3];
			gControleTracaoRX.gControleTracaoM2Ki = gControleTracao[4];
			gControleTracaoRX.gControleTracaoM2Kd = gControleTracao[5];
			gControleTracaoRX.gControleTracaoM3Kp = gControleTracao[6];
			gControleTracaoRX.gControleTracaoM3Ki = gControleTracao[7];
			gControleTracaoRX.gControleTracaoM3Kd = gControleTracao[8];

			rxtype=0;

		}
	    else
	    {
	      //Notificação não enviada a tempo
	    }
	}
	if(rxtype ==2){
		HAL_UART_Receive_IT(&huart2, gControleVelocidadeRX, sizeof(gControleVelocidadeRX));
		uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		if( ulNotificationValue == 1 )
		{
			gControleVelocidadeRX.gControleVelocidadeM1Kp = gControleVelocidade[0];
			gControleVelocidadeRX.gControleVelocidadeM1Ki = gControleVelocidade[1];
			gControleVelocidadeRX.gControleVelocidadeM1Kd = gControleVelocidade[2];
			gControleVelocidadeRX.gControleVelocidadeM2Kp = gControleVelocidade[3];
			gControleVelocidadeRX.gControleVelocidadeM2Ki = gControleVelocidade[4];
			gControleVelocidadeRX.gControleVelocidadeM2Kd = gControleVelocidade[5];
			gControleVelocidadeRX.gControleVelocidadeM3Kp = gControleVelocidade[6];
			gControleVelocidadeRX.gControleVelocidadeM3Ki = gControleVelocidade[7];
			gControleVelocidadeRX.gControleVelocidadeM3Kd = gControleVelocidade[8];

			rxtype=0;
		}
	    else
	    {
	    	//Notificação não enviada a tempo
	    }
	}
	if(rxtype ==3){
		HAL_UART_Receive_IT(&huart2, gControleTracaoRX, sizeof(gControleTracaoRX));
		uint32_t ulNotificationValue = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );

		if( ulNotificationValue == 1 )
		{
			gControlePosRX.gControlePosKp = gControlePos[0];
			gControlePosRX.gControlePosKi = gControlePos[1];
			gControlePosRX.gControlePosKd = gControlePos[2];

			rxtype=0;
		}
	    else
	    {
	    	//Notificação não enviada a tempo
	    }
	}
  }
}
