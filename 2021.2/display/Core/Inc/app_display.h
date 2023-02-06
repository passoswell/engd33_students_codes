/**
 * @file  app_display.h
 * @date  25-November-2021
 * @brief Main application function.
 *
 * @author
 * @author
 */

#ifndef INC_APP_DISPLAY_H_
#define INC_APP_DISPLAY_H_

/**** Unidades de medida das variáveis ****/

#define V_EIXO "rpm"
#define GPS_X_Y "m"
#define GPS_ANGULO "graus"
#define V_BASE "m/s"
#define V_BASE_W "rpm"

/**** Estados para verificação de falha no display ****/

typedef enum {
    DISPLAY_OK = 0x00,
    DISPLAY_ERR = 0x01
} Display_Error;

void displayTask(void *arg);
void start_rtos(void);
void printVBase(void);
void printGPS(void);
void printVEixo(void);
void updateScreen(void);

#endif /* INC_APP_DISPLAY_H_ */
