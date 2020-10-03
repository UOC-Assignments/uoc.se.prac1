/******************************************************************************

                        Sistemes Encastats - UOC 2019/20-1

                               PR�CTICA - Projecte 1

                                 Jordi Bericat Ruz

*******************************************************************************/


/*------------------------------------------------------------------------------

                                      INCLUDES

------------------------------------------------------------------------------*/


/* Standard includes */
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

/* Free-RTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "portmacro.h"

/* MSP432 drivers includes */
#include "msp432_launchpad_board.h"

/* Boosterpack buttons drivers includes */
#include "edu_boosterpack_buttons.h"

/* Boosterpack Accelerometer drivers includes */
#include "edu_boosterpack_accelerometer.h"

/* Boosterpack RGB drivers includes */
#include "edu_boosterpack_rgb.h"

/* LCD related drivers includes */
#include "st7735.h"
#include "st7735_msp432.h"
#include "grlib.h"


/*------------------------------------------------------------------------------

                                      CONSTANTS

------------------------------------------------------------------------------*/


#define ADC_TASK_PRIORITY           ( tskIDLE_PRIORITY + 4 )
#define PROCESSING_TASK_PRIORITY    ( tskIDLE_PRIORITY + 3 )
#define LCD_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2 )
#define HEARTBEAT_TASK_PRIORITY     ( tskIDLE_PRIORITY + 1 )

#define ADC_TASK_STACK_SIZE         ( 1024 )
#define PROCESSING_TASK_STACK_SIZE  ( 1024 )
#define LCD_TASK_STACK_SIZE         ( 1024 )
#define HEARTBEAT_TASK_STACK_SIZE   ( 128 )

#define QUEUE_SIZE                  ( 15 )

#define HEART_BEAT_ON_MS            ( pdMS_TO_TICKS(10) )
#define HEART_BEAT_OFF_MS           ( pdMS_TO_TICKS(990) )
#define PROCESSING_TASK_DELAY_MS    ( pdMS_TO_TICKS(500) )
#define ADC_TASK_DELAY_MS           ( pdMS_TO_TICKS(10) )

/* Establim els valors promig de l'acceler�metre en l'estat de rep�s */
#define REPOSE_ACC_X               ( 8250.00 )
#define REPOSE_ACC_Y               ( 8250.00 )
#define REPOSE_ACC_Z               ( 11563.0 )

/* Valors m�nims i m�xims obtinguts de cada eix de l'accerel�metre amb la implementaci� de la PAC2 */
#define MIN_ACC_X                  ( 5000.00 )
#define MAX_ACC_X                  ( 12000.00 )
#define MIN_ACC_Y                  ( 5000.00 )
#define MAX_ACC_Y                  ( 12000.00 )

#define ERROR_THRESHOLD            ( 0.5 )
#define MEAN_VALUES                ( 10.0 )


/*------------------------------------------------------------------------------

                               PROTOTIPS DE FUNCIONS

------------------------------------------------------------------------------*/


/* Tasques */
static void ADCTask(void *pvParameters);
static void ProcessingTask(void *pvParameters);
static void HeartBeatTask(void *pvParameters);
static void LCDTask(void *pvParameters);

/* ISR Callbacks */
void buttons_callback(void);
void get_adc_results(adc_result);

/* Funcions auxiliars */
uint8_t map_i(uint16_t, uint16_t, uint16_t, uint16_t);


/*------------------------------------------------------------------------------

                                 TIPUS PREDEFINITS

------------------------------------------------------------------------------*/


/* Estructura per a desar els valors de lectura de l'acceler�metre */
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} acc_t;


/*------------------------------------------------------------------------------

                                 OBJECTES GLOBALS

------------------------------------------------------------------------------*/


/* Mecanismes de sincronitzaci� de tasques */
SemaphoreHandle_t xRunADCTaskSemaphore;
SemaphoreHandle_t xADCReadingAvailableSemaphore;
QueueHandle_t xQueueADCToProcess;
QueueHandle_t xQueueProcessToLCD;

/* Variable Globals: LCD */
static Graphics_Context g_sContext;

/* Variable Global: Timestamp d'inici de lectures de l'acceler�metre */
TickType_t start_timestamp;

/* Variable Globals: Valors de l'acceler�metre */
acc_t accel_values;

/* Variable Global: "Token" que utilitzem per a poder alliberar el sem�for "xRunADCTaskSemaphore",
 * des de la tasca "ADCTask" i aix� poder prendre lectures del ADC cada 10 ms*/
bool TakeADCReadings = false;


/*------------------------------------------------------------------------------

                                       MAIN

------------------------------------------------------------------------------*/


int main(int argc, char** argv)
{
    int32_t retVal = -1;

    /* Inicialitzaci� de sem�fors, m�texs i cues */
    xRunADCTaskSemaphore = xSemaphoreCreateBinary ();
    xADCReadingAvailableSemaphore =  xSemaphoreCreateBinary ();
    xQueueADCToProcess = xQueueCreate( QUEUE_SIZE, sizeof( acc_t ) );
    xQueueProcessToLCD = xQueueCreate( QUEUE_SIZE, sizeof( TickType_t ) );

    /* Inicialitzaci� de la placa */
    board_init();

    /* Inicialitzaci� des botons del boosterpack */
    edu_boosterpack_buttons_init();

    /* Inicialitzaci� de l'acceler�metre */
    edu_boosterpack_accelerometer_init();

    /* Inicialitzaci� del led RGB */
    edu_boosterpack_rgb_init();

    /* Inicialitzaci� del display LCD */
    Crystalfontz128x128_Init();

    /* Set default screen orientation */
    Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

    /* Initializes graphics context */
    Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
    Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_BLACK);
    Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
    GrContextFontSet(&g_sContext, &g_sFontFixed6x8);

    /* Configuraci� dels callbacks d'interrupcions */
    edu_boosterpack_buttons_set_callback(MSP432_EDU_BOOSTERPACK_BUTTON_S1, buttons_callback);
    edu_boosterpack_buttons_set_callback(MSP432_EDU_BOOSTERPACK_BUTTON_S2, buttons_callback);
    edu_boosterpack_accelerometer_set_callback(get_adc_results);

    if (       (xRunADCTaskSemaphore != NULL)
            && (xADCReadingAvailableSemaphore != NULL)
            && (xQueueADCToProcess != NULL)
            && (xQueueProcessToLCD != NULL) ) {

        /* Creaci� de la tasca "ADCTask" */
        retVal = xTaskCreate(ADCTask,
                             "ADCTask",
                             ADC_TASK_STACK_SIZE,
                             NULL,
                             ADC_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "ProcessingTask" */
        retVal = xTaskCreate(ProcessingTask,
                             "ProcessingTask",
                             PROCESSING_TASK_STACK_SIZE,
                             NULL,
                             PROCESSING_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "LCDTask" */
        retVal = xTaskCreate(LCDTask,
                             "LCDTask",
                             LCD_TASK_STACK_SIZE,
                             NULL,
                             LCD_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Creaci� de la tasca "HeartBeatTask" */
        retVal = xTaskCreate(HeartBeatTask,
                             "HeartBeatTask",
                             HEARTBEAT_TASK_STACK_SIZE,
                             NULL,
                             HEARTBEAT_TASK_PRIORITY,
                             NULL );
        if(retVal < 0) {
            led_on(MSP432_LAUNCHPAD_LED_RED);
            while(1);
        }

        /* Inicialitzaci� de l'scheduler */
        vTaskStartScheduler();
    }
    return 0;
}


/*------------------------------------------------------------------------------

                                        TASQUES

------------------------------------------------------------------------------*/


static void ADCTask(void *pvParameters){

    for(;;){

        if( xSemaphoreTake( xRunADCTaskSemaphore, portMAX_DELAY ) == pdTRUE ){

            /* Obtenim la lectura actual de l'acceler�metre. Concretament, La seg�ent crida
             * disparar� la corresponent interrupci� de l'ADC que permetra desar als registres de
             * mem�ria de l'ADC les lectures de les entrades anal�giques configurades per llegir
             * els 3 canals de sortida (x, y i z) de l'acceler�metre. La ISR de l'ADC utilitzar�
             * la funci� de callback "get_adc_results" per a retornar les lectures, les quals ser�n desades
             * a la variable global "accel_values" per a ser finalment enviades per la cua a la tasca "LCDTask"  */
            edu_boosterpack_accelerometer_read();

            if( xSemaphoreTake( xADCReadingAvailableSemaphore, portMAX_DELAY ) == pdTRUE ){
                xQueueSend(xQueueADCToProcess, &accel_values, portMAX_DELAY);
            }
        }
        /* Si la placa encara no est� anivellada en aquest punt hem de seguir realitzant
         * lectures de l'ADC cada 10 ms. Per a assolir aquest objectiu establirem un delay
         * i alliberarem el sem�for des de dins de la mateixa tasca. Un cop anivellada
         * la placa utilitzarem una variabel global per a indicar-li a la tasca que no ha de tornar
         * a alliberar el sem�for*/
        vTaskDelay(ADC_TASK_DELAY_MS);
        if ( TakeADCReadings == true ){
            xSemaphoreGive( xRunADCTaskSemaphore );
        }
    }
}

static void ProcessingTask(void *pvParameters) {

    acc_t new_accel_value;
    float mean_acc_x = 0;
    float mean_acc_y = 0;
    float mean_acc_z = 0;
    float z_axis_error = 0;
    TickType_t end_timestamp;
    TickType_t total_ticks;
    uint8_t red_rgb_led;
    uint8_t blue_rgb_led;

    for(;;){

        if( xQueueReceive( xQueueADCToProcess, &new_accel_value, portMAX_DELAY ) == pdPASS ){

            mean_acc_x = (float)new_accel_value.x * (1/MEAN_VALUES) + mean_acc_x * ((MEAN_VALUES-1)/MEAN_VALUES);
            mean_acc_y = (float)new_accel_value.y * (1/MEAN_VALUES) + mean_acc_y * ((MEAN_VALUES-1)/MEAN_VALUES);
            mean_acc_z = (float)new_accel_value.z * (1/MEAN_VALUES) + mean_acc_z * ((MEAN_VALUES-1)/MEAN_VALUES);

            /* Calculem el percentatge (Valor absolut) d'inclinaci� de la placa respecte del valor de rep�s
             * de l'eix z en rep�s de l'acceler�metre */
            z_axis_error = fabs( ( ( (float)mean_acc_z - REPOSE_ACC_Z ) / REPOSE_ACC_Z ) * 100 );

            /* Si z_axis_error �s m�s petit o igual que 1, aleshores tenim la placa anivellada */
            if ( z_axis_error <= ERROR_THRESHOLD ){

                /* Obtenim el timestamp (en ticks) del moment en que s'ha assolit l'anivellat de la placa */
                end_timestamp = xTaskGetTickCount();

                /* "Desactivem" l'alliberament del sem�for "xRunADCTaskSemaphore" cada 10ms per  tal de
                 * deixar de prendre mostres de l'acceler�metre */
                TakeADCReadings = false;

                /* Apaguem leds RGB i vermell i encenem el verd */
                edu_boosterpack_rgb_pwm_all(0, 0, 0);
                led_off(MSP432_LAUNCHPAD_LED_RED1);
                led_on(MSP432_LAUNCHPAD_LED_GREEN);

                /* Imposem un retard de 500ms per tal de poder observar que el led verd s'enc�n */
                vTaskDelay(PROCESSING_TASK_DELAY_MS);

                /* calculem la quantitat de ticks totals que han passat des de que s'ha comen�at
                 * a prendre mostres del ADC fins que s'ha anivellat la placa */
                total_ticks = end_timestamp - start_timestamp;

                /* Enviem el diferencial de timestamps a la tasca LCDTask amb la segona cua */
                xQueueSend(xQueueProcessToLCD, &total_ticks, portMAX_DELAY);
            }
            else {
                /* Obtenim els valors d'intensitat de cada led en funci� de la lectura de l'acceler�metre
                 * (veure definici� de la funci� "map_i" per una descripci� m�s detallada). */
                red_rgb_led = map_i(mean_acc_x, REPOSE_ACC_X, MIN_ACC_X, MAX_ACC_X);
                blue_rgb_led = map_i(mean_acc_y, REPOSE_ACC_Y, MIN_ACC_Y, MAX_ACC_Y);

                /* Activem el led RGB amb els valors d'intensitat obtinguts a les crides a la funci� "map_i" */
                edu_boosterpack_rgb_pwm_all(red_rgb_led, 0, blue_rgb_led);
            }
        }
    }
}

static void LCDTask(void *pvParameters){

    TickType_t total_ticks;
    unsigned long total_ms;
    char total_ms_str[10];

    for(;;){

        /* Rebem diferencial de timestamps de la cua */
        if( xQueueReceive( xQueueProcessToLCD, &total_ticks, portMAX_DELAY ) == pdPASS )
        {
            /* Apaguem el led verd */
            led_off(MSP432_LAUNCHPAD_LED_GREEN);

            /* Convertim els ticks a ms (multipliquem * 10 ja que la resoluci� tick est�
             * configurada de manera que "1 tick > 1 ms")*/
            total_ms = ( total_ticks / pdMS_TO_TICKS(10) )*10;
            sprintf(total_ms_str,"%lu ms",total_ms);

            Graphics_drawString(&g_sContext,
                                "Nivell aconseguit!",
                                AUTO_STRING_LENGTH,
                                10,
                                30,
                                OPAQUE_TEXT);

            Graphics_drawString(&g_sContext,
                                "Temps transcorregut:",
                                AUTO_STRING_LENGTH,
                                5,
                                60,
                                OPAQUE_TEXT);

            Graphics_drawString(&g_sContext,
                                (int8_t*)total_ms_str,
                                AUTO_STRING_LENGTH,
                                50,
                                90,
                                OPAQUE_TEXT);
        }
    }
}

static void HeartBeatTask(void *pvParameters){

    for(;;){

        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( HEART_BEAT_ON_MS );
        led_toggle(MSP432_LAUNCHPAD_LED_RED);
        vTaskDelay( HEART_BEAT_OFF_MS );
    }
}


/*------------------------------------------------------------------------------

                                   ISR CALLBACKS

------------------------------------------------------------------------------*/


void buttons_callback(void) {
    /* Establim el "token" que permet a la tasca "ADCReadingsTask" alliberar el sem�for
     * cada 10ms */
    TakeADCReadings = true;

    /* Esborrem el text del display LCD abans de comen�ar el proc�s d'anivellat */
    Graphics_clearDisplay(&g_sContext);

    /* Encenem el led vermell per indicar que la placa encara no es troba anivellada */
    led_on(MSP432_LAUNCHPAD_LED_RED1);

    /* Obtenim el comptador de ticks actual per tal de poder calcular el temps total
     * D'execuci� del programa des de que es prem un dels botons del boosterpack fins
     * que s'aconsegueix nivellar la placa  */
    start_timestamp = xTaskGetTickCount();

    /* Alliberem el sem�for desde la ISR */
    xSemaphoreGiveFromISR( xRunADCTaskSemaphore, NULL );
}

void get_adc_results (adc_result results_buffer ){
    accel_values.x = results_buffer[0];
    accel_values.y = results_buffer[1];
    accel_values.z = results_buffer[2];
    xSemaphoreGiveFromISR( xADCReadingAvailableSemaphore, NULL );
}


/*------------------------------------------------------------------------------

                                FUNCIONS AUXILIARS

------------------------------------------------------------------------------*/


// FUNCI� DE MAPEIG ADAPTADA DE --> https://energia.nu/reference/en/language/functions/math/map/

// La funci� "map_i()" re-mapeja una lectura de l'acceler�metre (compresa entre un rang de lectures m�xima
// "max_eix_i" i m�nima "min_eix_i" que s'actualitzen en temps d'execuci�) a la seva correspond�cia compresa
// entre 0 i 255 (valor d'intensitat del led "r", "g" o "b"), tot tenint en compte la inclinaci� de l'eix "i"
// (inclinaci� positiva o negativa). Les correspond�ncies s�n les seg�ents, on "m�xima intensitat led" = 255
// i "off" = 0:
//
// ----------------------------------------------------
//
//                  Eix X Acceler�metre
//
//        min_x_axis   repose_x_axis    max_x_axis
//            ^             ^               ^
//            |             |               |
// m�xima intensitat led / off / m�xima intensitat led
//
//
// ----------------------------------------------------
//
//                  Eix Y Acceler�metre
//
//        min_y_axis   repose_y_axis    max_y_axis
//            ^             ^               ^
//            |             |               |
// m�xima intensitat led / off / m�xima intensitat led
//
// ----------------------------------------------------
//
//                  Eix Z Acceler�metre
//
//        min_z_axis   repose_z_axis    max_z_axis
//            ^             ^               ^
//            |             |               |
//           off / m�xima intensitat led / off
//
// ----------------------------------------------------


// Els valors en l'estat de repos (repose_i_axis) de l'acceler�metre s'han especificat de manera absoluta
// en funci� de tot un seguit de lectures de l'acceler�mentre realitzes amb la implementaci� de la PAC2
//

uint8_t map_i(uint16_t i, uint16_t repose_i_axis, uint16_t min_i_axis, uint16_t max_i_axis)
{
    uint16_t in_min;
    uint16_t in_max;
    uint8_t out_min;
    uint8_t out_max;

    //Si l'eix "I" s'inclina cap amunt (inclinaci� positiva)
    if (i < repose_i_axis & i > min_i_axis)
    {
        in_min = min_i_axis;
        in_max = repose_i_axis;
        out_min = 255;
        out_max = 0;
    }

    //Si l'eix "I" s'inclina cap avall (inclinaci� negativa)
    else if (i < max_i_axis & i > repose_i_axis)
    {
        in_min = repose_i_axis;
        in_max = max_i_axis;
        out_min = 0;
        out_max = 255;
    }

    // retornem la correspond�ncia del valor d'intensitat del led (entre 0 i 255) en funci� del rang
    // del valor d'entrada (compr�s entre el valor m�nim i m�xim de lectura de l'acceler�metre per a l'eix "I")
    return (i - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
