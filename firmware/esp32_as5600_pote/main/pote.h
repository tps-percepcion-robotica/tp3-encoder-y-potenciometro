#pragma once

#include "esp_err.h"

typedef struct {
    int   raw_senal;   /* cuenta ADC cruda del cursor (0-4095). Autoritativa:
                           la calibracion raw->grados vive en la PC. */
    int   raw_ref;     /* cuenta ADC cruda del Nodo A (referencia), 0-4095 */
    int   mv_senal;    /* mv calibrados de fabrica, solo diagnostico local */
    int   mv_ref;
    float grados;      /* estimacion LOCAL con constantes de circuito.
                           Solo para el comando de consola, no para RViz. */
} pote_muestra_t;

/* Cuantas muestras se toman para la mediana que se publica por micro-ROS */
#define POTE_MEDIANA_N 10

esp_err_t pote_init(void);

/* Una sola lectura, rapida. La usa el comando 'p' de consola. */
esp_err_t pote_leer(pote_muestra_t *m);

/* POTE_MEDIANA_N lecturas, ordenadas, se toma el valor central.
   Filtra los saltos aislados que vimos en las mediciones (~150 mV,
   no explicables por ruido termico). Esta es la que usa el publicador
   micro-ROS. */
esp_err_t pote_leer_mediana(pote_muestra_t *m);
