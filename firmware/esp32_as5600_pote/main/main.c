#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "pote.h"
#include "as5600.h"
#include "consola.h"

static const char *TAG = "app";

static bool s_streaming = false;

static void mostrar_menu(void)
{
    printf("\n=== AS5600 - Practica 3 ===\n");
    printf("  s : escanear bus I2C\n");
    printf("  d : diagnostico del iman (STATUS, AGC, MAGNITUDE)\n");
    printf("  a : leer angulo una vez\n");
    printf("  t : tabla - marcar punto de medicion\n");
    printf("  c : streaming continuo ON/OFF\n");
    printf("  p : leer potenciometro\n");
    printf("  h : mostrar este menu\n");
    printf("===========================\n");
}

static void cmd_diagnostico(void)
{
    as5600_muestra_t m;
    if (as5600_leer(&m) != ESP_OK) {
        printf("ERROR: no se pudo leer el sensor\n");
        return;
    }
    printf("\n--- Diagnostico del iman ---\n");
    printf("STATUS    : 0x%02X\n", m.status);
    printf("  MD (detectado)   : %d  %s\n", m.md, m.md ? "OK" : "<-- NO HAY IMAN");
    printf("  ML (muy debil)   : %d  %s\n", m.ml, m.ml ? "<-- ALEJAR MENOS" : "OK");
    printf("  MH (muy fuerte)  : %d  %s\n", m.mh, m.mh ? "<-- ALEJAR MAS" : "OK");
    printf("AGC       : %3u/255  %s\n", m.agc,
           (m.agc > 40 && m.agc < 215) ? "OK" : "<-- AJUSTAR DISTANCIA");
    printf("MAGNITUDE : %4u\n", m.magnitud);
    printf("----------------------------\n");
}

static void cmd_angulo(void)
{
    as5600_muestra_t m;
    if (as5600_leer(&m) != ESP_OK) {
        printf("ERROR: no se pudo leer el sensor\n");
        return;
    }
    printf("RAW_ANGLE=%4u (%7.3f deg)   ANGLE=%4u (%7.3f deg)\n",
           m.raw_angle, m.grados_raw, m.angle, m.grados);
}

static void cmd_tabla(void)
{
    static int punto = 0;
    as5600_muestra_t m;
    if (as5600_leer(&m) != ESP_OK) {
        printf("ERROR: no se pudo leer el sensor\n");
        return;
    }
    printf("TABLA,%d,%.1f,%u,%u,%.3f,%u,%u\n",
           punto, punto * 22.5f,
           m.angle, m.raw_angle, m.grados_raw, m.agc, m.magnitud);
    punto++;
}

void app_main(void)
{
    consola_init();

    ESP_ERROR_CHECK(pote_init());
    ESP_LOGI(TAG, "Pote inicializado");

    if (as5600_init() != ESP_OK) {
        ESP_LOGE(TAG, "Fallo la inicializacion del AS5600");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));
    mostrar_menu();
    printf("TABLA,punto,esperado_deg,ANGLE,RAW_ANGLE,medido_deg,AGC,MAG\n");

    while (1) {
        char c;
        if (consola_hay_tecla(&c)) {
            switch (c) {
                case 's': as5600_escanear_bus(); break;
                case 'd': cmd_diagnostico();     break;
                case 'a': cmd_angulo();          break;
                case 't': cmd_tabla();           break;
                case 'h': mostrar_menu();        break;
                case 'c':
                    s_streaming = !s_streaming;
                    printf("Streaming %s\n", s_streaming ? "ON" : "OFF");
                    break;
                case 'p': {
                    pote_muestra_t p;
                    pote_leer(&p);
                    printf("Pote raw=%4d mV=%4d ref=%4d ang=%6.1f deg\n",
                           p.raw_senal, p.mv_senal, p.mv_ref, p.grados);
                    break;
                }
                default: break;
            }
        }

        if (s_streaming) {
            as5600_muestra_t m;
            if (as5600_leer(&m) == ESP_OK) {
                printf("STREAM,%u,%.3f,%u,%u\n",
                       m.raw_angle, m.grados_raw, m.agc, m.magnitud);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
