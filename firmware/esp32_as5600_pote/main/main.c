#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "pote.h"
#include "as5600.h"
#include "consola.h"
#include "uros.h"

static const char *TAG = "app";

static bool s_streaming = false;
static int  s_punto     = 0;

static void mostrar_menu(void)
{
    printf("\n=== AS5600 + Potenciometro - TP3 ===\n");
    printf("  s : escanear bus I2C\n");
    printf("  d : diagnostico del iman (STATUS, AGC, MAGNITUDE)\n");
    printf("  a : leer angulo una vez\n");
    printf("  z : setear cero en la posicion actual (ZPOS)\n");
    printf("  Z : resetear cero a 0 (volver al crudo)\n");
    printf("  v : ver el ZPOS actual\n");
    printf("  t : tabla - marcar punto de medicion\n");
    printf("  r : reiniciar contador de puntos de la tabla\n");
    printf("  c : streaming continuo ON/OFF\n");
    printf("  p : leer potenciometro (una muestra)\n");
    printf("  P : leer potenciometro (mediana de %d, lo que se publica)\n", POTE_MEDIANA_N);
    printf("  h : mostrar este menu\n");
    printf("=====================================\n");
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
    printf("  ML (muy debil)   : %d  %s\n", m.ml, m.ml ? "<-- ACERCAR" : "OK");
    printf("  MH (muy fuerte)  : %d  %s\n", m.mh, m.mh ? "<-- ALEJAR" : "OK");
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

static void cmd_setear_cero(void)
{
    uint16_t zpos;
    if (as5600_setear_cero(&zpos) != ESP_OK) {
        printf("ERROR: no se pudo escribir ZPOS\n");
        return;
    }
    printf("ZPOS = %u cuentas (%.3f deg). Verificando...\n",
           zpos, zpos * 360.0f / 4096.0f);

    as5600_muestra_t m;
    if (as5600_leer(&m) == ESP_OK) {
        printf("  RAW_ANGLE=%4u (%7.3f deg)  ANGLE=%4u (%7.3f deg) <- deberia ser ~0\n",
               m.raw_angle, m.grados_raw, m.angle, m.grados);
    }
}

static void cmd_ver_cero(void)
{
    uint16_t zpos;
    if (as5600_leer_cero(&zpos) != ESP_OK) {
        printf("ERROR: no se pudo leer ZPOS\n");
        return;
    }
    printf("ZPOS actual = %u cuentas (%.3f deg)\n",
           zpos, zpos * 360.0f / 4096.0f);
}

static void cmd_reset_cero(void)
{
    if (as5600_reset_cero() != ESP_OK) {
        printf("ERROR: no se pudo resetear ZPOS\n");
        return;
    }
    printf("ZPOS = 0. ANGLE vuelve a coincidir con RAW_ANGLE.\n");
}

static void cmd_tabla(void)
{
    as5600_muestra_t m;
    if (as5600_leer(&m) != ESP_OK) {
        printf("ERROR: no se pudo leer el sensor\n");
        return;
    }
    printf("TABLA,%d,%.1f,%u,%.3f,%u,%.3f,%u,%u\n",
           s_punto, s_punto * 22.5f,
           m.raw_angle, m.grados_raw,
           m.angle,     m.grados,
           m.agc, m.magnitud);
    s_punto++;
}

static void cmd_reiniciar_tabla(void)
{
    s_punto = 0;
    printf("Contador de puntos reiniciado en 0.\n");
    printf("TABLA,punto,esperado_deg,RAW_ANGLE,raw_deg,ANGLE,ang_deg,AGC,MAG\n");
}

void app_main(void)
{
    consola_init();

    /* Silenciado: sin AS5600 conectado, cada intento de lectura genera
       varias lineas de NACK que tapan la consola. El error real sigue
       devolviendose por codigo, solo se apaga el log. Sacar este filtro
       cuando conectes el sensor para ver problemas de cableado. */
    esp_log_level_set("i2c.master", ESP_LOG_NONE);

    esp_err_t err_pote = pote_init();
    if (err_pote != ESP_OK) {
        ESP_LOGE(TAG, "Fallo la inicializacion del potenciometro: %s",
                 esp_err_to_name(err_pote));
        return;
    }
    ESP_LOGI(TAG, "Pote inicializado");

    if (as5600_init() != ESP_OK) {
        ESP_LOGE(TAG, "Fallo la inicializacion del AS5600");
        return;
    }

    /* Arranca en paralelo: conecta WiFi y publica potenciometro/AS5600
       por micro-ROS, sin bloquear el menu de consola que sigue abajo. */
    uros_iniciar();

    vTaskDelay(pdMS_TO_TICKS(500));
    mostrar_menu();
    printf("TABLA,punto,esperado_deg,RAW_ANGLE,raw_deg,ANGLE,ang_deg,AGC,MAG\n");

    while (1) {
        char c;
        if (consola_hay_tecla(&c)) {
            switch (c) {
                case 's': as5600_escanear_bus(); break;
                case 'd': cmd_diagnostico();     break;
                case 'a': cmd_angulo();          break;
                case 'z': cmd_setear_cero();     break;
                case 'Z': cmd_reset_cero();      break;
                case 'v': cmd_ver_cero();        break;
                case 't': cmd_tabla();           break;
                case 'r': cmd_reiniciar_tabla(); break;
                case 'h': mostrar_menu();        break;
                case 'c':
                    s_streaming = !s_streaming;
                    printf("Streaming %s\n", s_streaming ? "ON" : "OFF");
                    break;
                case 'p': {
                    pote_muestra_t p;
                    if (pote_leer(&p) == ESP_OK) {
                        printf("Pote raw=%4d mV=%4d ref=%4d ang=%6.1f deg\n",
                               p.raw_senal, p.mv_senal, p.mv_ref, p.grados);
                    } else {
                        printf("ERROR: fallo la lectura del potenciometro\n");
                    }
                    break;
                }
                case 'P': {
                    pote_muestra_t p;
                    if (pote_leer_mediana(&p) == ESP_OK) {
                        printf("Pote MEDIANA(n=%d) raw=%4d mV=%4d ref=%4d ang=%6.1f deg\n",
                               POTE_MEDIANA_N, p.raw_senal, p.mv_senal, p.mv_ref, p.grados);
                    } else {
                        printf("ERROR: fallo la lectura del potenciometro (mediana)\n");
                    }
                    break;
                }
                default: break;
            }
        }

        if (s_streaming) {
            as5600_muestra_t m;
            if (as5600_leer(&m) == ESP_OK) {
                printf("STREAM,%u,%.3f,%u,%.3f,%u,%u\n",
                       m.raw_angle, m.grados_raw,
                       m.angle,     m.grados,
                       m.agc, m.magnitud);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
