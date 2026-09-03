#include "pote.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_log.h"

static const char *TAG = "pote";

#define CANAL_SENAL   ADC_CHANNEL_7   /* GPIO35 */
#define CANAL_REF     ADC_CHANNEL_6   /* GPIO34 */
#define ATENUACION    ADC_ATTEN_DB_12

/* Constantes medidas el 2026-09-02 con el circuito actual (Rb=2.02k).
   Ganancia real medida ~1.913, no 2.0 exacto -- ver bitacora.
   Estos numeros SOLO alimentan el campo 'grados' de diagnostico local;
   la calibracion que efectivamente mueve el joint en RViz es la de
   dos puntos (raw_min/raw_max) que vive en el nodo Python de la PC. */
#define GANANCIA_AMP  1.913f
#define R_POTE        9740.0f
#define R_PIE         2020.0f
#define RECORRIDO_DEG 300.0f

static adc_oneshot_unit_handle_t s_adc  = NULL;
static adc_cali_handle_t         s_cali = NULL;

static esp_err_t crear_calibracion(void)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ATENUACION,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_cali_create_scheme_curve_fitting(&cfg, &s_cali);
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cfg = {
        .unit_id  = ADC_UNIT_1,
        .atten    = ATENUACION,
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_cali_create_scheme_line_fitting(&cfg, &s_cali);
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t pote_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&unit_cfg, &s_adc);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ATENUACION,
    };
    err = adc_oneshot_config_channel(s_adc, CANAL_SENAL, &chan_cfg);
    if (err != ESP_OK) return err;
    err = adc_oneshot_config_channel(s_adc, CANAL_REF, &chan_cfg);
    if (err != ESP_OK) return err;

    if (crear_calibracion() != ESP_OK) {
        ESP_LOGW(TAG, "Sin calibracion de fabrica, solo cuentas crudas");
        s_cali = NULL;
    }
    return ESP_OK;
}

static void calcular_grados(pote_muestra_t *m)
{
    float v_cursor = m->mv_senal / GANANCIA_AMP;
    float v_arriba = m->mv_ref;
    float v_abajo  = v_arriba * R_PIE / (R_POTE + R_PIE);
    float span = v_arriba - v_abajo;
    m->grados = (span > 1.0f)
              ? ((v_cursor - v_abajo) / span) * RECORRIDO_DEG
              : 0.0f;
}

esp_err_t pote_leer(pote_muestra_t *m)
{
    esp_err_t err;

    err = adc_oneshot_read(s_adc, CANAL_SENAL, &m->raw_senal);
    if (err != ESP_OK) return err;

    err = adc_oneshot_read(s_adc, CANAL_REF, &m->raw_ref);
    if (err != ESP_OK) return err;

    m->mv_senal = m->raw_senal;
    m->mv_ref   = m->raw_ref;
    if (s_cali) {
        adc_cali_raw_to_voltage(s_cali, m->raw_senal, &m->mv_senal);
        adc_cali_raw_to_voltage(s_cali, m->raw_ref,   &m->mv_ref);
    }

    calcular_grados(m);
    return ESP_OK;
}

/* n lecturas de un canal, ordenadas por insercion (n<=10, no hace falta
   qsort), se devuelve el valor central. Lecturas individuales que fallan
   simplemente no se cuentan -- no abortan la mediana completa. */
static esp_err_t leer_mediana_canal(adc_channel_t canal, int n, float *mediana_out)
{
    int muestras[POTE_MEDIANA_N];
    if (n > POTE_MEDIANA_N) n = POTE_MEDIANA_N;

    int validas = 0;
    for (int i = 0; i < n; i++) {
        int valor;
        if (adc_oneshot_read(s_adc, canal, &valor) == ESP_OK) {
            muestras[validas++] = valor;
        }
    }
    if (validas == 0) {
        return ESP_FAIL;
    }

    for (int i = 1; i < validas; i++) {
        int clave = muestras[i];
        int j = i - 1;
        while (j >= 0 && muestras[j] > clave) {
            muestras[j + 1] = muestras[j];
            j--;
        }
        muestras[j + 1] = clave;
    }

    *mediana_out = (validas % 2 == 0)
                 ? (muestras[validas/2 - 1] + muestras[validas/2]) / 2.0f
                 : (float)muestras[validas/2];
    return ESP_OK;
}

esp_err_t pote_leer_mediana(pote_muestra_t *m)
{
    float mediana_senal, mediana_ref;
    esp_err_t err;

    err = leer_mediana_canal(CANAL_SENAL, POTE_MEDIANA_N, &mediana_senal);
    if (err != ESP_OK) return err;

    err = leer_mediana_canal(CANAL_REF, POTE_MEDIANA_N, &mediana_ref);
    if (err != ESP_OK) return err;

    m->raw_senal = (int)(mediana_senal + 0.5f);
    m->raw_ref   = (int)(mediana_ref   + 0.5f);

    m->mv_senal = m->raw_senal;
    m->mv_ref   = m->raw_ref;
    if (s_cali) {
        adc_cali_raw_to_voltage(s_cali, m->raw_senal, &m->mv_senal);
        adc_cali_raw_to_voltage(s_cali, m->raw_ref,   &m->mv_ref);
    }

    calcular_grados(m);
    return ESP_OK;
}
