#include "uros.h"
#include "esp_wifi.h"

#include <stdio.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include "uxr/client/config.h"

#include "pote.h"
#include "as5600.h"

/* RCCHECK: para fallos de INICIALIZACION (crear nodo, publicador, etc).
   Si algo de esto falla, no hay forma sensata de seguir: se aborta la
   tarea. RCSOFTCHECK: para fallos en tiempo de ejecucion (un publish
   puntual). No abortamos por uno solo; seguimos al proximo ciclo. */
#define RCCHECK(fn)  { rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        printf("micro-ROS fallo linea %d: %d. Abortando.\n", __LINE__, (int)temp_rc); \
        vTaskDelete(NULL); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; \
    if ((temp_rc != RCL_RET_OK)) { \
        printf("micro-ROS fallo linea %d: %d. Continuando.\n", __LINE__, (int)temp_rc); } }

#define PUBLICAR_PERIODO_MS  50   /* 1000/50 = 20 Hz */
#define MICRO_ROS_TASK_STACK  16000
#define MICRO_ROS_TASK_PRIO   5

static rcl_publisher_t pub_pote_raw;
static rcl_publisher_t pub_encoder_raw;
static rcl_publisher_t pub_encoder_detectado;

static std_msgs__msg__Int32 msg_pote_raw;
static std_msgs__msg__Int32 msg_encoder_raw;
static std_msgs__msg__Bool  msg_encoder_detectado;

/* Se ejecuta cada PUBLICAR_PERIODO_MS, llamado por el executor (no por
   nosotros directamente). Lee los sensores y publica lo que haya. */
static void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);
    if (timer == NULL) return;

    pote_muestra_t p;
    if (pote_leer_mediana(&p) == ESP_OK) {
        msg_pote_raw.data = p.raw_senal;
        RCSOFTCHECK(rcl_publish(&pub_pote_raw, &msg_pote_raw, NULL));
    }
    /* si falla, no publicamos: el nodo de la PC conserva el ultimo
       valor recibido. Preferible a publicar un dato inventado. */

    as5600_muestra_t a;
    // esp_err_t err_as5600 = as5600_leer(&a);

    msg_encoder_detectado.data = false;
    RCSOFTCHECK(rcl_publish(&pub_encoder_detectado, &msg_encoder_detectado, NULL));

    if (err_as5600 == ESP_OK) {
        msg_encoder_raw.data = a.raw_angle;
        RCSOFTCHECK(rcl_publish(&pub_encoder_raw, &msg_encoder_raw, NULL));
    }
    /* sin AS5600 conectado, err_as5600 va a ser distinto de ESP_OK en
       cada ciclo: no se publica raw_angle nuevo, pero SI se publica
       detectado=false, asi el nodo de la PC sabe que no hay que confiar
       en el ultimo raw_angle. */
}

static void micro_ros_task(void *arg)
{
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    /* Le decimos a micro-ROS la IP y puerto del agente ANTES de
       inicializar -- viene de lo que cargaste en menuconfig
       (micro-ROS Settings). Sin esto, intentaria autodescubrimiento
       por broadcast, que es mas fragil. */
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
    RCCHECK(rmw_uros_options_set_udp_address(
        CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "esp32_potenciometro_encoder", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &pub_pote_raw, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "potenciometro/raw"));

    RCCHECK(rclc_publisher_init_default(
        &pub_encoder_raw, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "as5600/raw_angle"));

    RCCHECK(rclc_publisher_init_default(
        &pub_encoder_detectado, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "as5600/detectado"));

    rcl_timer_t timer;
    RCCHECK(rclc_timer_init_default(
        &timer, &support, RCL_MS_TO_NS(PUBLICAR_PERIODO_MS), timer_callback));

    /* El executor es lo que efectivamente llama a timer_callback cuando
       corresponde. "1" es la cantidad de handles (timers/subscripciones)
       que va a manejar -- tenemos un solo timer. */
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    msg_pote_raw.data = 0;
    msg_encoder_raw.data = 0;
    msg_encoder_detectado.data = false;

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        usleep(10000);
    }

    RCCHECK(rcl_publisher_fini(&pub_pote_raw, &node));
    RCCHECK(rcl_publisher_fini(&pub_encoder_raw, &node));
    RCCHECK(rcl_publisher_fini(&pub_encoder_detectado, &node));
    RCCHECK(rcl_node_fini(&node));
    vTaskDelete(NULL);
}

void uros_iniciar(void)
{
#ifdef UCLIENT_PROFILE_UDP
    /* Conecta el WiFi usando el SSID/password que cargaste en
       menuconfig. Bloquea hasta conectar. */
    ESP_ERROR_CHECK(uros_network_interface_initialize());
    esp_wifi_set_ps(WIFI_PS_NONE);
#endif
    xTaskCreate(micro_ros_task,
                "uros_task",
                MICRO_ROS_TASK_STACK,
                NULL,
                MICRO_ROS_TASK_PRIO,
                NULL);
}
