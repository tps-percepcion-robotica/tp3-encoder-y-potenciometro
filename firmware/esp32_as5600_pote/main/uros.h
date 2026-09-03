#pragma once

/* Arranca la tarea de FreeRTOS que maneja micro-ROS: conecta WiFi,
   crea el nodo y los publicadores, y publica periodicamente el
   potenciometro y el AS5600 (si esta conectado). No bloquea: crea
   la tarea y vuelve enseguida. */
void uros_iniciar(void);
