#include "consola.h"

#include <stdio.h>
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_vfs_dev.h"

void consola_init(void)
{
    setvbuf(stdin, NULL, _IONBF, 0);

    uart_vfs_dev_port_set_rx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CR);
    uart_vfs_dev_port_set_tx_line_endings(UART_NUM_0, ESP_LINE_ENDINGS_CRLF);

    const uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &cfg);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);
    uart_vfs_dev_use_driver(UART_NUM_0);
}

bool consola_hay_tecla(char *c)
{
    int leido = fgetc(stdin);
    if (leido == EOF) {
        return false;
    }
    *c = (char)leido;
    return true;
}
