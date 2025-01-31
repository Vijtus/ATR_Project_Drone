#include "bt_telem.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"
#include "hardware/uart.h"

static uint8_t bt_buffer[1024];
const uint reserved_len = 11; // 8 for the timestamp, 2 for the \r\n, 1 for the null terminator
static uart_inst_t *bt_uart;

void bt_init(uart_inst_t *uart, uint uart_pin_tx, uint uart_pin_rx) {
    bt_uart = uart;

    uart_init(uart, 9600);

    gpio_set_function(uart_pin_tx, GPIO_FUNC_UART);
    gpio_set_function(uart_pin_rx, GPIO_FUNC_UART);

    uart_set_baudrate(uart, 9600);

    uart_set_fifo_enabled(uart, false);
}

uint buffer_content_length() {
    uint i = 0;
    while (bt_buffer[i] != 0 && i < 1024) {
        i++;
    }
    return i;
}

bool buffer_append(char *data, uint len) {
    uint length = buffer_content_length(); // Calculate the length once
    if (len + length + 3 + reserved_len > 1024) { // 3 for the separator
        return false;
    }

    // append a " | " if the buffer is not empty
    if (length != 0) {
        bt_buffer[length] = ' ';
        bt_buffer[length + 1] = '|';
        bt_buffer[length + 2] = ' ';
        length += 3; // Update the length after adding separator
    }

    for (uint i = 0; i < len; i++) {
        bt_buffer[length + i] = data[i]; // Use the updated length
    }

    bt_buffer[length + len] = '\0';

    return true;
}

void buffer_clear() {
    memset(bt_buffer, 0, 1024);
    // append the timestamp
    uint32_t timestamp = time_us_32();
    for (uint i = 0; i < 8; i++) {
        bt_buffer[i] = (timestamp >> (28 - i * 4)) & 0xF;
        if (bt_buffer[i] < 10) {
            bt_buffer[i] += '0';
        } else {
            bt_buffer[i] += 'A' - 10;
        }
    }
}

void buffer_send() {
    // append the \r\n and the null terminator
    uint length = buffer_content_length();
    bt_buffer[length] = '\r';
    bt_buffer[length + 1] = '\n';
    bt_buffer[length + 2] = 0;
    uart_write_blocking(bt_uart, bt_buffer, buffer_content_length());
}
