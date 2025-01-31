#ifndef BT_TELEM_HPP
#define BT_TELEM_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"

// message structure:
// 32 bit hex timestamp | message 1 | message 2 | ... | message n \r\n
// messages are automatically separated by |

void bt_init(uart_inst_t *uart, uint uart_pin_tx, uint uart_pin_rx);
bool buffer_append(char *data, uint len);
void buffer_clear();
void buffer_send();
uint buffer_content_length();


#endif // BT_TELEM_HPP