#include "ibus_rx.hpp"
#include <stdio.h>
#include "pico/stdlib.h"
#include "string.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

static IBUS_Decode *ibus_irq_instance;
static void ibus_irq(){
    ibus_irq_instance->last_packet_time = time_us_32();
    while (uart_is_readable(ibus_irq_instance->uart_id)) {
        uint8_t ch = uart_getc(ibus_irq_instance->uart_id);
        if(!ibus_irq_instance->packet_started){
            if(ch == 0x20){
                ibus_irq_instance->packet_started = true;
                ibus_irq_instance->live_data_index = 0;
                ibus_irq_instance->live_data[ibus_irq_instance->live_data_index] = ch;
            }
        }else{
            ibus_irq_instance->live_data_index++;
            ibus_irq_instance->live_data[ibus_irq_instance->live_data_index] = ch;
            if(ibus_irq_instance->live_data_index == 31){
                ibus_irq_instance->packet_started = false;
                ibus_irq_instance->live_data_index = 0;
            }
        }
    }
}

void IBUS_Decode::init(uart_inst_t *uart, uint uart_pin_tx, uint uart_pin_rx) {
    // Initialize UART with the specified pins and standard iBUS baud rate (115200)
    gpio_init(uart_pin_tx);
    gpio_init(uart_pin_rx);
    gpio_set_function(uart_pin_tx, GPIO_FUNC_UART);
    gpio_set_function(uart_pin_rx, GPIO_FUNC_UART);
    uart_init(uart, 115200);
    uart_set_hw_flow(uart, false, false);
    uart_set_format(uart, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart, true);
    int UART_IRQ = uart == uart0 ? UART0_IRQ : UART1_IRQ;
    ibus_irq_instance = this;
    // setup an interrupt for software flow control
    irq_set_exclusive_handler(UART_IRQ, ibus_irq);
    irq_set_enabled(UART_IRQ, true);
    uart_set_irq_enables(uart, true, false);

    uart_id = uart;
}

void IBUS_Decode::drain_fifo() {
    while (uart_is_readable(uart_id)) {
        uart_getc(uart_id);
    }
}

void IBUS_Decode::reset() {
    ibus_rx.state = 0;
    ibus_rx.len = 0;
    ibus_rx.chksum = 0;
    memset(ibus_rx.buf, 0, 32);
    memset(ibus_channels.channels, 0, 14);
}

bool IBUS_Decode::timed_out() {
    if (time_us_32() - last_packet_time > timeout_val) {
        packet_timeout = true;
    } else {
        packet_timeout = false;
    }
    return packet_timeout;
}

void IBUS_Decode::decode() {
    // disable the interrupt handler while we're processing the data
    uart_set_irq_enables(uart_id, false, false);
    if(!packet_started){
        // compute the checksum
        uint16_t chksum = 0xFFFF;
        for (int i = 0; i < 31; i++) { // Loop through all bytes except the last one
            chksum -= live_data[i];
        }

        // Subtract the second the last byte (shifted by 8 bits) from the checksum (MSByte)
        chksum -= live_data[31] << 8;

        // Check if the checksum is zero, indicating a valid packet
        if (chksum == 0) {
            // Packet is valid, process the data
            for (int i = 0; i < 14; ++i) {
                ibus_channels.channels[i] = (live_data[2 * i + 2] | live_data[2 * i + 3] << 8);
            }
        }
    }
    // re-enable the interrupt handler
    uart_set_irq_enables(uart_id, true, false);
}

uint16_t IBUS_Decode::get_channel(uint8_t channel) {
    if (channel < 14) {
        return ibus_channels.channels[channel];
    }
    return 0; // Return 0 if the channel is out of range
}

void IBUS_Decode::get_channels(uint16_t *channels, uint8_t min, uint8_t max) {
    if (min < max && max < 14) {
        for (uint8_t i = min; i <= max; i++) {
            channels[i - min] = ibus_channels.channels[i];
        }
    }
}

void IBUS_Decode::get_all_channels(uint16_t *channels) {
    for (int i = 0; i < 14; i++) {
        channels[i] = ibus_channels.channels[i];
    }
}

