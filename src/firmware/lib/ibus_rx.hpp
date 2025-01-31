#ifndef IBUS_RX_HPP
#define IBUS_RX_HPP

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

/*
  Sample iBUS data: 
    20 40 DB 5 DC 5 54 5 DC 5 E8 3 D0 7 D2 5 E8 3 DC 5 DC 5 DC 5 DC 5 DC 5 DC 5 DA F3
  Explanation
    Protocol length: 20
    Command code: 40 
    Channel 0: DB 5  -> value 0x5DB
    Channel 1: DC 5  -> value 0x5Dc
    Channel 2: 54 5  -> value 0x554
    Channel 3: DC 5  -> value 0x5DC
    Channel 4: E8 3  -> value 0x3E8
    Channel 5: D0 7  -> value 0x7D0
    Channel 6: D2 5  -> value 0x5D2
    Channel 7: E8 3  -> value 0x3E8
    Channel 8: DC 5  -> value 0x5DC
    Channel 9: DC 5  -> value 0x5DC
    Channel 10: DC 5 -> value 0x5DC
    Channel 11: DC 5 -> value 0x5DC
    Channel 12: DC 5 -> value 0x5DC
    Channel 13: DC 5 -> value 0x5DC
    Checksum: DA F3 -> calculated by adding up all previous bytes, total must be FFFF
 */

class IBUS_Decode{
    public:
        void init(uart_inst_t *uart, uint uart_pin_tx, uint uart_pin_rx);
        void decode();
        uint16_t get_channel(uint8_t channel);
        void get_channels(uint16_t *channels, uint8_t min, uint8_t max);
        void get_all_channels(uint16_t *channels);
        void on_uart_rx();
        void drain_fifo();
        void reset();
        bool timed_out();

        // these have to be public for the interrupt handler to access them
        volatile uint8_t live_data_index = 0;
        volatile bool packet_started = false;
        volatile char live_data[32] = {0};
        volatile uint32_t last_packet_time = 0;  // last time we received a packet in us
        uart_inst_t *uart_id;

    private:
        const static uint32_t timeout_val = 1000000; // 1 second
        bool packet_timeout = false;

        struct ibus_rx_state {
            uint8_t state = 0;
            uint8_t len = 0;
            uint8_t buf[32] = {0};
            uint16_t chksum = 0;
        } ibus_rx;

        struct ibus_channels {
            uint16_t channels[14] = {0};
        } ibus_channels;
};

#endif // IBUS_RX_HPP