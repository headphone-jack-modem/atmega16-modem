#ifndef MODEM_H
#define MODEM_H
/*
 * Title:    C include file for the AVR Modem library (avr-modem.h)
 * Author:   Hasan Karimi <hasankarimi.dev@gmail.com>
 */

#ifndef BAUD
#define BAUD          (1600)
#endif
#ifndef LOW_FREQ
#define LOW_FREQ      (4800)
#endif
#ifndef HIGH_FREQ
#define HIGH_FREQ     (11200)
#endif

// the buffer size must be 2^x, such as {2, 4, 8, 16, 32, ...}
#ifndef TX_BUFF_SIZE
#define TX_BUFF_SIZE  4
#endif
#define TX_RSLT_OK    0
#define TX_RSLT_BUSY  1

#ifndef MODEM_DEBUG
#define MODEM_DEBUG   1
#endif

extern uint8_t modem_snd_async(const char* msg, uint8_t size);

extern void modem_lsn(void (*lsn_cb)(const char* msg, uint8_t size));

extern uint16_t info();

#endif //MODEM_H
