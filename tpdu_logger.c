/*
    tpdu_logger.c

    This is part of OsEID (Open source Electronic ID)

    Copyright (C) 2021 Peter Popovec, popovec.peter@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

    Sniffer for ISO7816-3 (to determine ATR/PPS log APDU etc)
    

Disclaimer
~~~~~~~~~~

Whole project is published in good faith and is to be used for educational
purposes only. 

AUTHOR IS NOT RESPONSIBLE FOR ANY DAMAGE OF CARD, CARD READERS, COMPUTER
EQUIPMENTS OR DATA LOSS/DATA LEAKAGE! USE AT YOUR OWN RISK ONLY! 

*/

/*
Xmega128A4U is connected to card reader:

+-------------------+
|   xmega128A4U     |
|                   +-------< voltage regulator 3.3 V
| C    C    C    C  |
| 0    2    7    3  +-------< GND
+-+----+----+----+--+
  |    |    |    |
 -+-  -+-  -+-   +--< debug to USB serial converter 921.6kbaud
 | |  | |  | |
 | |  | |  | |
 | |  | |  | |  <---------------  3x 1kOhm resistor
 -+-  -+-  -+-
  |    |    |
 I/O  CLK  RST

- RST is handled by interrupt handler, falling edge is used to restart MCU
- FLASH/EEPROM is slow, we using cache in RAM (2KiB), EEPOM/FLASH write
  can be optimized - write only if whole page of flash/eeprom is available.
- If card is idle, flash/eeprom routine is called to save even partial block.
- MCU is clocked from 2MHZ oscilator over PLL (32MHz)
- TCC0 is used to measure pulse period, TCC0 input is connected over EVENT
  channel 0 to CLK
- to use external FLASH/EEPROM, you need write own functions for:
  a) eeprom_read_byte()
  b) eeprom_write_block()
  c) eeprom_is_ready()
  (https://www.nongnu.org/avr-libc/user-manual/group__avr__eeprom.html)

  FUSES: default values as descibed in xmega doc.

*/

#include <stdint.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
/*/////////////////////////////////////////////////////////////////////////////////////

Default connection:

PC7  over 1k Ohm resistor to RST
PC0  over 1k Ohm resistor to I/O
PC2  over 1k Ohm resistor to CLK

PC3 debug serial output (serial speed 921600)

Change below if needed
*/

#define P_RST_PORT PORTC
#define P_RST_BIT 7

#define P_IO_PORT PORTC
#define P_IO_PIN 0

// port A=0, port B=1, here PORTC -> 2
#define P_CLK_INDEX 2
#define P_CLK_PIN 2

// enable this if parity and error signal is to be logged
//#define LOG_PARITY

////////////////////////////////////////////////////////////////////////////////////

#define PPS_END 0
#define PPS_RESPONSE 1
#define PPS_REQUEST 2

#define P_RST (P_RST_PORT.IN & (1<<P_RST_BIT))
#define P_IO  (P_IO_PORT.IN & (1<<P_IO_PIN))

// forward declarations
void log_write(uint16_t size, uint8_t timeout);
//////////////////////////////////  Serial debug output functions ////////////
static void serial_putchar(uint8_t c)
{
//  while ((USARTC0.STATUS & 0x20)!=0x20);
	while (!(USARTC0_STATUS & USART_DREIF_bm)) ;
	USARTC0.DATA = c;
}

static void serial_print_nibble(uint8_t d)
{
	d &= 15;
	if (d > 9)
		d += 'a' - 10;
	else
		d += '0';
	serial_putchar(d);

}

static void serial_print_hex(uint8_t data)
{
	serial_putchar(' ');
	serial_print_nibble(data >> 4);
	serial_print_nibble(data);
}

static void serial_init()
{

// PC3 output
	PORTC.DIRSET = PIN3_bm;
	USARTC0_CTRLA = 0;	// no interrupts from usart
// enable transmiter only..
	USARTC0_CTRLB = USART_TXEN_bm;	// | USART_RXEN_bm;
	USARTC0_CTRLC = USART_CHSIZE_8BIT_gc;	// 8 bit no parity 1 stop bit
#if 0
	USARTC0.BAUDCTRLA = 107;	// 460.8
	USARTC0.BAUDCTRLB = 0xb0;
#else
	USARTC0.BAUDCTRLA = 75;	// 921.6
	USARTC0.BAUDCTRLB = 0xa0;
#endif
}

//////////////////////////////////  CPU init ////////////

void init_cpu(void) __attribute__((naked))
    __attribute__((section(".init1")));
void init_cpu(void)
{
	cli();
	// use PLL - multiply 2MHz RC oscilator to 32MHz
	OSC.PLLCTRL = OSC_PLLSRC_RC2M_gc | (32 / 2);
	OSC.CTRL |= OSC_PLLEN_bm;
	// wait to PLL ready
	while (!(OSC.STATUS & OSC_PLLRDY_bm)) ;
// switch CPU core clock to run from PLL
	{
		asm volatile (	//
				     "ldi r25,4\n"	//      value (clock source for CPU core..)
				     "ldi r24,0xd8\n"	//      key
				     "ldi r30,0x40\n"	//      0x0040 = CTRL reg address
				     "ldi r31,0\n"	//
				     "out 0x3b,r1\n"	//      clear RAMPZ
				     "out 0x34,r24\n"	//      write key to CCP
				     "st  Z,r25\n"	//      write value
				     "ldi r24,0x40\n"	//      delay aproximatly 1ms
				     "ldi r25,0x1f\n"	//
				     "sbiw r24,1\n"	//
				     "brne .-4\n"	//
				     :::);
	}
}

//////////////////////////////////  CPU restart ////////////

ISR(PORTC_INT0_vect)
{
	// save rest of data into EEPROM/FLASH
	log_write(0, 1);
	// reset MCU
	CCP = CCP_IOREG_gc;
	RST.CTRL = 1;
}

///////////////////////////////// log cache /////
// max cache size 32kB !!! (respect to device RAM size max 2kB..)

uint8_t cache[1024];
uint8_t *cache_read_position, *cache_write_position, *cache_block;
uint16_t cache_block_size;
uint8_t cache_full;

static uint8_t *cache_move_pointer(uint8_t * p)
{
	p++;
	if (p == &cache[sizeof(cache)])
		p = cache;
	return p;
}

static void cache_write_byte(uint8_t byte)
{
	// cache full ?
	if (cache_write_position < cache_read_position)
		cache_write_position += sizeof(cache);
	// reserve minimal 3 bytes!

	if ((size_t)((cache_write_position + 3) - cache_read_position) > sizeof(cache)) {
// TEMP code
//              eeprom_write_block(cache, (uint8_t *) 0, 1024);
//              for (;;) ;
// TEMP code end
// print F on debug console to signalize cache full
		if (cache_full != 0x80)
			serial_putchar('F');
		cache_full = 0x80;
		return;
	}
	if (cache_block_size == 0) {
		// save position for block length bytes
		cache_block = cache_write_position;
		// move to block start
		cache_write_position = cache_move_pointer(cache_write_position);
		cache_write_position = cache_move_pointer(cache_write_position);
	}
	*(cache_write_position) = byte;
	cache_write_position = cache_move_pointer(cache_write_position);
	cache_block_size++;
}

static void cache_finalize_block()
{
	*(cache_block) = (cache_block_size >> 8) | cache_full;
	cache_block = cache_move_pointer(cache_block);
	*(cache_block) = cache_block_size & 0xff;
	cache_block = cache_move_pointer(cache_block);
// write block to EEROM/FLASH) (but eeprom may refuse write if block is small)
// this is handled in eeprom code
	log_write(cache_block_size + 2, 0);

	cache_block_size = 0;
	cache_full = 0;
}

static void cache_init()
{
	memset(cache, 0xff, sizeof(cache));
	cache_read_position = cache;
	cache_write_position = cache;
	cache_block = cache;
	cache_full = 0;
	cache_block_size = 0;
}

static int16_t cache_read_byte()
{
	uint16_t r;

	if (cache_read_position == cache_write_position)
		return -1;
	r = *cache_read_position;
	cache_read_position = cache_move_pointer(cache_read_position);
	return r;
}

////////////////////////////////// EEPROM functions /////////////////////
// values corresponding to ATxmega16/32 EEPROM size
#define BLOCK_SIZE 32
#define MAX_LOG_SIZE 1024
// values corresponding to ATxmega64/128 EEPROM size
//#define BLOCK_SIZE 32
//#define MAX_LOG_SIZE 2048

static uint32_t log_data_pointer;
void log_init()
{
	uint8_t r;
//
// do not owerwrite EEPROm if already filled
	r = eeprom_read_byte(NULL);
	if (r != 0xff) {
		log_data_pointer = MAX_LOG_SIZE;
		return;
	}
	r = eeprom_read_byte(NULL + 1);
	if (r != 0xff) {
		log_data_pointer = MAX_LOG_SIZE;
		return;
	}
// ok eeprom is free
// start position in FLASH/EEPROM
	log_data_pointer = 0;
}

// cache is circular buffer, we need read data byte by byte
void log_write(uint16_t size, uint8_t timeout)
{
// block size for cache corresponds to TPDU block size
// write to EEPROM/FLASH must be optimized for FLASH block size
//
	static uint16_t data_size;
	uint8_t buffer[BLOCK_SIZE];	// flash page size
	uint8_t *b;
	int16_t r;
	uint16_t count;
	data_size += size;

	if (data_size == 0)
		return;
	// refuse block update if no data for full BLOCK_SIZE
	// and no timeout is signalized
	if ((data_size < BLOCK_SIZE) && timeout == 0)
		return;
// TODO change this if no internal EEPROM is used
	// ignore request, EEPROM is busy
	if (eeprom_is_ready() == 0)
		return;
//
	memset(buffer, 0xff, BLOCK_SIZE);
	b = buffer;
	for (count = 0; count < BLOCK_SIZE; count++) {
		r = cache_read_byte();
		// internal check, no enough bytes in cache
		if (r == -1)
			break;
		*(b++) = r;
	}
	if (count == 0)
		return;

	if (log_data_pointer + count > MAX_LOG_SIZE)
		return;

//  TODO change this if no internal EEPROM is used
	// here stupid converter from uint32_t to void * - because
	// eeprom_update_block uses void * as address eeprom address
	{
		volatile union convert_ {
			void *to;
			uint32_t from;
		} c;
		c.to = NULL;
		c.from = log_data_pointer;
//              serial_putchar('^');
		eeprom_write_block(buffer, c.to, BLOCK_SIZE);
	}
	log_data_pointer += BLOCK_SIZE;
	if (data_size > BLOCK_SIZE)
		data_size = 0;
	else
		data_size -= BLOCK_SIZE;
}

/////////////////////////////////////////////

struct card {
	uint16_t etu;
	uint16_t etu_from_ta1;
	uint8_t dir;
	uint8_t ta1;
	uint8_t protocol;
	uint8_t guard_time;
	uint8_t pps_state;
	uint8_t pps1_request;
	uint8_t pps1_response;
	uint8_t atr[32 * 2];
} card;

// return value:
// -1 timeout
// bit 0..7 = byte from reader/card
// bit 8 parity bit
// bit 9 there was error signal activated while reading this byte
int16_t sample_byte(struct card *c)
{
	uint8_t l, h, i, b = 0;
	int16_t byte = 0;
//
 repeat_sample_byte:
	l = (c->etu / 2) & 0xff;
	h = c->etu >> 9;
// wait line idle
	while (P_IO == 0) ;
// wait start bit
	TCC0.PER = (5 + c->guard_time) * c->etu;
	TCC0.INTFLAGS = 1;
	for (;;) {
		if (P_IO == 0)
			break;
// timeout no start bit .. 
		if (TCC0.INTFLAGS & 1)
			return -1;
	}
	TCC0.CNTL = l;
	TCC0.CNTH = h;
	TCC0.PER = c->etu;
	TCC0.INTFLAGS = 1;
	for (i = 0; i < 9; i++) {
		while ((TCC0.INTFLAGS & 1) == 0) ;
		if (P_IO)
			l = 0;
		else
			l = 1;
		if (c->dir) {
			b <<= 1;
			b |= l;
		} else {
			b >>= 1;
			b |= (l << 7);
		}

		TCC0.INTFLAGS = 1;
	}
// parity bit
	while ((TCC0.INTFLAGS & 1) == 0) ;
	TCC0.INTFLAGS = 1;
	if (P_IO)
		byte |= 0x100;
	else
		byte = 0;
	if (c->dir == 0)
		b = b ^ 0xff;
	byte |= b;
	if (c->protocol == 0) {
// stop bit (error signal?)
		while ((TCC0.INTFLAGS & 1) == 0) ;
		TCC0.INTFLAGS = 1;
		if (P_IO == 0) {
			byte = 0x200;
			goto repeat_sample_byte;
		}
	}
	return byte;
}

uint16_t parse_pps(uint8_t * data, uint16_t len, struct card *c)
{
	uint8_t pps_pck;
	uint16_t pps_ta = 0x8000;
	uint8_t count = 2;	// PPSS, PPS0
//
	// check pps
	pps_pck = 0xff;
	pps_pck ^= data[1];
	if (data[1] & 0x10) {
		pps_ta = data[2];
		pps_pck ^= data[count++];
		c->protocol = data[1] & 0xf;
	}
	if (data[1] & 0x20)
		pps_pck ^= data[count++];
	if (data[1] & 0x40)
		pps_pck ^= data[count++];
	if (len != (uint16_t) count + 1)
		return 0xffff;
	pps_pck ^= data[count];
	if (pps_pck)
		return 0xffff;
	return pps_ta;
}

// recelculate measured ETU value to new value (based on TA1 or PPS1)

static uint16_t etu_from_ta(uint8_t ta, uint16_t etu)
{
	uint32_t n;
	uint16_t F_table[] =
	    { 372, 372, 558, 744, 1116, 1488, 1860, 372, 372, 512, 768, 1024, 2048, 372, 372 };
	uint16_t D_table[] =
	    { 1 * 372, 1 * 372, 2 * 372, 4 * 372, 8 * 372, 16 * 372, 32 * 372, 64 * 372, 12 * 372,
		20 * 372, 372, 372, 372, 372, 372, 372
	};
//
	n = (uint32_t) etu *F_table[ta >> 4];
	n /= D_table[ta & 0x0f];
	return n & 0xffff;
}

static void parse_atr(struct card *c)
{
	uint16_t ta1_etu;
	uint8_t *t = c->atr;
	uint8_t td;
//
	c->etu_from_ta1 = c->etu;
	ta1_etu = c->etu;
	t++;
	// T0
	if (c->atr[1] & 0x10) {
		// TA
		ta1_etu = etu_from_ta(c->atr[2], c->etu);
		t++;
	}
	if (c->atr[1] & 0x20) {
		t++;
	}
	if (c->atr[1] & 0x40) {
		t++;
		c->guard_time = *t;
	}
	if (c->atr[1] & 0x80) {
		t++;
		td = *t;
		if (td & 0x10) {
			t++;
			// TA2 present
			if (*t & 0x80) {
				// specific mode
				c->protocol = td & 0xf;
				if ((*t & 0x10) == 0) {
					// Fi/Di defined in TA1 should be used
					c->etu_from_ta1 = ta1_etu;
					c->etu = ta1_etu;
				}
				c->pps_state = PPS_END;
			}
		}
	}
}

int main()
{
	uint8_t block[254 + 5];	// large enough for T1 protocol frame with CRC
	uint8_t l, h;
	uint8_t val, pval;
	uint16_t i;
	int16_t byte;
	uint8_t timeout;
//
	card.guard_time = 0;
	card.protocol = 0;
	serial_init();
	log_init();
	cache_init();
// enable inverted input if needed..
//      PORTC.PIN0CTRL = 0x40;
//      PORTC.PIN1CTRL = 0x40;
//      PORTC.PIN2CTRL = 0x40;
	EVSYS.CH0MUX = ((10 + P_CLK_INDEX) << 3) | P_CLK_PIN;
	TCC0.CTRLA = 8;		// timer source event 0
	TCC0.CTRLB = 0;		//
	TCC0.PER = 65535;
	serial_putchar('O');
	serial_putchar('K');
	serial_putchar(0x0a);
//
// INT0 in falling edge on RTS - restart whole MCU
	CCP = CCP_IOREG_gc;
	PMIC.CTRL = 7;

//      PORTC.DIRCLR = 0x80;
	P_RST_PORT.DIRCLR = (1 << P_RST_BIT);
//      PORTC.INT0MASK = 0x80;
	P_RST_PORT.INT0MASK = (1 << P_RST_BIT);
//      PORTC.INTCTRL = 1;
	P_RST_PORT.INTCTRL = 1;	// interrupt level
//      PORTC.PIN7CTRL = 2;
#define TOKENPASTE( x, y, z ) x ## y ## z
#define TOKENPASTE3( x, y, z ) TOKENPASTE( x, y, z )
#define _PINCTRL(x) TOKENPASTE3( PIN, x, CTRL )
	P_RST_PORT._PINCTRL(P_RST_BIT) = 2;
	sei();

// wait for rising edge on RST line
	while (P_RST != 0) ;
	while (P_RST == 0) ;
	serial_putchar('X');
	serial_putchar(0x0a);

	val = P_IO;
	TCC0.CNTL = 0;
	TCC0.CNTH = 0;
	pval = val;
	while (val == pval)
		val = P_IO;
	l = TCC0.CNTL;
	h = TCC0.CNTH;
	TCC0.CNTL = 0;
	TCC0.CNTH = 0;
// save RST to ATR start bit
	cache_write_byte(pval);
	cache_write_byte(h);
	cache_write_byte(l);
	serial_print_hex(pval);
	serial_print_hex(h);
	serial_print_hex(l);
	serial_putchar(0x0a);

	pval = val;
// save ATR start bit
	while (val == pval)
		val = P_IO;
	l = TCC0.CNTL;
	h = TCC0.CNTH;
	TCC0.CNTL = 0;
	TCC0.CNTH = 0;

	cache_write_byte(pval);
	cache_write_byte(h);
	cache_write_byte(l);
	serial_print_hex(pval);
	serial_print_hex(h);
	serial_print_hex(l);
	serial_putchar(0x0a);

	pval = val;
	while (val == pval)
		val = P_IO;
// save initial two bits from ATR
	l = TCC0.CNTL;
	h = TCC0.CNTH;
	TCC0.CNTL = 0;
	TCC0.CNTH = 0;
	card.etu = l | (h << 8);

	cache_write_byte(pval);
	cache_write_byte(h);
	cache_write_byte(l);
	serial_print_hex(pval);
	serial_print_hex(h);
	serial_print_hex(l);
	serial_putchar(0x0a);

	pval = val;
// one or 6 ETU
	while (val == pval)
		val = P_IO;
	l = TCC0.CNTL;
	h = TCC0.CNTH;
	TCC0.CNTL = 0;
	TCC0.CNTH = 0;

	cache_write_byte(pval);
	cache_write_byte(h);
	cache_write_byte(l);
	serial_print_hex(pval);
	serial_print_hex(h);
	serial_print_hex(l);
	serial_putchar(0x0a);

	if (card.etu > (uint16_t) (l | (h << 8))) {
		// 0x3b
		card.dir = 0;
		card.etu += l | (h << 8);
		pval = val;
//  3 ETU
		while (val == pval)
			val = P_IO;
		l = TCC0.CNTL;
		h = TCC0.CNTH;
		TCC0.CNTL = 0;
		TCC0.CNTH = 0;
		card.etu += l | (h << 8);
		pval = val;
//  2 ETU
		while (val == pval)
			val = P_IO;
		l = TCC0.CNTL;
		h = TCC0.CNTH;
		TCC0.CNTL = 0;
		TCC0.CNTH = 0;
		card.etu += l | (h << 8);
		card.atr[0] = 0x3b;
	} else {
		// 0x3f
		card.dir = 0xff;
		card.etu += l | (h << 8);
		card.atr[0] = 0x3f;
	}
	card.etu /= 8;
	cache_write_byte(card.dir);
	cache_write_byte(card.etu >> 8);
	cache_write_byte(card.etu & 0xff);

	serial_print_hex(card.dir);
	serial_print_hex(card.etu >> 8);
	serial_print_hex(card.etu & 0xff);
	serial_putchar(0x0a);

	cache_finalize_block();
	// read rest of ATR

	for (l = 1, i = 1; i < 32; i++) {
		byte = sample_byte(&card);
		if (byte < 0)
			break;
		card.atr[l++] = byte & 0xff;
#ifdef LOG_PARITY
		cache_write_byte(byte >> 8);
#endif
		cache_write_byte(byte & 0xff);
		serial_print_hex(byte & 0xff);
	}
	serial_putchar(0x0a);
	cache_finalize_block();

// if mode is negotiable, wait for PPS
	card.pps_state = PPS_REQUEST;
	parse_atr(&card);
// read data block
	for (;;) {
		serial_putchar('>');
		for (timeout = 0;; timeout++) {
			byte = sample_byte(&card);
			if (byte >= 0)
				break;
			// if card is idle, force log update
			if (timeout > 250) {
				log_write(0, 1);
				timeout = 0;
			}
		}
		for (i = 0; i < sizeof(block); i++) {
			if (byte < 0) {
				serial_putchar(0x0a);
				cache_finalize_block();
				break;
			}

			block[i] = byte & 0xff;
#ifdef LOG_PARITY
			cache_write_byte(byte >> 8);
#endif
			cache_write_byte(byte & 0xff);
			serial_print_hex(block[i]);
			byte = sample_byte(&card);
		}

		if (card.pps_state == PPS_REQUEST) {
			if (block[0] != 0xff)
				card.pps_state = PPS_END;
			else {
				uint16_t p;
//
				p = parse_pps(block, i, &card);
				if (p == 0xffff)
					card.pps_state = PPS_END;
				else if (p == 0x8000)	// no PPS1 in PPS request
					card.pps1_request = 0x11;	// use defaults (Fd = Fn = 372, Dd = Dn = 1)
				else
					card.pps1_request = p & 0xff;
				card.pps_state = PPS_RESPONSE;
			}
		} else if (card.pps_state == PPS_RESPONSE) {
			if (block[0] == 0xff) {
				uint16_t p;
//
				p = parse_pps(block, i, &card);
				if (p == 0xffff)
					card.pps_state = PPS_END;	// wrong response, continue with defaults
				if (p == 0x8000)	// no PPS1 in PPS response
					card.pps1_response = 0x11;	// use defaults (Fd = Fn = 372, Dd = Dn = 1)
				else if (p != 0xffff) {
					card.pps1_response = p & 0xff;
					// switch to new baud rate
					card.etu = etu_from_ta(card.pps1_response, card.etu);
				}

			}
			card.pps_state = PPS_END;
		}
	}
}
