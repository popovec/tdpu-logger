# tpdu-logger

Tool for capture card - card reader communiction

*************************************************************************

## Hardware

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

*************************************************************************

Device is able to determine correct baud rate from ATR and PPS exchange.
Specific mode is supported too.

Guard time (TC1) is partially supported.

For now no support for CWT and BWT.

Error signalig (protocol T0) is handled, but this code is untested.

Disclaimer
~~~~~~~~~~

Whole project is published in good faith and is to be used for educational
purposes only.

AUTHOR IS NOT RESPONSIBLE FOR ANY DAMAGE OF CARD, CARD READERS, COMPUTER
EQUIPMENTS OR DATA LOSS/DATA LEAKAGE! USE AT YOUR OWN RISK ONLY!

