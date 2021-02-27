CC=avr-gcc
OBJCOPY=avr-objcopy
CPU          = atxmega128a4u
PCPU         = x128a4u
F_CPU        = 32000000

CFLAGS=  -Wall -Os -mmcu=$(CPU) -Wextra
CFLAGS += -std=gnu99



tpdu_logger.elf:	tpdu_logger.c
		$(CC) $(CFLAGS) tpdu_logger.c -o tpdu_logger.elf

tpdu_logger.hex:	tpdu_logger.elf
		$(OBJCOPY) -O ihex -R .eeprom -R .fuse -R .lock tpdu_logger.elf tpdu_logger.hex

	
program:	tpdu_logger.hex
		avrdude -p $(PCPU) -c avrispmkII -e -v -U flash:w:tpdu_logger.hex
#		avrdude -p $(PCPU) -c avrispmkII -v -U fuse2:w:0xff:m	

program_read:
		avrdude -p $(PCPU) -c avrispmkII  -v -U flash:r:tpdu_logger.bin:r
		

eeprom_read:
		avrdude -p $(PCPU) -c avrispmkII -v -U eeprom:r:eeprom.bin:r
		hexdump -C eeprom.bin
#		avrdude -p $(PCPU) -c avrispmkII -v -U fuse2:r:-:i
			