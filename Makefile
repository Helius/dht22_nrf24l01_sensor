CC = avr-gcc
MCU = atmega328
#DEBUG = -g
TARGET = cooler
F_CPU = 16000000

CCFLAGS = -DF_CPU=$(F_CPU) -mmcu=$(MCU) -Wall $(DEBUG) -I../ -std=gnu99 -Os -ffunction-sections -Wa,-adhlns=$(<:.c=.lst)
LDFLAGS = -Wl,-Map,$(TARGET).map,--gc-sections -mmcu=$(MCU)

all: $(TARGET).elf


$(TARGET).elf: $(TARGET).o uart.o dht.o ../nrf24L01_plus/nrf24.o radioPinFunctions.o
	$(CC) $^ -o $@ $(LDFLAGS) #$(CCFLAGS)
	avr-objcopy -j .text -j .data -O ihex $(TARGET).elf $(TARGET).hex
	avr-size --mcu=$(MCU) $(TARGET).elf


%.o: %.c
	$(CC) -c $< $(CCFLAGS) -o $(*).o

clean:
	rm -f *.o *.elf *.hex *.map *.lst

load: all
	avreal -aft2232:enable=~adbus4 +ATmega328p -evw -c $(TARGET).hex
#	avreal +ATmega8 -aft2232 -evw -c $(TARGET).hex
