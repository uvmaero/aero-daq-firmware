#--------------------------------------
# Set device, directories and programmer here
#--------------------------------------

# main file
TARGET = pedal_board

# directories
SRC = src

# device parameters
DEVICE     = atmega328p
CLOCK      = 8000000
PROGRAMMER = usbtiny
LFUSE      = 0xE2
HFUSE      = 0xD8
EFUSE      = 0xFF

# compilers
CC      = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
AVRDUDE = avrdude -c $(PROGRAMMER) -p $(DEVICE)
CFLAGS  = -g -mmcu=$(DEVICE) -Os -Wall -std=c99 -DF_CPU=$(CLOCK) -I.
LFLAGS  = -mmcu=$(DEVICE)

CANPROG = python3.6 /home/cullen/Documents/Programming/atmega328p-can-bootloader/can_prog.py

#--------------------------------------
# No need to edit the below code
#--------------------------------------

# source files and object directory
SOURCES = $(wildcard $(SRC)/*.c)
OBJECTS = $(patsubst %.c, %.o, $(SOURCES))

# get file dependencies then build hex
all: Makefile.deps $(TARGET).hex

# include object dependencies
-include Makefile.deps

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LFLAGS) -o $@ $^

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

# generate object dependencies
Makefile.deps: $(SOURCES)
	$(CC) $(CFLAGS) -MM $^ > Makefile.deps

# remove build files
clean:
	@rm src/*.o $(TARGET).elf $(TARGET).hex Makefile.deps

# flash to micro-controller
flash: $(TARGET).hex
	$(AVRDUDE) -U flash:w:$<

can: $(TARGET).hex
	$(CANPROG) -p can0 -b 500000 -i 2D $<

# set fuses
fuses:
	$(AVRDUDE) -U lfuse:w:$(LFUSE):m -U hfuse:w:$(HFUSE):m -U efuse:w:$(EFUSE):m

# show fuses already set on micro-controller
show_fuses:
	$(AVRDUDE) -nv

# disassemble the code
disassemble:
	$(OBJDUMP) -d $(TARGET).elf > $(TARGET).asm