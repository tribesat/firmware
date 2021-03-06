CC = avr-gcc
OBJCOPY = avr-objcopy

MMCU = atmega328p
PROGRAMMER = avrispmkII

CFLAGS = -g -Os -mmcu=$(MMCU) -c -Wall
LDFLAGS = -g -mmcu=$(MMCU)
HEXFLAGS = -j .text -j .data -O ihex

DEFS = -DF_CPU=16000000UL

SOURCES = ./src/blink.cpp
OBJECTS = $(SOURCES:.cpp=.o)
BINARY = blink.elf
HEX = blink.hex

all: #$(HEX)

# compile .cpp files to .o
.cpp.o:
	$(CC) $(CFLAGS) $(DEFS) $< -o $@

# link .o files to .elf binary
$(BINARY): $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@

# convert .elf to hex file
$(HEX): $(BINARY)
	$(OBJCOPY) $(HEXFLAGS) $< $@

# symlinks this repo to the Arduino libraries directory so that its
# code can be included in Arduino Sketches written in the IDE
develop:
	ln -s `pwd` $(ARDUINO_LIB_DIR)/TribeSat

docker-build:
	docker build -t firmware-builder .
	docker run --rm -v `pwd`/:/firmware firmware-builder make

flash-arduino-uno: $(HEX)
	avrdude -c $(PROGRAMMER) -p $(MMCU) -P $(USB) -U flash:w:$< -v

get-deps:
	sudo apt-get install -y build-essential gcc-avr avr-libc

clean:
	rm -f ./src/*.o *.elf *.hex
