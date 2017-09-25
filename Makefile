CC = avr-gcc
OBJCOPY = avr-objcopy
MMCU = atmega328p
CFLAGS = -g -Os -mmcu=$(MMCU) -c -Wall
LDFLAGS = -g -mmcu=$(MMCU)
HEXFLAGS = -j .text -j .data -O ihex
SOURCES = blink.cpp
OBJECTS = $(SOURCES:.cpp=.o)
BINARY = blink.elf
HEX = blink.hex

all: $(HEX)

# compile .cpp files to .o
.cpp.o:
	$(CC) $(CFLAGS) $< -o $@

# link .o files to .elf binary
$(BINARY): $(OBJECTS)
	$(CC) $(LDFLAGS) $^ -o $@

# convert .elf to hex file
$(HEX): $(BINARY)
	$(OBJCOPY) $(HEXFLAGS) $< $@

docker-build:
	mkdir -p build
	docker build -t firmware-builder .
	docker run -it -v `pwd`/:/firmware firmware-builder /bin/bash -c "cd firmware; make"

get-deps:
	sudo apt-get install -y build-essential gcc-avr avr-libc

clean:
	rm -f *.o *.elf *.hex
