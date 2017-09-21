all:
	avr-gcc -g -Os -mmcu=atmega32 -c blink.c
	avr-gcc -g -mmcu=atmega32 -o blink.elf blink.o
	avr-objcopy -j .text -j .data -O ihex blink.elf blink.hex

docker-build:
	mkdir -p build
	docker build -t firmware-builder .
	docker run -it -v `pwd`/:/firmware firmware-builder /bin/bash -c "cd firmware; make"

clean:
	rm -f *.o *.elf *.hex
