all:
	avr-gcc -g -Os -mmcu=atmega32 -c blink.c
	avr-gcc -g -mmcu=atmega32 -o blink.elf blink.o
	avr-objcopy -j .text -j .data -O ihex blink.elf blink.hex

docker-build:
	docker build -t firmware-builder .
	docker run -it firmware-builder

clean:
	rm -f *.o *.elf *.hex
