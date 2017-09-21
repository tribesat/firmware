FROM ubuntu

MAINTAINER Kelvin Abrokwa (kelvinabrokwa@gmail.com)

# install dependencies
RUN apt-get update && apt-get install -y \
        build-essential \
        gcc-avr \
        binutils-avr \
        avr-libc
