FROM alpine

MAINTAINER Kelvin Abrokwa (kelvinabrokwa@gmail.com)

# install dependencies
RUN apk add --update \
        alpine-sdk \
        gcc-avr \
        binutils-avr \
        avr-libc

ADD . firmware/
WORKDIR firmware/
