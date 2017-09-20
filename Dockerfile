FROM ubuntu
MAINTAINER Kelvin Abrokwa (kelvinabrokwa@gmail.com)
RUN apt-get update
RUN apt-get install build-essential arduino arduino-core -y
RUN mkdir /build
ADD ./ /build
WORKDIR /build
CMD ["make"]
