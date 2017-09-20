FROM ubuntu
MAINTAINER Kelvin Abrokwa (kelvinabrokwa@gmail.com)
RUN apt-get update
RUN apt-get install -y --force-yes build-essential arduino arduino-core
RUN mkdir /build
ADD ./ /build
WORKDIR /build
CMD ["make"]
