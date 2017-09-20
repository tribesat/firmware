all:
	arduino blink.uno

docker-build:
	docker build -t firmware-builder .
	docker run -it firmware-builder
