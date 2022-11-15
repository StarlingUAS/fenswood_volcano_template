MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
DOCKERFILE:=$(MAKEFILE_DIR)/../fenswood_drone_controller

IMAGE_NAME?=fenswood_drone_controller
NETWORK?=8_containers_default
ENV?=
RUN_ARGS?=

all: build

help:
	@echo "all - run build"
	@echo "build - build the dockerfile for this project"
	@echo "run - builds and runs the dockerfile for this project"
	@echo "run_bash - builds and runs the dockerfile putting you into a bash shell"
	@echo "help - show this help screen"

build:
	docker build -t $(IMAGE_NAME) $(DOCKERFILE)

run: build
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(RUN_ARGS) $(IMAGE_NAME)

run_bash: build
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true $(RUN_ARGS) $(IMAGE_NAME) bash

.PHONY: all help build run run_bash