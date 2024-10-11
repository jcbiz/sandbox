# Copyright (c) 2024 by Jeffrey Chang
# All rights reserved.
#
# Makefile for creating Docker CLI Shell container IMAGE

TARGET?=.
OWNER=jcb
NAME=dockershell
VERSION=201
SRC=
IMAGE_TAG=$(OWNER)/$(NAME):$(VERSION)
IMAGE=$(TARGET)/$(OWNER)_$(NAME)-$(VERSION)-docker.tgz
PROFILE=$(TARGET)/$(NAME).zip

IMG_SRC=Dockerfile \
	start.sh \
	docker-ce-cli_27.3.1-1_bookworm_amd64.deb \
	docker-compose-plugin_2.29.7-1_bookworm_amd64.deb

PROFILE_SRC=$(NAME).yml $(NAME).env global.yml launch.sh

all: $(IMAGE) $(PROFILE)

$(IMAGE): $(IMG_SRC)
	docker build . -f Dockerfile -t $(IMAGE_TAG)
	docker image save $(IMAGE_TAG) | gzip -9c > $(IMAGE)

$(PROFILE): $(PROFILE_SRC)
	zip $(PROFILE) $(PROFILE_SRC)

clean:
	rm -f $(PROFILE)
	rm -f $(IMAGE)
