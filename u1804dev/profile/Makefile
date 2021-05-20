# Copyright (c) 2021 by Lilee Technology, Ltd.
# All rights reserved.
#
# Makefile for creating LileeOS Container diag shell service profile

TARGET?=.
SRC=u1804dev.env u1804dev.yml global.yml lilee.env
target=$(TARGET)/u1804dev.zip

all: $(target)

$(target): $(SRC)
	zip $(target) $(SRC)

clean:
	rm -f $(target)
