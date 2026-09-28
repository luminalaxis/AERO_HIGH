CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Werror -O2

test: test_aero_frame
	./test_aero_frame

test_aero_frame: test_aero_frame.c aero_frame.c aero_frame.h
	$(CC) $(CFLAGS) -o $@ test_aero_frame.c aero_frame.c

clean:
	rm -f test_aero_frame

.PHONY: test clean
