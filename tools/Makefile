# Tools Build

CC = clang
CFLAGS = -Wall -Wextra -O2
FRAMEWORKS = -framework CoreGraphics -framework ApplicationServices -framework Foundation
INCLUDES = -I../lib/wire-protocol/include

all: mouse_counteract kmbox_relay

mouse_counteract: mouse_counteract.c
	$(CC) $(CFLAGS) -o $@ $< $(FRAMEWORKS)

kmbox_relay: kmbox_relay.c ../lib/wire-protocol/include/wire_protocol.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $<

# Cross-compile for Windows (requires mingw-w64: brew install mingw-w64)
kmbox_relay.exe: kmbox_relay.c ../lib/wire-protocol/include/wire_protocol.h
	x86_64-w64-mingw32-gcc $(CFLAGS) $(INCLUDES) -o $@ $< -lws2_32

clean:
	rm -f mouse_counteract kmbox_relay kmbox_relay.exe

.PHONY: all clean
