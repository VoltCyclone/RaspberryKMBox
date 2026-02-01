# Mouse Counteraction Tool - macOS Build

CC = clang
CFLAGS = -Wall -Wextra -O2
FRAMEWORKS = -framework CoreGraphics -framework ApplicationServices -framework Foundation
TARGET = mouse_counteract

all: $(TARGET)

$(TARGET): mouse_counteract.c
	$(CC) $(CFLAGS) -o $@ $< $(FRAMEWORKS)

clean:
	rm -f $(TARGET)

.PHONY: all clean