CC      = gcc
CFLAGS  = -Wall -Wextra -std=c99 -I..
TARGET  = linux_spi_master
SRCS    = linux_spi_master.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)

.PHONY: all clean
