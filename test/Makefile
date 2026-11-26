CC      ?= gcc
CFLAGS  := -std=c11 -Wall -Wextra -Werror -Wpedantic
INC     := -I../components/nrf24l01/include -I../main -Imocks -I.
BUILD   := build
TARGET  := $(BUILD)/test_nrf24l01

SRCS    := ../components/nrf24l01/src/nrf24l01.c mock_nrf24_hw.c test_nrf24l01.c

.PHONY: all test check-main clean

all: test check-main

$(TARGET): $(SRCS)
	@mkdir -p $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(SRCS) -o $(TARGET)

test: $(TARGET)
	./$(TARGET)

check-main:
	$(CC) $(CFLAGS) $(INC) -fsyntax-only -DCONFIG_STATION_MODE_TX_INTERACTIVE=1 ../main/main.c
	$(CC) $(CFLAGS) $(INC) -fsyntax-only -DCONFIG_STATION_MODE_TX_BEACON=1 ../main/main.c
	$(CC) $(CFLAGS) $(INC) -fsyntax-only -DCONFIG_STATION_MODE_RX_MONITOR=1 ../main/main.c

clean:
	rm -rf $(BUILD)
