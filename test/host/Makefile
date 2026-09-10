CC ?= gcc
CFLAGS = -Wall -Wextra -Werror -std=c11 -g -O2

SRC = test_sntp_policy.c
BIN = test_sntp_policy

.PHONY: test clean

test: $(BIN)
	./$(BIN)

$(BIN): $(SRC) ../../include/apps/esp_lwgsm_sntp_policy.h
	$(CC) $(CFLAGS) -o $@ $(SRC)

clean:
	rm -f $(BIN) $(BIN).exe
