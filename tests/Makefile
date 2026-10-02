# tests/Makefile — host-side unit tests for the pure algorithm modules.
#
# Author: Luca Obwegs
#
# These tests link the real firmware source for IMU_KF.c / PID.c directly
# (no mocks) and build with the host's native gcc — no ARM toolchain
# required. Usage:
#   make -C tests        # build
#   make -C tests test   # build and run, non-zero exit on any failure

CC       := gcc
INC_DIR  := ../Core/Inc
SRC_DIR  := ../Core/Src
CFLAGS   := -std=c11 -Wall -Wextra -O2 -I. -I$(INC_DIR)
LDLIBS   := -lm

BIN_DIR  := build

TARGETS  := $(BIN_DIR)/test_imu_kf $(BIN_DIR)/test_pid

.PHONY: all test clean

all: $(TARGETS)

$(BIN_DIR):
	-mkdir $(BIN_DIR)

$(BIN_DIR)/test_imu_kf: test_imu_kf.c $(SRC_DIR)/IMU_KF.c | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

$(BIN_DIR)/test_pid: test_pid.c $(SRC_DIR)/PID.c | $(BIN_DIR)
	$(CC) $(CFLAGS) -o $@ $^ $(LDLIBS)

test: all
	$(BIN_DIR)/test_imu_kf
	$(BIN_DIR)/test_pid

clean:
	rm -rf $(BIN_DIR)
