# Native (host) builds of the hardware-independent firmware logic.
#   make test      build + run unit tests (with AddressSanitizer/UBSan when supported)
#   make sim       build the virtual Uno used by Python tests / --port virtual
#   make compile   cross-compile for the Uno with arduino-cli (needs arduino:avr core)
CXX ?= c++
BUILD ?= build
CXXFLAGS ?= -std=c++11 -Wall -Wextra -Werror -O1
SANITIZE ?= -fsanitize=address,undefined
SRC := $(wildcard src/*.cpp)
HDR := $(wildcard src/*.h) test/fake_hal.h
FQBN ?= arduino:avr:uno

.PHONY: test sim compile clean

$(BUILD)/test_main: $(SRC) $(HDR) test/test_main.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) $(SANITIZE) -o $@ $(SRC) test/test_main.cpp

$(BUILD)/fw_sim: $(SRC) $(HDR) test/fw_sim.cpp
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) test/fw_sim.cpp

test: $(BUILD)/test_main
	./$(BUILD)/test_main

sim: $(BUILD)/fw_sim

compile:
	arduino-cli compile --fqbn $(FQBN) --warnings all .

clean:
	rm -rf $(BUILD)
