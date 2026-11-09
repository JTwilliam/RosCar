# Run in Linux (WSL Ubuntu or Raspberry Pi), not Windows MinGW.
CXX ?= g++
AR ?= ar
PYTHON ?= /usr/bin/python3
BUILD_DIR ?= build/make
CPPFLAGS += -Iinclude
CXXFLAGS ?= -O2 -g
CXXFLAGS += -std=c++17 -Wall -Wextra -Wpedantic -Werror
DEPFLAGS = -MMD -MP
CORE_OBJS = $(BUILD_DIR)/protocol.o $(BUILD_DIR)/odometry.o $(BUILD_DIR)/serial_port.o
LIBRARY = $(BUILD_DIR)/liblearm_bridge_core.a

.PHONY: all test parity help
all: $(BUILD_DIR)/learm_replay

$(BUILD_DIR):
	mkdir -p $@

$(BUILD_DIR)/%.o: src/%.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@

$(LIBRARY): $(CORE_OBJS)
	$(AR) rcs $@ $^

$(BUILD_DIR)/learm_replay: $(BUILD_DIR)/replay.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -o $@

$(BUILD_DIR)/test_core.o: test/test_core.cpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(DEPFLAGS) -c $< -o $@

$(BUILD_DIR)/test_core: $(BUILD_DIR)/test_core.o $(LIBRARY)
	$(CXX) $(CXXFLAGS) $(LDFLAGS) $^ $(LDLIBS) -lutil -o $@

test: $(BUILD_DIR)/test_core $(BUILD_DIR)/learm_replay
	$(BUILD_DIR)/test_core
	PYTHONDONTWRITEBYTECODE=1 $(PYTHON) test/check_python_parity.py $(BUILD_DIR)/learm_replay

parity: $(BUILD_DIR)/learm_replay
	PYTHONDONTWRITEBYTECODE=1 $(PYTHON) test/check_python_parity.py $(BUILD_DIR)/learm_replay

help:
	@echo 'make / make test / make -n / make BUILD_DIR=build/debug CXXFLAGS="-O0 -g -std=c++17"'
	@echo 'ROS 2 uses CMake/colcon; src/serial_bridge_node.cpp is your exercise.'

-include $(CORE_OBJS:.o=.d) $(BUILD_DIR)/replay.d $(BUILD_DIR)/test_core.d
