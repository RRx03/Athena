
APP_NAME = athena
BUILD_DIR = ./build
SRC_FOLDER = ./src
OBJS_FOLDER = $(BUILD_DIR)/objs
BIN_PATH = $(BUILD_DIR)/$(APP_NAME)

CXX = clang++
CXXFLAGS = -std=c++17 -I. -I$(SRC_FOLDER) -I/opt/homebrew/include -MMD -MP -O2
LDFLAGS =

SRCS := $(shell find $(SRC_FOLDER) -name '*.cpp')
OBJS := $(patsubst $(SRC_FOLDER)/%.cpp, $(OBJS_FOLDER)/%.o, $(SRCS))
DEPS := $(OBJS:.o=.d)

.PHONY: all clean run test init bear

all: $(BIN_PATH)

$(BIN_PATH): $(OBJS)
	@echo "Linkage de l'exécutable..."
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(OBJS) $(LDFLAGS) -o $@
	@echo "Succès ! Exécutable : $@"

$(OBJS_FOLDER)/%.o: $(SRC_FOLDER)/%.cpp
	@echo "Compilation de $< ..."
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(DEPS)

init:
	brew install nlohmann-json

clean:
	rm -rf $(BUILD_DIR)
	@echo "Dossier build nettoyé."

run: all
	$(BIN_PATH)

PROBLEM ?= examples/nozzle_delaval.json
solve: all
	$(BIN_PATH) $(PROBLEM)

test: all
	$(BIN_PATH) --test

bear:
	make clean
	bear -- make

setup:
	xcode-select --install
