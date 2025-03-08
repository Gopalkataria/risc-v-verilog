# Compiler and Simulator
VLOG = iverilog
VVP = vvp
# GTKWAVE = killall gtkwave ; gtkwave
GTKWAVE = echo 

# Directories
SRC_DIR = ./src
TEST_DIR = ./tests
OUT_DIR = .

# Source Files
SRC_FILES = $(wildcard $(SRC_DIR)/*.v)
TEST_FILES = $(wildcard $(TEST_DIR)/*.v)

# Targets
all: alu_test branch_jump_test memory_test

alu_test: $(TEST_DIR)/alu_test.v ./main.v $(SRC_FILES)
	$(VLOG) -o $(OUT_DIR)/alu_test.out $(TEST_DIR)/alu_test.v ./main.v $(SRC_FILES)
	$(VVP) $(OUT_DIR)/alu_test.out
	$(GTKWAVE) $(OUT_DIR)/alu_test.vcd &

branch_jump_test: $(TEST_DIR)/branch_Jump_test.v ./main.v $(SRC_FILES)
	$(VLOG) -o $(OUT_DIR)/branch_jump_test.out $(TEST_DIR)/branch_Jump_test.v ./main.v $(SRC_FILES)
	$(VVP) $(OUT_DIR)/branch_jump_test.out
	$(GTKWAVE) $(OUT_DIR)/branch_jump_test.vcd &

memory_test: $(TEST_DIR)/memory_test.v ./main.v $(SRC_FILES)
	$(VLOG) -o $(OUT_DIR)/memory_test.out $(TEST_DIR)/memory_test.v ./main.v $(SRC_FILES)
	$(VVP) $(OUT_DIR)/memory_test.out
	$(GTKWAVE) $(OUT_DIR)/memory_test.vcd &

stall_flush_test: $(TEST_DIR)/stall_flush_test.v ./main.v $(SRC_FILES)
	$(VLOG) -o $(OUT_DIR)/stall_flush_test.out $(TEST_DIR)/stall_flush_test.v ./main.v $(SRC_FILES)
	$(VVP) $(OUT_DIR)/stall_flush_test.out
	$(GTKWAVE) $(OUT_DIR)/stall_flush_test.vcd &

hazard_test: $(TEST_DIR)/hazard_test.v ./main.v $(SRC_FILES)
	$(VLOG) -o $(OUT_DIR)/hazard_test.out $(TEST_DIR)/hazard_test.v ./main.v $(SRC_FILES)
	$(VVP) $(OUT_DIR)/hazard_test.out
	$(GTKWAVE) $(OUT_DIR)/hazard_test.vcd &
clean:
	rm -f $(OUT_DIR)/*.out $(OUT_DIR)/*.vcd

# Help
help:
	@echo "Usage: make [target]"
	@echo "Targets:"
	@echo "  help              - Display this help message"