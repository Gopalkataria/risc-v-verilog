`timescale 1ns / 1ps

module riscv_tb;
  
  reg clk;
  reg reset;
  
  // Instantiate the processor
  RISC_V_Multi_Cycle uut (
    .clk(clk),
    .reset(reset)
  );

  // Clock generation (10ns period)
  always #5 clk = ~clk;

  // For monitoring purposes - access processor internal signals
  wire [31:0] PC = uut.PC;
  wire [31:0] IR = uut.IR;
  wire [3:0] state = uut.state;
  
  // For debugging - monitor register values
  wire [31:0] reg_x1 = uut.RegisterFile[1];
  wire [31:0] reg_x2 = uut.RegisterFile[2];
  wire [31:0] reg_x3 = uut.RegisterFile[3];

  // Initialize the Memory module with instructions
  initial begin
    // Initialize the Memory
    // This needs to be adapted to match your actual Memory module implementation
    // Since we don't have direct access to your Memory module internals, 
    // the following is a placeholder - you may need to adjust this
    
    uut.memory.mem[0] = 32'b000000000010_00000_000_00001_0010011; // ADDI x1, x0, 2  (x1 = 2)
    uut.memory.mem[1] = 32'b000000000011_00000_000_00010_0010011; // ADDI x2, x0, 3  (x2 = 3)

    // ADD (x3 = x1 + x2)
    uut.memory.mem[2] = 32'b0000000_00010_00001_000_00011_0110011; // ADD x3, x1, x2

    // SUB (x4 = x1 - x2)
    uut.memory.mem[3] = 32'b0100000_00010_00001_000_00011_0110011; // SUB x4, x1, x2

    // AND (x5 = x1 & x2)
    uut.memory.mem[4] = 32'b0000000_00010_00001_111_00011_0110011; // AND x5, x1, x2

    // OR (x6 = x1 | x2)
    uut.memory.mem[5] = 32'b0000000_00010_00001_110_00011_0110011; // OR x6, x1, x2

    // XOR (x7 = x1 ^ x2)
    uut.memory.mem[6] = 32'b0000000_00010_00001_100_00011_0110011; // XOR x7, x1, x2

    // SLL (Shift Left Logical, x8 = x1 << (x2 & 0x1F))
    uut.memory.mem[7] = 32'b0000000_00010_00001_001_00011_0110011; // SLL x8, x1, x2

    // SRL (Shift Right Logical, x9 = x1 >> (x2 & 0x1F))
    uut.memory.mem[8] = 32'b0000000_00010_00001_101_00011_0110011; // SRL x9, x1, x2

    // SRA (Shift Right Arithmetic, x10 = x1 >> (x2 & 0x1F), preserving sign)
    uut.memory.mem[9] = 32'b0100000_00010_00001_101_00011_0110011; // SRA x10, x1, x2

    // SLT (Set Less Than, x11 = (x1 < x2) ? 1 : 0)
    uut.memory.mem[10] = 32'b0000000_00010_00001_010_00011_0110011; // SLT x11, x1, x2

    // SLTU (Set Less Than Unsigned, x12 = (x1 < x2) ? 1 : 0, unsigned)
    uut.memory.mem[11] = 32'b0000000_00010_00001_011_00011_0110011; // SLTU x12, x1, x2

    // ADDI (Immediate addition, x13 = x1 + 5)
    uut.memory.mem[12] = 32'b000000000101_00001_000_00011_0010011; // ADDI x13, x1, 5

    // ANDI (Immediate AND, x14 = x1 & 5)
    uut.memory.mem[13] = 32'b000000000101_00001_111_00011_0010011; // ANDI x14, x1, 5

    // ORI (Immediate OR, x15 = x1 | 5)
    uut.memory.mem[14] = 32'b000000000101_00001_110_00011_0010011; // ORI x15, x1, 5

    // XORI (Immediate XOR, x16 = x1 ^ 5)
    uut.memory.mem[15] = 32'b000000000101_00001_100_00011_0010011; // XORI x16, x1, 5

    // SLLI (Shift Left Logical Immediate, x17 = x1 << 1)
    uut.memory.mem[16] = 32'b000000000001_00001_001_00011_0010011; // SLLI x17, x1, 1

    // SRLI (Shift Right Logical Immediate, x18 = x1 >> 1)
    uut.memory.mem[17] = 32'b000000000001_00001_101_00011_0010011; // SRLI x18, x1, 1

    // SRAI (Shift Right Arithmetic Immediate, x19 = x1 >> 1, preserving sign)
    uut.memory.mem[18] = 32'b010000000001_00001_101_00011_0010011; // SRAI x19, x1, 1

    // SLTI (Set Less Than Immediate, x20 = (x1 < 5) ? 1 : 0)
    uut.memory.mem[19] = 32'b000000000101_00001_010_00011_0010011; // SLTI x20, x1, 5

    // SLTIU (Set Less Than Immediate Unsigned, x21 = (x1 < 5) ? 1 : 0, unsigned)
    uut.memory.mem[20] = 32'b000000000101_00001_011_00011_0010011; // SLTIU x21, x1, 5

    uut.memory.mem[3] = 32'h00000013; // NOP
    uut.memory.mem[4] = 32'h0000006F; // JAL x0, 0 (infinite loop)

    // Fill rest with NOPs
    for (integer i = 5; i < 16; i = i + 1) begin
      uut.memory.mem[i] = 32'h00000013; // NOP
    end
  end

  initial begin
    // Initialize signals
    clk = 0;
    reset = 1;
    
    // Apply reset for a few cycles
    #20 reset = 0;

    // Run for 100 cycles
    repeat (100) begin
      @(posedge clk); // Wait for clock edge
      
      // Display processor state
      $display("Time=%0t, State=%d, PC=%h, IR=%h", $time, state, PC, IR);
      
      if ( state == uut.FETCH ) begin
        $display("  Fetching instruction at PC=%h", PC);
      end

      // Display register values when in WRITEBACK state
      if (state == uut.WRITEBACK) begin
        $display("  Registers: x1=%h, x2=%h, x3=%h", reg_x1, reg_x2, reg_x3);
      end
    end

    $finish;
  end

  // Dump waveform for debugging
  initial begin
    $dumpfile("riscv_tb.vcd");
    $dumpvars(0, riscv_tb);
  end

endmodule