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
    
    // Add instructions to memory at initialization (assuming Memory has an array called mem)
    uut.memory.mem[0] = 32'h00000093; // ADDI x1, x0, 0 (x1 = 0)
    uut.memory.mem[1] = 32'h00100113; // ADDI x2, x0, 1 (x2 = 1)
    uut.memory.mem[2] = 32'h002081B3; // ADD x3, x1, x2 (x3 = x1 + x2)
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