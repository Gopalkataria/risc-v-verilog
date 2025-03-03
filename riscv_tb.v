`timescale 1ns / 1ps

module riscv_tb;
  
  reg clk;
  reg reset;
  reg [31:0] temp_mem [0:49];
  integer i;
  
  RISC_V_Multi_Cycle uut (
    .clk(clk),
    .reset(reset)
  );
  
  always #5 clk = ~clk;
  
  wire [31:0] PC = uut.PC;
  wire [31:0] IR = uut.IR;
  wire [3:0] state = uut.state;
  
  wire [31:0] reg_x1 = uut.RegisterFile[1];
  wire [31:0] reg_x2 = uut.RegisterFile[2];
  wire [31:0] reg_x3 = uut.RegisterFile[3];
  
  initial begin
    $readmemb("program.bin", temp_mem);
    for (i = 0; i < 50; i = i + 1) begin
      uut.memory.mem[i] = temp_mem[i];
    end
  end
  
  initial begin
    clk = 0;
    reset = 1;
    #20 reset = 0;
    repeat (100) begin
      @(posedge clk); 
      $display("Time=%0t, State=%d, PC=%h, IR=%h", $time, state, PC, IR);
      if (state == uut.FETCH) begin
        $display("  Fetching instruction at PC=%h", PC);
      end
      if (state == uut.WRITEBACK) begin
        $display("  Registers: x1=%h, x2=%h, x3=%h", reg_x1, reg_x2, reg_x3);
      end
      if (state == uut.BRANCH_COMPLETION) begin
        $display("  Branch Taken to PC=%h", uut.PCSrc);
      end
    
    end
    $finish;
  end
  
  initial begin
    $dumpfile("riscv_tb.vcd");
    $dumpvars(0, riscv_tb);
  end
  
endmodule