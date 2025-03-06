`timescale 1ns / 1ps

module riscv_tb;

  reg clk;
  reg reset;
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
   
    for (i = 0; i < 64; i = i + 1) begin
      uut.memory.mem[i] = 32'b0;
    end

   
    $readmemb("programs/program.bin", uut.memory.mem, 0, 63);

   
    // for (i = 0; i < 16; i = i + 1) begin
    //   $display("Instruction Memory[%0d] = %b", i, uut.memory.mem[i]);
    // end
  end

 
  initial begin
    clk = 0;
    reset = 1;
    #20 reset = 0;

    repeat (100) begin
      @(posedge clk);

      if (state == uut.FETCH)
        $display("\n Fetching instruction at PC=%h", PC);
      
      $display("Time=%10t, State=%d, PC=%h, IR=%b", $time, state, PC, IR);


      if (state == uut.WRITEBACK)
        $display("writeback Registers: x1=%h, x2=%h, x3=%h", reg_x1, reg_x2, reg_x3);
    end

   
    $display("\nMemory Dump After Execution:");
    for (i = 8000; i < 8016; i = i + 1) begin
      $display("Mem[%2d] = %b", i, uut.memory.mem[i]);
    end

    $finish;
  end

 
  initial begin
    $dumpfile("riscv_tb.vcd");
    $dumpvars(0, riscv_tb);
  end

endmodule
