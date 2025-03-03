module Fetch (
    input clk,
    input reset,
    input PCSrc,
    input [31:0] branch_target,
    output [31:0] PC,
    output [31:0] instr
);
    reg [31:0] PC_reg;
    wire [31:0] next_PC;

    // Instruction memory (combinational read)
    reg [31:0] instr_mem [0:1023];  // Example: 4KB memory
    assign instr = instr_mem[PC_reg >> 2];  // Byte to word addressing

    // Next PC logic
    assign next_PC = PCSrc ? branch_target : PC_reg + 4;

    // Update PC on clock edge
    always @(posedge clk or posedge reset) begin
        if (reset) PC_reg <= 32'h0;
        else PC_reg <= next_PC;
    end

    assign PC = PC_reg;
endmodule