module InstructionMemory (
    input [31:0] instr_addr,   
    output reg [31:0] instr
);
    reg [31:0] instr_mem [0:8191];  
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1)
            instr_mem[i] = 32'h0;
    end

    always @(*) begin
        instr = instr_mem[instr_addr[13:2]];
    end
endmodule
