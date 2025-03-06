`include "alu_module.v" 
module Execute (
    input [31:0] PC,
    input [31:0] read_data1,
    input [31:0] read_data2,
    input [31:0] imm,
    input ALUSrc,
    input Branch,
    input [2:0] funct3,
    input [6:0] funct7,         
    output [31:0] ALU_result,
    output branch_taken,
    output [31:0] branch_target
);
    
    wire [63:0] read_data1_64 = {{32{read_data1[31]}}, read_data1};
    wire [63:0] read_data2_64 = {{32{read_data2[31]}}, read_data2};
    wire [63:0] imm_64 = {{32{imm[31]}}, imm};
    wire [63:0] operand2_64 = ALUSrc ? imm_64 : read_data2_64;
    
    wire [63:0] alu_result_64;
    
    
    alu_64bit alu (
        .funct3(funct3),
        .funct7(funct7),
        .a(read_data1_64),
        .b(operand2_64),
        .result(alu_result_64)
    );
    
    
    assign ALU_result = alu_result_64[31:0];

    
    assign branch_target = PC + imm;
    assign branch_taken = Branch & (
        (funct3 == 3'b000 & (read_data1 == read_data2)) |                
        (funct3 == 3'b001 & (read_data1 != read_data2)) |                
        (funct3 == 3'b100 & ($signed(read_data1) < $signed(read_data2))) |  
        (funct3 == 3'b101 & ($signed(read_data1) >= $signed(read_data2))) | 
        (funct3 == 3'b110 & (read_data1 < read_data2)) |                 
        (funct3 == 3'b111 & (read_data1 >= read_data2))                  
    );
endmodule
