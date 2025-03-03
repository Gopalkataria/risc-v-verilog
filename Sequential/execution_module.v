module Execute (
    input [63:0] PC,
    input [63:0] read_data1,
    input [63:0] read_data2,
    input [63:0] imm,
    input [2:0] ALUOp,
    input ALUSrc,
    input Branch,
    input [2:0] funct3,
    input [6:0] funct7,  // Added explicit funct7 input
    output [63:0] ALU_result,
    output branch_taken,
    output [63:0] branch_target
);
    wire [63:0] operand2 = ALUSrc ? imm : read_data2;
    
    // Direct connection to the 64-bit ALU
    alu_64bit alu (
        .funct3(funct3),
        .funct7(funct7),
        .a(read_data1),
        .b(operand2),
        .result(ALU_result)
    );

    // Branch logic - expanded to handle all RISC-V branch conditions
    assign branch_target = PC + imm;
    assign branch_taken = Branch & (
        (funct3 == 3'b000 & (read_data1 == read_data2)) |              // beq
        (funct3 == 3'b001 & (read_data1 != read_data2)) |              // bne
        (funct3 == 3'b100 & ($signed(read_data1) < $signed(read_data2))) |  // blt
        (funct3 == 3'b101 & ($signed(read_data1) >= $signed(read_data2))) | // bge
        (funct3 == 3'b110 & (read_data1 < read_data2)) |               // bltu
        (funct3 == 3'b111 & (read_data1 >= read_data2))                // bgeu
    );
endmodule