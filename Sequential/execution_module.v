module Execute (
    input [31:0] PC,
    input [31:0] read_data1,
    input [31:0] read_data2,
    input [31:0] imm,
    input [2:0] ALUOp,
    input ALUSrc,
    input Branch,
    input [2:0] funct3,
    output [31:0] ALU_result,
    output branch_taken,
    output [31:0] branch_target
);
    wire [31:0] operand2 = ALUSrc ? imm : read_data2;
    wire zero;

    // ALU Control
    wire [3:0] alu_control;
    ALUControl alu_ctrl (
        .ALUOp(ALUOp),
        .funct3(funct3),
        .funct7(imm[31:25]),
        .ALU_control(alu_control)
    );

    // ALU
    ALU alu (
        .a(read_data1),
        .b(operand2),
        .alu_control(alu_control),
        .result(ALU_result),
        .zero(zero)
    );

    // Branch logic
    assign branch_target = PC + imm;
    assign branch_taken = Branch & (
        (funct3 == 3'b000 & zero) |  // beq
        (funct3 == 3'b001 & !zero)    // bne
        // Add other branch conditions here
    );
endmodule