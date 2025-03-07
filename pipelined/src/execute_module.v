module Execute(
    input [31:0] PC,
    input [31:0] read_data1,
    input [31:0] read_data2,
    input [31:0] imm,
    input [2:0] funct3,
    input [6:0] funct7,
    input [1:0] forwardA, forwardB,
    input [31:0] alu_result_MEM, reg_write_data_WB,
    input ALUSrc,
    input Branch,
    output reg [31:0] ALU_result,
    output reg branch_taken,
    output reg [31:0] branch_target
);

    // MUX for Forwarding A (Selecting ALU Operand 1)
    reg [31:0] operand1;
    always @(*) begin
        case (forwardA)
            2'b00: operand1 = read_data1;           // From ID/EX pipeline
            2'b01: operand1 = reg_write_data_WB;   // From WB stage
            2'b10: operand1 = alu_result_MEM;      // From MEM stage
            default: operand1 = read_data1;
        endcase
    end
    
    // MUX for Forwarding B (Selecting ALU Operand 2)
    reg [31:0] operand2;
    always @(*) begin
        case (forwardB)
            2'b00: operand2 = (ALUSrc) ? imm : read_data2;
            2'b01: operand2 = reg_write_data_WB;
            2'b10: operand2 = alu_result_MEM;
            default: operand2 = read_data2;
        endcase
    end
    
    // ALU Operation
    alu_64bit alu_unit (
        .funct3(funct3),
        .funct7(funct7),
        .a(operand1),
        .b(operand2),
        .result(ALU_result)
    );
    
    // Branch Target Calculation
    always @(*) begin
        branch_target = PC + (imm << 1);  // PC-relative addressing
    end
    
    // Branch Decision
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (operand1 == operand2) && Branch;  // BEQ
            3'b001: branch_taken = (operand1 != operand2) && Branch;  // BNE
            3'b100: branch_taken = ($signed(operand1) < $signed(operand2)) && Branch; // BLT
            3'b101: branch_taken = ($signed(operand1) >= $signed(operand2)) && Branch; // BGE
            default: branch_taken = 1'b0;
        endcase
    end
    
endmodule
