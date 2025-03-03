module ALUControl (
    input [2:0] ALUOp,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] ALU_control
);
    always @(*) begin
        case (ALUOp)
            // R-type: Use funct3 and funct7
            3'b010: begin
                case (funct3)
                    3'b000: ALU_control = (funct7[5] == 1'b1 ? 4'b0010 : 4'b0000); // ADD/SUB
                    3'b001: ALU_control = 4'b0100; // SLL
                    3'b010: ALU_control = 4'b0110; // SLT
                    3'b011: ALU_control = 4'b0111; // SLTU
                    3'b100: ALU_control = 4'b1000; // XOR
                    3'b101: ALU_control = (funct7[5] == 1'b1 ? 4'b1101 : 4'b0101); // SRL/SRA
                    3'b110: ALU_control = 4'b0001; // OR
                    3'b111: ALU_control = 4'b0011; // AND
                    default: ALU_control = 4'b0000;
                endcase
            end

            // I-type: Use funct3
            3'b011: begin
                case (funct3)
                    3'b000: ALU_control = 4'b0000; // ADDI
                    3'b010: ALU_control = 4'b0110; // SLTI
                    3'b011: ALU_control = 4'b0111; // SLTIU
                    3'b100: ALU_control = 4'b1000; // XORI
                    3'b110: ALU_control = 4'b0001; // ORI
                    3'b111: ALU_control = 4'b0011; // ANDI
                    3'b001: ALU_control = 4'b0100; // SLLI
                    3'b101: ALU_control = (funct7[5] == 1'b1 ? 4'b1101 : 4'b0101); // SRLI/SRAI
                    default: ALU_control = 4'b0000;
                endcase
            end

            // Memory/Arithmetic: ADD
            3'b000: ALU_control = 4'b0000; // LW/SW

            // Branch: SUB
            3'b001: ALU_control = 4'b0010; // BEQ/BNE

            default: ALU_control = 4'b0000;
        endcase
    end
endmodule