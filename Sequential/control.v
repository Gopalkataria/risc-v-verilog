module ControlUnit (
    input [6:0] opcode,
    output reg [2:0] ALUOp,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg ALUSrc,
    output reg Branch,
    output reg MemtoReg,
    output reg Jump,
    output reg AUIPC
);
    always @(*) begin
        // Default values
        ALUOp = 3'b000;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUSrc = 1'b0;
        Branch = 1'b0;
        MemtoReg = 1'b0;
        Jump = 1'b0;
        AUIPC = 1'b0;

        case (opcode)
            // R-type (ADD, SUB, AND, OR, etc.)
            7'b0110011: begin
                ALUOp = 3'b010;  // R-type ALUOp
                RegWrite = 1'b1;
            end

            // I-type (ADDI, ORI, ANDI, etc.)
            7'b0010011: begin
                ALUOp = 3'b011;  // I-type ALUOp
                RegWrite = 1'b1;
                ALUSrc = 1'b1;  // Use immediate
            end

            // Load (LW)
            7'b0000011: begin
                ALUOp = 3'b000;  // ADD for address
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                MemRead = 1'b1;
                MemtoReg = 1'b1; // Write data from memory
            end

            // Store (SW)
            7'b0100011: begin
                ALUOp = 3'b000;  // ADD for address
                ALUSrc = 1'b1;
                MemWrite = 1'b1;
            end

            // Branch (BEQ, BNE, etc.)
            7'b1100011: begin
                ALUOp = 3'b001;  // SUB for comparison
                Branch = 1'b1;
            end

            // Jump (JAL)
            7'b1101111: begin
                Jump = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            // JALR (Jump and Link Register)
            7'b1100111: begin
                Jump = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            // LUI (Load Upper Immediate)
            7'b0110111: begin
                ALUOp = 3'b100;  // LUI operation
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            // AUIPC (Add Upper Immediate to PC)
            7'b0010111: begin
                AUIPC = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end
        endcase
    end
endmodule
