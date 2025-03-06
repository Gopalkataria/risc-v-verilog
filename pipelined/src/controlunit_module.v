module ControlUnit (
    input [6:0] opcode,
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
        
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUSrc = 1'b0;
        Branch = 1'b0;
        MemtoReg = 1'b0;
        Jump = 1'b0;
        AUIPC = 1'b0;

        case (opcode)
            
            7'b0110011: begin
                RegWrite = 1'b1;
            end

            
            7'b0010011: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;  
            end

            
            7'b0000011: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                MemRead = 1'b1;
                MemtoReg = 1'b1; 
            end

            
            7'b0100011: begin
                ALUSrc = 1'b1;
                MemWrite = 1'b1;
            end

            
            7'b1100011: begin
                Branch = 1'b1;
            end

            
            7'b1101111: begin
                Jump = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            
            7'b1100111: begin
                Jump = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            
            7'b0110111: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            
            7'b0010111: begin
                AUIPC = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end
        endcase
    end
endmodule
