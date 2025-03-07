
module Decode (
    input clk,
    input rst,                 
    input [31:0] instr,
    input [31:0] reg_write_data,
    input RegWrite,            
    output [31:0] read_data1,
    output [31:0] read_data2,
    output [31:0] imm,
    output MemRead,
    output MemWrite,
    output ALUSrc,
    output Branch,
    output MemtoReg,
    output Jump,               
    output AUIPC,              
    output [2:0] funct3        
);
    reg [31:0] reg_file [31:0];  
    integer i;                   
    
    
    wire [6:0] opcode = instr[6:0];
    
    ControlUnit ctrl (
        .opcode(opcode),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg),
        .Jump(Jump),
        .AUIPC(AUIPC)
    );
endmodule