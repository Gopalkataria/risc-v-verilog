module Decode (
    input clk,
    input rst,  // Reset input
    input [31:0] instr,
    input [31:0] reg_write_data,
    input RegWrite,  // External control signal
    output [31:0] read_data1,
    output [31:0] read_data2,
    output [31:0] imm,
    output [2:0] ALUOp,
    output MemRead,
    output MemWrite,
    output ALUSrc,
    output Branch,
    output MemtoReg
);
    reg [31:0] reg_file [31:0];  // Register file

    // Control Unit
    wire [6:0] opcode = instr[6:0];
    wire [2:0] funct3 = instr[14:12];
    ControlUnit ctrl (
        .opcode(opcode),
        .funct3(funct3),
        .ALUOp(ALUOp),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg)
    );

    // Immediate Generator
    ImmGen imm_gen (
        .instr(instr),
        .imm(imm)
    );

    // Read registers
    assign read_data1 = reg_file[instr[19:15]];  // rs1
    assign read_data2 = reg_file[instr[24:20]];  // rs2

    // Register Write
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for (i = 0; i < 32; i = i + 1)
                reg_file[i] <= 32'b0;
        end else if (RegWrite && instr[11:7] != 5'b00000) begin
            reg_file[instr[11:7]] <= reg_write_data;  // Avoid writing to x0
        end
    end
endmodule
