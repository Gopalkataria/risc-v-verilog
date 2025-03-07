module EX_MEM(
    input clk, reset,
    input [31 : 0] alu_result_in, reg2_in,
    input [4 : 0] rd_in,
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in,
    output reg [31 : 0] alu_result_out, reg2_out,
    output reg [4 : 0] rd_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out);

module ID_EX(
    input clk, reset,
    input [31 : 0] pc_in, reg1_in, reg2_in, imm_in,
    input [4 : 0] rs1_in, rs2_in, rd_in,
    input [3 : 0] alu_ctrl_in,
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in,
    output reg [31 : 0] pc_out, reg1_out, reg2_out, imm_out,
    output reg [4 : 0] rs1_out, rs2_out, rd_out,
    output reg [3 : 0] alu_ctrl_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out);

module IF_ID(
    input clk, reset,
    input [31 : 0] pc_in, instr_in,
    output reg [31 : 0] pc_out, instr_out);

module MEM_WB(
    input clk, reset,
    input [31 : 0] mem_data_in, alu_result_in,
    input [4 : 0] rd_in,
    input reg_write_in, mem_to_reg_in,
    output reg [31 : 0] mem_data_out, alu_result_out,
    output reg [4 : 0] rd_out,
    output reg reg_write_out, mem_to_reg_out);

module alu_64bit(
    input [2 : 0] funct3,
    input [6 : 0] funct7,
    input [63 : 0] a,
    input [63 : 0] b,
    output reg [63 : 0] result);

module ControlUnit(
    input [6 : 0] opcode,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg ALUSrc,
    output reg Branch,
    output reg MemtoReg,
    output reg Jump,
    output reg AUIPC);

module Decode(
    input clk,
    input rst,
    input [31 : 0] instr,
    input [31 : 0] reg_write_data,
    input RegWrite,
    output [31 : 0] read_data1,
    output [31 : 0] read_data2,
    output [31 : 0] imm,
    output MemRead,
    output MemWrite,
    output ALUSrc,
    output Branch,
    output MemtoReg,
    output Jump,
    output AUIPC,
    output [2 : 0] funct3);

module Execute(
    input [31 : 0] PC,
    input [31 : 0] read_data1,
    input [31 : 0] read_data2,
    input [31 : 0] imm,
    input ALUSrc,
    input Branch,
    input [2 : 0] funct3,
    input [6 : 0] funct7,
    output [31 : 0] ALU_result,
    output branch_taken,
    output [31 : 0] branch_target);

module Fetch(
    input clk,
    input reset,
    input PCWrite,
    input [1 : 0] PCSrc,
    input [31 : 0] branch_target,
    input [31 : 0] alu_result,
    output reg [31 : 0] next_PC,
    input [31 : 0] current_PC);

module ImmGen(
    input [31 : 0] instr,
    output reg [31 : 0] imm);

module InstructionMemory(
    input [31 : 0] instr_addr,
    output reg [31 : 0] instr);
reg [31:0] instr_mem [0:8191];
integer i;
initial begin for (i = 0; i < 8192; i = i + 1)
    instr_mem[i] = 32'h0;
end

    always @(*) begin
    instr = instr_mem[instr_addr [13:2]];
end endmodule

    module
    DataMemory(
        input clk,
        input reset,
        input [31:0] data_addr,
        input [31:0] write_data,
        input MemRead,
        input MemWrite,
        output reg [31:0] read_data);
reg [31:0] data_mem [0:8191];
integer i;
initial begin for (i = 0; i < 8192; i = i + 1)
    data_mem[i] = 32'h0;
end

    always @(*) begin
    if (reset || !MemRead)
        read_data = 32'h0;
else read_data = data_mem[data_addr [13:2]];
end

        always @(posedge clk) begin
    if (!reset && MemWrite)
        data_mem[data_addr [13:2]] <= write_data;
end endmodule