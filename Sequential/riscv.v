module RISC_V_Single_Cycle (
    input clk,
    input reset
);
    // Fetch stage signals
    wire [31:0] PC, instr;
    wire PCSrc = execute.branch_taken;
    wire [31:0] branch_target = execute.branch_target;

    // Decode stage signals
    wire [31:0] read_data1, read_data2, imm;
    wire [2:0] ALUOp;
    wire RegWrite, MemRead, MemWrite, ALUSrc, Branch, MemtoReg;

    // Execute stage signals
    wire [31:0] ALU_result;
    wire branch_taken;

    // Memory stage signals
    wire [31:0] mem_read_data;

    // Writeback stage signals
    wire [31:0] reg_write_data;

    Fetch fetch (
        .clk(clk),
        .reset(reset),
        .PCSrc(branch_taken),
        .branch_target(branch_target),
        .PC(PC),
        .instr(instr)
    );

    Decode decode (
        .clk(clk),
        .instr(instr),
        .reg_write_data(reg_write_data),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .imm(imm),
        .ALUOp(ALUOp),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg)
    );

    Execute execute (
        .PC(PC),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .imm(imm),
        .ALUOp(ALUOp),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .funct3(instr[14:12]),
        .ALU_result(ALU_result),
        .branch_taken(branch_taken),
        .branch_target(branch_target)
    );

    Memory memory (
        .clk(clk),
        .address(ALU_result),
        .write_data(read_data2),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .read_data(mem_read_data)
    );

    WriteBack writeback (
        .ALU_result(ALU_result),
        .mem_read_data(mem_read_data),
        .MemtoReg(MemtoReg),
        .reg_write_data(reg_write_data)
    );
endmodule