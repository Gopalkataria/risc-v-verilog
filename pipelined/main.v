module RISC_V_Pipelined_CPU (
    input clk,
    input reset
);
    // Pipeline register wires - IF/ID stage
    wire [31:0] IF_ID_PC, IF_ID_Instr;
    
    // Pipeline register wires - ID/EX stage
    wire [31:0] ID_EX_PC, ID_EX_Instr;
    wire [31:0] ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
    wire [3:0] ID_EX_ALUCtrl;
    wire ID_EX_MemRead, ID_EX_MemWrite, ID_EX_RegWrite, ID_EX_MemtoReg;
    wire ID_EX_ALUSrc, ID_EX_Branch, ID_EX_Jump, ID_EX_AUIPC;
    
    // Pipeline register wires - EX/MEM stage
    wire [31:0] EX_MEM_ALUResult, EX_MEM_RegR2;
    wire [4:0] EX_MEM_Rd;
    wire EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_MemtoReg;
    
    // Pipeline register wires - MEM/WB stage
    wire [31:0] MEM_WB_MemData, MEM_WB_ALUResult;
    wire [4:0] MEM_WB_Rd;
    wire MEM_WB_RegWrite, MEM_WB_MemtoReg;
    
    // Additional wires for data and control signals
    wire [31:0] PC_Out, Instr, NextPC;
    wire [31:0] RegR1Data, RegR2Data, ImmGenOut;
    wire [4:0] Rs1, Rs2, Rd;
    wire [31:0] ALUResult, MemReadData, RegWriteData;
    wire [31:0] BranchTarget;
    wire [2:0] Funct3;
    wire [6:0] Funct7;
    
    // Control signals
    wire MemRead, MemWrite, ALUSrc, RegWrite, MemtoReg, Branch, Jump, AUIPC;
    wire [3:0] ALUCtrl;
    
    // Hazard detection and forwarding signals
    wire PCWrite, IF_ID_Write, ID_EX_Flush;
    wire [1:0] ForwardA, ForwardB, PCSrc;
    wire BranchTaken, InstrFlush;
    
    // Instantiate Fetch stage
    Fetch fetch_stage (
        .clk(clk),
        .reset(reset),
        .PCWrite(PCWrite),
        .PCSrc(PCSrc),
        .branch_target(BranchTarget),
        .alu_result(ALUResult),
        .Instr_Flush(InstrFlush),
        .IF_ID_Write(IF_ID_Write),
        .next_PC(NextPC),
        .current_PC(PC_Out),
        .instr(Instr)
    );
    
    // IF/ID Pipeline Register
    IF_ID if_id_reg (
        .clk(clk),
        .reset(reset),
        .IF_ID_Write(IF_ID_Write),
        .Instr_Flush(InstrFlush),
        .pc_in(PC_Out),
        .instr_in(Instr),
        .pc_out(IF_ID_PC),
        .instr_out(IF_ID_Instr)
    );
    
    // Decode stage
    Decode decode_stage (
        .clk(clk),
        .rst(reset),
        .instr(IF_ID_Instr),
        .reg_write_addr(MEM_WB_Rd),
        .reg_write_data(RegWriteData),
        .RegWrite_WB(MEM_WB_RegWrite),
        .read_data1(RegR1Data),
        .read_data2(RegR2Data),
        .imm(ImmGenOut),
        .rs1(Rs1),
        .rs2(Rs2),
        .rd(Rd),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg),
        .Jump(Jump),
        .AUIPC(AUIPC),
        .alu_ctrl(ALUCtrl),
        .funct3(Funct3),
        .funct7(Funct7)
    );
    
    // ID/EX Pipeline Register
    ID_EX id_ex_reg (
        .clk(clk),
        .reset(reset),
        .ID_EX_Flush(ID_EX_Flush),
        .pc_in(IF_ID_PC),
        .instr_in(IF_ID_Instr),
        .read_data1_in(RegR1Data),
        .read_data2_in(RegR2Data),
        .imm_in(ImmGenOut),
        .rs1_in(Rs1),
        .rs2_in(Rs2),
        .rd_in(Rd),
        .alu_ctrl_in(ALUCtrl),
        .mem_read_in(MemRead),
        .mem_write_in(MemWrite),
        .reg_write_in(RegWrite),
        .mem_to_reg_in(MemtoReg),
        .alu_src_in(ALUSrc),
        .branch_in(Branch),
        .jump_in(Jump),
        .auipc_in(AUIPC),
        .pc_out(ID_EX_PC),
        .instr_out(ID_EX_Instr),
        .read_data1_out(ID_EX_RegR1),
        .read_data2_out(ID_EX_RegR2),
        .imm_out(ID_EX_Imm),
        .rs1_out(ID_EX_Rs1),
        .rs2_out(ID_EX_Rs2),
        .rd_out(ID_EX_Rd),
        .alu_ctrl_out(ID_EX_ALUCtrl),
        .mem_read_out(ID_EX_MemRead),
        .mem_write_out(ID_EX_MemWrite),
        .reg_write_out(ID_EX_RegWrite),
        .mem_to_reg_out(ID_EX_MemtoReg),
        .alu_src_out(ID_EX_ALUSrc),
        .branch_out(ID_EX_Branch),
        .jump_out(ID_EX_Jump),
        .auipc_out(ID_EX_AUIPC)
    );
    
    // Execute stage
    Execute execute_stage (
        .PC(ID_EX_PC),
        .read_data1(ID_EX_RegR1),
        .read_data2(ID_EX_RegR2),
        .imm(ID_EX_Imm),
        .funct3(ID_EX_Instr[14:12]),  // Extract funct3 from instruction
        .funct7(ID_EX_Instr[31:25]),  // Extract funct7 from instruction
        .forwardA(ForwardA),
        .forwardB(ForwardB),
        .alu_result_MEM(EX_MEM_ALUResult),
        .reg_write_data_WB(RegWriteData),
        .ALUSrc(ID_EX_ALUSrc),
        .Branch(ID_EX_Branch),
        .ALU_result(ALUResult),
        .branch_taken(BranchTaken),
        .branch_target(BranchTarget)
    );
    
    // EX/MEM Pipeline Register
    EX_MEM ex_mem_reg (
        .clk(clk),
        .reset(reset),
        .ALU_result_in(ALUResult),
        .reg2_in(ID_EX_RegR2),
        .rd_in(ID_EX_Rd),
        .mem_read_in(ID_EX_MemRead),
        .mem_write_in(ID_EX_MemWrite),
        .reg_write_in(ID_EX_RegWrite),
        .mem_to_reg_in(ID_EX_MemtoReg),
        .ALU_result_out(EX_MEM_ALUResult),
        .reg2_out(EX_MEM_RegR2),
        .rd_out(EX_MEM_Rd),
        .mem_read_out(EX_MEM_MemRead),
        .mem_write_out(EX_MEM_MemWrite),
        .reg_write_out(EX_MEM_RegWrite),
        .mem_to_reg_out(EX_MEM_MemtoReg)
    );
    
    // Data Memory (Memory stage)
    DataMemory data_memory (
        .clk(clk),
        .reset(reset),
        .data_addr(EX_MEM_ALUResult),
        .write_data(EX_MEM_RegR2),
        .MemRead(EX_MEM_MemRead),
        .MemWrite(EX_MEM_MemWrite),
        .read_data(MemReadData)
    );
    
    // MEM/WB Pipeline Register
    MEM_WB mem_wb_reg (
        .clk(clk),
        .reset(reset),
        .mem_data_in(MemReadData),
        .alu_result_in(EX_MEM_ALUResult),
        .rd_in(EX_MEM_Rd),
        .reg_write_in(EX_MEM_RegWrite),
        .mem_to_reg_in(EX_MEM_MemtoReg),
        .mem_data_out(MEM_WB_MemData),
        .alu_result_out(MEM_WB_ALUResult),
        .rd_out(MEM_WB_Rd),
        .reg_write_out(MEM_WB_RegWrite),
        .mem_to_reg_out(MEM_WB_MemtoReg)
    );
    
    // Write Back stage
    WriteBack writeback_stage (
        .ALU_result(MEM_WB_ALUResult),
        .mem_read_data(MEM_WB_MemData),
        .MemtoReg(MEM_WB_MemtoReg),
        .reg_write_data(RegWriteData)
    );
    
    // Hazard Detection Unit - Adding a new module for hazard detection
    HazardDetectionUnit hazard_detection_unit (
        .ID_EX_MemRead(ID_EX_MemRead),
        .ID_EX_Rd(ID_EX_Rd),
        .IF_ID_Rs1(Rs1),
        .IF_ID_Rs2(Rs2),
        .Branch_Taken(BranchTaken),
        .Jump(ID_EX_Jump),
        .PCWrite(PCWrite),
        .IF_ID_Write(IF_ID_Write),
        .ID_EX_Flush(ID_EX_Flush),
        .InstrFlush(InstrFlush)
    );
    
    // Forwarding Unit - Adding a new module for data forwarding
    ForwardingUnit forwarding_unit (
        .ID_EX_Rs1(ID_EX_Rs1),
        .ID_EX_Rs2(ID_EX_Rs2),
        .EX_MEM_Rd(EX_MEM_Rd),
        .MEM_WB_Rd(MEM_WB_Rd),
        .EX_MEM_RegWrite(EX_MEM_RegWrite),
        .MEM_WB_RegWrite(MEM_WB_RegWrite),
        .ForwardA(ForwardA),
        .ForwardB(ForwardB)
    );
    
    // Branch Prediction Logic
    assign PCSrc = (BranchTaken || ID_EX_Jump) ? 2'b01 : 2'b00;

endmodule

// Hazard Detection Unit - New module
module HazardDetectionUnit (
    input ID_EX_MemRead,
    input [4:0] ID_EX_Rd,
    input [4:0] IF_ID_Rs1,
    input [4:0] IF_ID_Rs2,
    input Branch_Taken,
    input Jump,
    output reg PCWrite,
    output reg IF_ID_Write,
    output reg ID_EX_Flush,
    output reg InstrFlush
);
    
    always @(*) begin
        // Default values
        PCWrite = 1'b1;
        IF_ID_Write = 1'b1;
        ID_EX_Flush = 1'b0;
        InstrFlush = 1'b0;
        
        // Load-use hazard detection (stall pipeline)
        if (ID_EX_MemRead && 
            ((ID_EX_Rd == IF_ID_Rs1) || (ID_EX_Rd == IF_ID_Rs2)) &&
            (ID_EX_Rd != 0)) begin
            PCWrite = 1'b0;     // Don't update PC
            IF_ID_Write = 1'b0; // Don't update IF/ID register
            ID_EX_Flush = 1'b1; // Insert bubble in EX stage
        end
        
        // Branch/Jump hazard (flush pipeline)
        if (Branch_Taken || Jump) begin
            InstrFlush = 1'b1;  // Flush the instruction in IF stage
            ID_EX_Flush = 1'b1; // Insert bubble in ID/EX register
        end
    end
endmodule

// Forwarding Unit - New module
module ForwardingUnit (
    input [4:0] ID_EX_Rs1,
    input [4:0] ID_EX_Rs2,
    input [4:0] EX_MEM_Rd,
    input [4:0] MEM_WB_Rd,
    input EX_MEM_RegWrite,
    input MEM_WB_RegWrite,
    output reg [1:0] ForwardA,
    output reg [1:0] ForwardB
);
    
    always @(*) begin
        // Forward A logic (for first ALU operand)
        if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs1))
            ForwardA = 2'b10; // Forward from MEM stage
        else if (MEM_WB_RegWrite && (MEM_WB_Rd != 0) && (MEM_WB_Rd == ID_EX_Rs1))
            ForwardA = 2'b01; // Forward from WB stage
        else
            ForwardA = 2'b00; // No forwarding
        
        // Forward B logic (for second ALU operand)
        if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs2))
            ForwardB = 2'b10; // Forward from MEM stage
        else if (MEM_WB_RegWrite && (MEM_WB_Rd != 0) && (MEM_WB_Rd == ID_EX_Rs2))
            ForwardB = 2'b01; // Forward from WB stage
        else
            ForwardB = 2'b00; // No forwarding
    end
endmodule
