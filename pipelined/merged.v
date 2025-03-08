module full_adder(
    input a,
    input b,
    input cin,
    output sum,
    output cout
);
    wire w1, w2, w3;
    
    xor(w1, a, b);
    xor(sum, w1, cin);
    and(w2, w1, cin);
    and(w3, a, b);
    or(cout, w2, w3);
endmodule


module adder_64bit(
    input [63:0] a,
    input [63:0] b,
    input cin,
    output [63:0] sum,
    output cout
);
    wire [63:0] carry;
    
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : adder_loop
            if(i == 0)
                full_adder fa(
                    .a(a[0]),
                    .b(b[0]),
                    .cin(cin),
                    .sum(sum[0]),
                    .cout(carry[0])
                );
            else
                full_adder fa(
                    .a(a[i]),
                    .b(b[i]),
                    .cin(carry[i-1]),
                    .sum(sum[i]),
                    .cout(carry[i])
                );
        end
    endgenerate
    
    assign cout = carry[63];
endmodule


module twos_complement_64bit(
    input [63:0] in,
    output [63:0] out
);
    wire [63:0] not_in;
    wire dummy_cout;
    
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : complement_loop
            not(not_in[i], in[i]);
        end
    endgenerate
    
    adder_64bit add_one(
        .a(not_in),
        .b(64'b1),
        .cin(1'b0),
        .sum(out),
        .cout(dummy_cout)
    );
endmodule


module subtractor_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] diff
);
    wire [63:0] b_complement;
    wire dummy_cout;
    
    twos_complement_64bit comp(
        .in(b),
        .out(b_complement)
    );
    
    adder_64bit sub(
        .a(a),
        .b(b_complement),
        .cin(1'b0),
        .sum(diff),
        .cout(dummy_cout)
    );
endmodule


module and_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] result
);
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : and_loop
            and(result[i], a[i], b[i]);
        end
    endgenerate
endmodule


module or_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] result
);
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : or_loop
            or(result[i], a[i], b[i]);
        end
    endgenerate
endmodule


module xor_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] result
);
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : xor_loop
            xor(result[i], a[i], b[i]);
        end
    endgenerate
endmodule


module sll_64bit(
    input [63:0] a,
    input [5:0] shift_amt,
    output [63:0] result
);
    wire [63:0] shift_stage [5:0];
    
    assign shift_stage[0] = shift_amt[0] ? {a[62:0], 1'b0} : a;
    
    genvar i;
    generate
        for(i = 1; i < 6; i = i + 1) begin : shift_loop
            assign shift_stage[i] = shift_amt[i] ? 
                {shift_stage[i-1][63-(2**i):0], {(2**i){1'b0}}} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule


module srl_64bit(
    input [63:0] a,
    input [5:0] shift_amt,
    output [63:0] result
);
    wire [63:0] shift_stage [5:0];
    
    assign shift_stage[0] = shift_amt[0] ? {1'b0, a[63:1]} : a;
    
    genvar i;
    generate
        for(i = 1; i < 6; i = i + 1) begin : shift_loop
            assign shift_stage[i] = shift_amt[i] ? 
                {{(2**i){1'b0}}, shift_stage[i-1][63:2**i]} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule


module sra_64bit(
    input [63:0] a,
    input [5:0] shift_amt,
    output [63:0] result
);
    wire [63:0] shift_stage [5:0];
    wire sign_bit;
    
    assign sign_bit = a[63];
    
    assign shift_stage[0] = shift_amt[0] ? {sign_bit, a[63:1]} : a;
    
    genvar i;
    generate
        for(i = 1; i < 6; i = i + 1) begin : shift_loop
            assign shift_stage[i] = shift_amt[i] ? 
                {{(2**i){sign_bit}}, shift_stage[i-1][63:2**i]} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule


module alu_64bit(
    input [2:0] funct3,     
    input [6:0] funct7,     
    input [63:0] a,         
    input [63:0] b,         
    output reg [63:0] result 
);
    wire [63:0] add_result;
    wire [63:0] sub_result;
    wire [63:0] and_result;
    wire [63:0] or_result;
    wire [63:0] xor_result;
    wire [63:0] sll_result;
    wire [63:0] srl_result;
    wire [63:0] sra_result;
    wire [63:0] slt_result;
    wire [63:0] sltu_result;

    adder_64bit add_op(
        .a(a),
        .b(b),
        .cin(1'b0),
        .sum(add_result)
    );

    subtractor_64bit sub_op(
        .a(a),
        .b(b),
        .diff(sub_result)
    );

    and_64bit and_op(
        .a(a),
        .b(b),
        .result(and_result)
    );

    or_64bit or_op(
        .a(a),
        .b(b),
        .result(or_result)
    );

    xor_64bit xor_op(
        .a(a),
        .b(b),
        .result(xor_result)
    );

    sll_64bit sll_op(
        .a(a),
        .shift_amt(b[5:0]),
        .result(sll_result)
    );

    srl_64bit srl_op(
        .a(a),
        .shift_amt(b[5:0]),
        .result(srl_result)
    );

    sra_64bit sra_op(
        .a(a),
        .shift_amt(b[5:0]),
        .result(sra_result)
    );

    assign slt_result = {63'b0, $signed(a) < $signed(b)};
    assign sltu_result = {63'b0, a < b};

    always @(*) begin
        case(funct3)
            3'b000: begin  
                if (funct7[5]) 
                    result = sub_result;
                else           
                    result = add_result;
            end
            3'b001: result = sll_result;   
            3'b010: result = slt_result;   
            3'b011: result = sltu_result;  
            3'b100: result = xor_result;   
            3'b101: begin  
                if (funct7[5]) 
                    result = sra_result;
                else           
                    result = srl_result;
            end
            3'b110: result = or_result;    
            3'b111: result = and_result;   
            default: result = 64'b0;
        endcase
    end
endmodule

module DataMemory (
    input clk,
    input reset,
    input [31:0] data_addr,    
    input [31:0] write_data,   
    input MemRead,             
    input MemWrite,            
    output reg [31:0] read_data 
);
    reg [31:0] data_mem [0:8191];  
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1)
            data_mem[i] = 32'h0;
    end

    always @(*) begin
        if (reset || !MemRead)
            read_data = 32'h0;
        else
            read_data = data_mem[data_addr[13:2]];
    end

    always @(posedge clk) begin
        if (!reset && MemWrite)
            data_mem[data_addr[13:2]] <= write_data;
    end
endmodule
module Decode(
    input clk,
    input rst,
    input [31:0] instr,
    input [4:0] reg_write_addr,
    input [31:0] reg_write_data,
    input RegWrite_WB,
    output [31:0] read_data1,
    output [31:0] read_data2,
    output [31:0] imm,
    output [4:0] rs1, rs2, rd,
    output MemRead,
    output MemWrite,
    output ALUSrc,
    output Branch,
    output MemtoReg,
    output Jump,
    output AUIPC,
    output reg [3:0] alu_ctrl,
    output [2:0] funct3,
    output [6:0] funct7
);

    assign rs1 = instr[19:15];
    assign rs2 = instr[24:20];
    assign rd = instr[11:7];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    
    RegisterFile reg_file (
        .clk(clk),
        .rst(rst),
        .RegWrite(RegWrite_WB),
        .rs1(rs1),
        .rs2(rs2),
        .rd(reg_write_addr),
        .write_data(reg_write_data),
        .read_data1(read_data1),
        .read_data2(read_data2)
    );

    
    ImmGen imm_gen(
        .instr(instr),
        .imm(imm)
    );

    

    
    always @(*) begin
        case (instr[6:0])
            7'b0110011: alu_ctrl = {instr[30], instr[14:12]}; 
            7'b0010011: alu_ctrl = {1'b0, instr[14:12]};      
            default:    alu_ctrl = 4'b0000;
        endcase
    end

endmodule
module EX_MEM(
    input clk, reset,
    input [31:0] ALU_result_in, reg2_in,
    input [4:0] rd_in,
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in,
    output reg [31:0] ALU_result_out, reg2_out,
    output reg [4:0] rd_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ALU_result_out <= 0;
            reg2_out <= 0;
            rd_out <= 0;
            mem_read_out <= 0;
            mem_write_out <= 0;
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
        end else begin
            ALU_result_out <= ALU_result_in;
            reg2_out <= reg2_in;
            rd_out <= rd_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule

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
    output  [31:0] ALU_result,
    output reg branch_taken,
    output reg [31:0] branch_target
);

    
    reg [31:0] operand1;
    always @(*) begin
        case (forwardA)
            2'b00: operand1 = read_data1;           
            2'b01: operand1 = reg_write_data_WB;   
            2'b10: operand1 = alu_result_MEM;      
            default: operand1 = read_data1;
        endcase
    end
    
    
    reg [31:0] operand2;
    always @(*) begin
        case (forwardB)
            2'b00: operand2 = (ALUSrc) ? imm : read_data2;
            2'b01: operand2 = reg_write_data_WB;
            2'b10: operand2 = alu_result_MEM;
            default: operand2 = read_data2;
        endcase
    end

    wire [63:0] alu_result_wire;  
    
     alu_64bit alu_unit (
        .funct3(funct3),
        .funct7(funct7),
        .a({{32{operand1[31]}}, operand1}),  
        .b({{32{operand2[31]}}, operand2}),  
        .result(alu_result_wire)
    );

    
    assign ALU_result = alu_result_wire[31:0];
    
    
    always @(*) begin
        branch_target = PC + (imm << 1);  
    end
    
    
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (operand1 == operand2) && Branch;  
            3'b001: branch_taken = (operand1 != operand2) && Branch;  
            3'b100: branch_taken = ($signed(operand1) < $signed(operand2)) && Branch; 
            3'b101: branch_taken = ($signed(operand1) >= $signed(operand2)) && Branch; 
            default: branch_taken = 1'b0;
        endcase
    end
    
endmodule
module Fetch(
    input clk,
    input reset,
    input PCWrite, 
    input [1:0] PCSrc, 
    input [31:0] branch_target, 
    input [31:0] alu_result, 
    input Instr_Flush, 
    input IF_ID_Write, 
    output reg [31:0] next_PC, 
    output reg [31:0] current_PC, 
    output [31:0] instr 
);

    wire [31:0] instr_mem_out;
    
    
    InstructionMemory instr_mem(
        .instr_addr(current_PC),
        .instr(instr_mem_out)
    );
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_PC <= 32'h0;
        end else if (PCWrite) begin
            case (PCSrc)
                2'b00: next_PC = current_PC + 4; 
                2'b01: next_PC = branch_target; 
                2'b10: next_PC = alu_result; 
                default: next_PC = current_PC + 4;
            endcase
            current_PC <= next_PC;
        end
    end
    
    assign instr = Instr_Flush ? 32'h00000013 : instr_mem_out; 

endmodule

module ID_EX(
    input clk,
    input reset,
    input ID_EX_Flush, 
    input [31:0] pc_in, 
    input [31:0] instr_in, 
    input [31:0] read_data1_in, 
    input [31:0] read_data2_in, 
    input [31:0] imm_in, 
    input [4:0] rs1_in, rs2_in, rd_in, 
    input [3:0] alu_ctrl_in, 
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in, alu_src_in, branch_in, jump_in, auipc_in, 
    output reg [31:0] pc_out,
    output reg [31:0] instr_out,
    output reg [31:0] read_data1_out,
    output reg [31:0] read_data2_out,
    output reg [31:0] imm_out,
    output reg [4:0] rs1_out, rs2_out, rd_out,
    output reg [3:0] alu_ctrl_out,
    output reg mem_read_out, mem_write_out, reg_write_out, mem_to_reg_out, alu_src_out, branch_out, jump_out, auipc_out
);

    always @(posedge clk or posedge reset) begin
        if (reset || ID_EX_Flush) begin
            
            pc_out <= 32'b0;
            instr_out <= 32'b0;
            read_data1_out <= 32'b0;
            read_data2_out <= 32'b0;
            imm_out <= 32'b0;
            rs1_out <= 5'b0;
            rs2_out <= 5'b0;
            rd_out <= 5'b0;
            alu_ctrl_out <= 4'b0;
            mem_read_out <= 0;
            mem_write_out <= 0;
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
            alu_src_out <= 0;
            branch_out <= 0;
            jump_out <= 0;
            auipc_out <= 0;
        end
        else begin
            
            pc_out <= pc_in;
            instr_out <= instr_in;
            read_data1_out <= read_data1_in;
            read_data2_out <= read_data2_in;
            imm_out <= imm_in;
            rs1_out <= rs1_in;
            rs2_out <= rs2_in;
            rd_out <= rd_in;
            alu_ctrl_out <= alu_ctrl_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
            alu_src_out <= alu_src_in;
            branch_out <= branch_in;
            jump_out <= jump_in;
            auipc_out <= auipc_in;
        end
    end

endmodule
module IF_ID(
    input clk,
    input reset,
    input IF_ID_Write,  
    input Instr_Flush,  
    input [31:0] pc_in, instr_in,
    output reg [31:0] pc_out, instr_out
);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            pc_out <= 32'h0;
            instr_out <= 32'h0;
        end 
        else if (Instr_Flush) begin
            pc_out <= 32'h0;
            instr_out <= 32'h00000013; 
        end
        else if (IF_ID_Write) begin
            pc_out <= pc_in;
            instr_out <= instr_in;
        end
    end

endmodule
module ImmGen (
    input [31:0] instr,
    output reg [31:0] imm
);
    always @(*) begin
        case (instr[6:0])  
            
            7'b0010011, 7'b0000011: 
                imm = {{20{instr[31]}}, instr[31:20]};  

            
            7'b0100011: 
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};

            
            7'b1100011: 
                imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};

            
            7'b1101111: 
                imm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

            
            7'b0110111, 7'b0010111: 
                imm = {instr[31:12], 12'b0};

            default: imm = 32'b0;
        endcase
    end
endmodule
module InstructionMemory (
    input [31:0] instr_addr,   
    output reg [31:0] instr
);
    reg [31:0] instr_mem [0:8191];  
    integer i;
    initial begin
        for (i = 0; i < 8192; i = i + 1)
            instr_mem[i] = 32'h0;
    end

    always @(*) begin
        instr = instr_mem[instr_addr[13:2]];
    end
endmodule
module MEM_WB (
    input clk, reset,
    input [31:0] mem_data_in, alu_result_in,
    input [4:0] rd_in,
    input reg_write_in, mem_to_reg_in,
    output reg [31:0] mem_data_out, alu_result_out,
    output reg [4:0] rd_out,
    output reg reg_write_out, mem_to_reg_out
);
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            mem_data_out <= 0;
            alu_result_out <= 0;
            rd_out <= 0;
            reg_write_out <= 0;
            mem_to_reg_out <= 0;
        end else begin
            mem_data_out <= mem_data_in;
            alu_result_out <= alu_result_in;
            rd_out <= rd_in;
            reg_write_out <= reg_write_in;
            mem_to_reg_out <= mem_to_reg_in;
        end
    end
endmodule
module RegisterFile(
    input clk,
    input rst,
    input RegWrite,
    input [4:0] rs1, rs2, rd,
    input [31:0] write_data,
    output [31:0] read_data1,
    output [31:0] read_data2
);
    reg [31:0] registers [0:31];

    
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'h0;
    end

    
    assign read_data1 = (rs1 != 0) ? registers[rs1] : 32'h0;
    assign read_data2 = (rs2 != 0) ? registers[rs2] : 32'h0;

    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                registers[i] <= 32'h0;
        end
        else if (RegWrite && rd != 0) begin
            registers[rd] <= write_data;
        end
    end
endmodule
module WriteBack (
    input [31:0] ALU_result,
    input [31:0] mem_read_data,
    input MemtoReg,
    output [31:0] reg_write_data
);
    assign reg_write_data = MemtoReg ? mem_read_data : ALU_result;
endmodule

module RISC_V_Pipelined_CPU (
    input clk,
    input reset
);
    
    wire [31:0] IF_ID_PC, IF_ID_Instr;
    
    
    wire [31:0] ID_EX_PC, ID_EX_Instr;
    wire [31:0] ID_EX_RegR1, ID_EX_RegR2, ID_EX_Imm;
    wire [4:0] ID_EX_Rs1, ID_EX_Rs2, ID_EX_Rd;
    wire [3:0] ID_EX_ALUCtrl;
    wire ID_EX_MemRead, ID_EX_MemWrite, ID_EX_RegWrite, ID_EX_MemtoReg;
    wire ID_EX_ALUSrc, ID_EX_Branch, ID_EX_Jump, ID_EX_AUIPC;
    
    
    wire [31:0] EX_MEM_ALUResult, EX_MEM_RegR2;
    wire [4:0] EX_MEM_Rd;
    wire EX_MEM_MemRead, EX_MEM_MemWrite, EX_MEM_RegWrite, EX_MEM_MemtoReg;
    
    
    wire [31:0] MEM_WB_MemData, MEM_WB_ALUResult;
    wire [4:0] MEM_WB_Rd;
    wire MEM_WB_RegWrite, MEM_WB_MemtoReg;
    
    
    wire [31:0] PC_Out, Instr, NextPC;
    wire [31:0] RegR1Data, RegR2Data, ImmGenOut;
    wire [4:0] Rs1, Rs2, Rd;
    wire [31:0] ALUResult, MemReadData, RegWriteData;
    wire [31:0] BranchTarget;
    wire [2:0] Funct3;
    wire [6:0] Funct7;
    
    
    wire MemRead, MemWrite, ALUSrc, RegWrite, MemtoReg, Branch, Jump, AUIPC;
    wire [3:0] ALUCtrl;
    
    
    wire PCWrite, IF_ID_Write, ID_EX_Flush;
    wire [1:0] ForwardA, ForwardB, PCSrc;
    wire BranchTaken, InstrFlush;
    
    
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
    
    
    Execute execute_stage (
        .PC(ID_EX_PC),
        .read_data1(ID_EX_RegR1),
        .read_data2(ID_EX_RegR2),
        .imm(ID_EX_Imm),
        .funct3(ID_EX_Instr[14:12]),  
        .funct7(ID_EX_Instr[31:25]),  
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
    
    
    DataMemory data_memory (
        .clk(clk),
        .reset(reset),
        .data_addr(EX_MEM_ALUResult),
        .write_data(EX_MEM_RegR2),
        .MemRead(EX_MEM_MemRead),
        .MemWrite(EX_MEM_MemWrite),
        .read_data(MemReadData)
    );
    
    
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
    
    
    WriteBack writeback_stage (
        .ALU_result(MEM_WB_ALUResult),
        .mem_read_data(MEM_WB_MemData),
        .MemtoReg(MEM_WB_MemtoReg),
        .reg_write_data(RegWriteData)
    );
    
    
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
    
    
    assign PCSrc = (BranchTaken || ID_EX_Jump) ? 2'b01 : 2'b00;

endmodule


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
        
        PCWrite = 1'b1;
        IF_ID_Write = 1'b1;
        ID_EX_Flush = 1'b0;
        InstrFlush = 1'b0;
        
        
        if (ID_EX_MemRead && 
            ((ID_EX_Rd == IF_ID_Rs1) || (ID_EX_Rd == IF_ID_Rs2)) &&
            (ID_EX_Rd != 0)) begin
            PCWrite = 1'b0;     
            IF_ID_Write = 1'b0; 
            ID_EX_Flush = 1'b1; 
        end
        
        
        if (Branch_Taken || Jump) begin
            InstrFlush = 1'b1;  
            ID_EX_Flush = 1'b1; 
        end
    end
endmodule


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
        
        if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs1))
            ForwardA = 2'b10; 
        else if (MEM_WB_RegWrite && (MEM_WB_Rd != 0) && (MEM_WB_Rd == ID_EX_Rs1))
            ForwardA = 2'b01; 
        else
            ForwardA = 2'b00; 
        
        
        if (EX_MEM_RegWrite && (EX_MEM_Rd != 0) && (EX_MEM_Rd == ID_EX_Rs2))
            ForwardB = 2'b10; 
        else if (MEM_WB_RegWrite && (MEM_WB_Rd != 0) && (MEM_WB_Rd == ID_EX_Rs2))
            ForwardB = 2'b01; 
        else
            ForwardB = 2'b00; 
    end
endmodule
module RISC_V_Pipelined_CPU_TB;
    reg clk;
    reg reset;
    
    
    RISC_V_Pipelined_CPU cpu (
        .clk(clk),
        .reset(reset)
    );
    
    
    initial begin
        $dumpfile("waveform.vcd");
        $dumpvars(0, RISC_V_Pipelined_CPU_TB);
    end
    
    
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    
    initial begin
        reset = 1;
        #10 reset = 0;  

        
        #5;
        
        
            cpu.fetch_stage.instr_mem.instr_mem[0] = 32'h00500093; 
            cpu.fetch_stage.instr_mem.instr_mem[1] = 32'h00A00113; 
            cpu.fetch_stage.instr_mem.instr_mem[2] = 32'h002081B3; 
            cpu.fetch_stage.instr_mem.instr_mem[3] = 32'h0041A233; 
            cpu.fetch_stage.instr_mem.instr_mem[4] = 32'h00208463; 
            cpu.fetch_stage.instr_mem.instr_mem[6] = 32'h004000EF; 
            cpu.fetch_stage.instr_mem.instr_mem[7] = 32'h00008067; 
    end

    
    initial begin
        #200;
        $finish;
    end

    
    initial begin
        $display("Time| PC       | Instr    | ALUResult| Reg1     | Reg2     | Reg3     | Reg4     | Reg4     |Branch     | Jump");

        $monitor("%3t | %h | %h | %h | %h | %h | %h | %h | %h | %b \t| %b", 
                 $time, cpu.PC_Out, cpu.Instr, cpu.ALUResult, 
                 cpu.decode_stage.reg_file.registers[1], 
                 cpu.decode_stage.reg_file.registers[2], 
                 cpu.decode_stage.reg_file.registers[3], 
                    cpu.decode_stage.reg_file.registers[4],
                    cpu.decode_stage.reg_file.registers[5],
                 cpu.BranchTaken, cpu.Jump);
    end

endmodule
