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

//  module alu_64bit(
//     input [2:0] funct3,     
//     input [6:0] funct7,     
//     input [63:0] a,         
//     input [63:0] b,         
//     output reg [63:0] result 
// );
//          wire [63:0] add_result;
//     wire [63:0] sub_result;
//     wire [63:0] and_result;
//     wire [63:0] or_result;
//     wire [63:0] xor_result;
//     wire [63:0] sll_result;
//     wire [63:0] srl_result;
//     wire [63:0] sra_result;
//     wire [63:0] slt_result;
//     wire [63:0] sltu_result;

//          adder_64bit add_op(
//         .a(a),
//         .b(b),
//         .cin(1'b0),
//         .sum(add_result)
//     );

//     subtractor_64bit sub_op(
//         .a(a),
//         .b(b),
//         .diff(sub_result)
//     );

//     and_64bit and_op(
//         .a(a),
//         .b(b),
//         .result(and_result)
//     );

//     or_64bit or_op(
//         .a(a),
//         .b(b),
//         .result(or_result)
//     );

//     xor_64bit xor_op(
//         .a(a),
//         .b(b),
//         .result(xor_result)
//     );

//     sll_64bit sll_op(
//         .a(a),
//         .shift_amt(b[5:0]),
//         .result(sll_result)
//     );

//     srl_64bit srl_op(
//         .a(a),
//         .shift_amt(b[5:0]),
//         .result(srl_result)
//     );

//     sra_64bit sra_op(
//         .a(a),
//         .shift_amt(b[5:0]),
//         .result(sra_result)
//     );

//          assign slt_result = {63'b0, $signed(a) < $signed(b)};
//     assign sltu_result = {63'b0, a < b};

//          always @(*) begin
//         case(funct3)
//             3'b000: begin  
//                 if (funct7[5]) 
//                     result = sub_result;
//                 else           
//                     result = add_result;
//             end
//             3'b001: result = sll_result;   
//             3'b010: result = slt_result;   
//             3'b011: result = sltu_result;  
//             3'b100: result = xor_result;   
//             3'b101: begin  
//                 if (funct7[5]) 
//                     result = sra_result;
//                 else           
//                     result = srl_result;
//             end
//             3'b110: result = or_result;    
//             3'b111: result = and_result;   
//             default: result = 64'b0;
//         endcase
//     end
// endmodule
module ALU (
    input [31:0] a,
    input [31:0] b,
    input [3:0] alu_control,
    output reg [31:0] result,
    output zero
);
    wire extra;
    always @(*) begin
        case (alu_control)
            4'b0000: adder_64bit adder (
                .a(a),
                .b(b),
                .cin(1'b0),
                .sum(result),
                .cout(extra)
            );
            4'b0010: subtractor_64bit subtractor (
                .a(a),
                .b(b),
                .diff(result)
            );
            4'b0001: or_64bit or_op (
                .a(a),
                .b(b),
                .result(result)
            );
            4'b0011: and_64bit and_op (
                .a(a),
                .b(b),
                .result(result)
            );
            4'b0100: sll_64bit sll_op (
                .a(a),
                .shift_amt(b[4:0]),
                .result(result)
            );
            4'b0101: srl_64bit srl_op (
                .a(a),
                .shift_amt(b[4:0]),
                .result(result)
            );
            4'b1101: sra_64bit sra_op (
                .a(a),
                .shift_amt(b[4:0]),
                .result(result)
            );
            4'b1000: xor_64bit xor_op (
                .a(a),
                .b(b),
                .result(result)
            );
            4'b1101: sra_64bit sra_op (
                .a(a),
                .shift_amt(b[5:0]),
                .result(result)
            );
            
            4'b1000: xor_64bit xor_op (
                .a(a),
                .b(b),
                .result(result)
            );
            4'b0110: result = ($signed(a) < $signed(b)) ? 32'd1 : 32'd0;
            4'b0111: result = (a < b) ? 32'd1 : 32'd0;
            default: result = 32'd0;
        endcase
    end
    assign zero = (result == 32'd0);
endmodule

module ALUControl (
    input [2:0] ALUOp,
    input [2:0] funct3,
    input [6:0] funct7,
    output reg [3:0] ALU_control
);
    always @(*) begin
        case (ALUOp)
            3'b010: begin
                case (funct3)
                    3'b000: ALU_control = (funct7[5] ? 4'b0010 : 4'b0000);
                    3'b001: ALU_control = 4'b0100;
                    3'b010: ALU_control = 4'b0110;
                    3'b011: ALU_control = 4'b0111;
                    3'b100: ALU_control = 4'b1000;
                    3'b101: ALU_control = (funct7[5] ? 4'b1101 : 4'b0101);
                    3'b110: ALU_control = 4'b0001;
                    3'b111: ALU_control = 4'b0011;
                    default: ALU_control = 4'b0000;
                endcase
            end
            3'b011: begin
                case (funct3)
                    3'b000: ALU_control = 4'b0000;
                    3'b010: ALU_control = 4'b0110;
                    3'b011: ALU_control = 4'b0111;
                    3'b100: ALU_control = 4'b1000;
                    3'b110: ALU_control = 4'b0001;
                    3'b111: ALU_control = 4'b0011;
                    3'b001: ALU_control = 4'b0100;
                    3'b101: ALU_control = (funct7[5] ? 4'b1101 : 4'b0101);
                    default: ALU_control = 4'b0000;
                endcase
            end
            3'b000: ALU_control = 4'b0000;
            3'b001: ALU_control = 4'b0010;
            default: ALU_control = 4'b0000;
        endcase
    end
endmodule

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
        ALUOp   = 3'b000;
        RegWrite= 1'b0;
        MemRead = 1'b0;
        MemWrite= 1'b0;
        ALUSrc  = 1'b0;
        Branch  = 1'b0;
        MemtoReg= 1'b0;
        Jump    = 1'b0;
        AUIPC   = 1'b0;
        case (opcode)
            7'b0110011: begin
                ALUOp    = 3'b010;
                RegWrite = 1'b1;
            end
            7'b0010011: begin
                ALUOp    = 3'b011;
                RegWrite = 1'b1;
                ALUSrc   = 1'b1;
            end
            7'b0000011: begin
                ALUOp    = 3'b000;
                RegWrite = 1'b1;
                ALUSrc   = 1'b1;
                MemRead  = 1'b1;
                MemtoReg = 1'b1;
            end
            7'b0100011: begin
                ALUOp   = 3'b000;
                ALUSrc  = 1'b1;
                MemWrite= 1'b1;
            end
            7'b1100011: begin
                ALUOp   = 3'b001;
                Branch  = 1'b1;
            end
            7'b1101111: begin
                Jump    = 1'b1;
                RegWrite= 1'b1;
                ALUSrc  = 1'b1;
            end
            7'b1100111: begin
                Jump    = 1'b1;
                RegWrite= 1'b1;
                ALUSrc  = 1'b1;
            end
            7'b0110111: begin
                ALUOp   = 3'b100;
                RegWrite= 1'b1;
                ALUSrc  = 1'b1;
            end
            7'b0010111: begin
                AUIPC   = 1'b1;
                RegWrite= 1'b1;
                ALUSrc  = 1'b1;
            end
            default: ;
        endcase
    end
endmodule

module ImmGen (
    input [31:0] instr,
    output reg [31:0] imm
);
    always @(*) begin
        case (instr[6:0])
            7'b0010011,7'b0000011: imm = {{20{instr[31]}},instr[31:20]};
            7'b0100011:         imm = {{20{instr[31]}},instr[31:25],instr[11:7]};
            7'b1100011:         imm = {{20{instr[31]}},instr[7],instr[30:25],instr[11:8],1'b0};
            7'b1101111:         imm = {{12{instr[31]}},instr[19:12],instr[20],instr[30:21],1'b0};
            7'b0110111,
            7'b0010111:         imm = {instr[31:12],12'b0};
            default:            imm = 32'd0;
        endcase
    end
endmodule

module Decode (
    input clk,
    input rst,
    input [31:0] instr,
    input [31:0] reg_write_data,
    output [31:0] read_data1,
    output [31:0] read_data2,
    output [31:0] imm,
    output [2:0] ALUOp,
    output RegWrite,
    output MemRead,
    output MemWrite,
    output ALUSrc,
    output Branch,
    output MemtoReg
);
    reg [31:0] reg_file [31:0];
    wire [6:0] opcode = instr[6:0];
    ControlUnit ctrl (
        .opcode(opcode),
        .ALUOp(ALUOp),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg),
        .Jump(),
        .AUIPC()
    );
    ImmGen imm_gen (
        .instr(instr),
        .imm(imm)
    );
    assign read_data1 = reg_file[instr[19:15]];
    assign read_data2 = reg_file[instr[24:20]];
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            integer i;
            for(i = 0; i < 32; i = i + 1)
                reg_file[i] <= 32'd0;
        end else if (RegWrite && (instr[11:7] != 5'b00000))
            reg_file[instr[11:7]] <= reg_write_data;
    end
endmodule

module Execute (
    input [31:0] PC,
    input [31:0] read_data1,
    input [31:0] read_data2,
    input [31:0] imm,
    input [2:0] ALUOp,
    input ALUSrc,
    input Branch,
    input [2:0] funct3,
    output [31:0] ALU_result,
    output branch_taken,
    output [31:0] branch_target
);
    wire [31:0] operand2 = ALUSrc ? imm : read_data2;
    wire zero;
    wire [3:0] alu_control;
    ALUControl alu_ctrl (
        .ALUOp(ALUOp),
        .funct3(funct3),
        .funct7(imm[31:25]),
        .ALU_control(alu_control)
    );
    ALU alu (
        .a(read_data1),
        .b(operand2),
        .alu_control(alu_control),
        .result(ALU_result),
        .zero(zero)
    );
    assign branch_target = PC + imm;
    assign branch_taken = Branch & (((funct3 == 3'b000) & zero) | ((funct3 == 3'b001) & ~zero));
endmodule

module Fetch (
    input clk,
    input reset,
    input PCSrc,
    input [31:0] branch_target,
    output [31:0] PC,
    output [31:0] instr
);
    reg [31:0] PC_reg;
    wire [31:0] next_PC;
    reg [31:0] instr_mem [0:1023];
    assign instr = instr_mem[PC_reg >> 2];
    assign next_PC = PCSrc ? branch_target : PC_reg + 4;
    always @(posedge clk or posedge reset) begin
        if (reset)
            PC_reg <= 32'd0;
        else
            PC_reg <= next_PC;
    end
    assign PC = PC_reg;
    initial begin
        integer i;
        for(i = 0; i < 1024; i = i + 1)
            instr_mem[i] = 32'd0;
        instr_mem[0] = {7'b0000000,5'd0,5'd0,3'b000,5'd1,7'b0110011};
    end
endmodule

module Memory (
    input clk,
    input [31:0] address,
    input [31:0] write_data,
    input MemRead,
    input MemWrite,
    output [31:0] read_data
);
    reg [31:0] data_mem [0:1023];
    assign read_data = MemRead ? data_mem[address >> 2] : 32'd0;
    always @(posedge clk) begin
        if (MemWrite)
            data_mem[address >> 2] <= write_data;
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

module RISC_V_Single_Cycle (
    input clk,
    input reset
);
    wire [31:0] PC, instr;
    wire branch_taken;
    wire [31:0] branch_target;
    wire [31:0] read_data1, read_data2, imm;
    wire [2:0] ALUOp;
    wire RegWrite, MemRead, MemWrite, ALUSrc, Branch, MemtoReg;
    wire [31:0] ALU_result;
    wire [31:0] mem_read_data;
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
        .rst(reset),
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

module tb_RISC_V_Single_Cycle;
    reg clk;
    reg reset;

    RISC_V_Single_Cycle dut (.clk(clk), .reset(reset));

    // Clock generation: period = 10 time units
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Reset generation
    initial begin
        reset = 1;
        #15 reset = 0;
    end

    // Dump waveforms for debugging
    initial begin
        $dumpfile("riscv_single_cycle.vcd");
        $dumpvars(0, tb_RISC_V_Single_Cycle);
    end

    // Load instructions from memory.txt into instruction memory
    initial begin
        #1;
        $readmemh("memory.txt", dut.fetch.instr_mem);
    end

    // End simulation and write final data memory contents to final_memory.txt
    initial begin
        #300;
        $writememh("final_memory.txt", dut.memory.data_mem);
        $finish;
    end
endmodule

