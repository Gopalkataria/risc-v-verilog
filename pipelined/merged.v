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
endmodulemodule Decode(
    input clk,
    input rst,
    input [31:0] instr,
    input [4:0] reg_write_addr, // Passed from WB stage
    input [31:0] reg_write_data, // Passed from WB stage
    input RegWrite_WB, // Control signal from WB stage
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
    output [3:0] alu_ctrl,
    output [2:0] funct3,
    output [6:0] funct7
);

    // Register File
    reg [31:0] registers [0:31];
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'h0;
    end

    // Extract instruction fields
    assign rs1 = instr[19:15];
    assign rs2 = instr[24:20];
    assign rd = instr[11:7];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    // Read registers
    assign read_data1 = (rs1 != 0) ? registers[rs1] : 32'h0;
    assign read_data2 = (rs2 != 0) ? registers[rs2] : 32'h0;

    // Immediate Generation
    ImmGen imm_gen(
        .instr(instr),
        .imm(imm)
    );

    // Control Unit
    ControlUnit control_unit(
        .opcode(instr[6:0]),
        .RegWrite(), // No longer needed here
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg),
        .Jump(Jump),
        .AUIPC(AUIPC)
    );

    // ALU Control (Based on funct fields)
    always @(*) begin
        case (instr[6:0])
            7'b0110011: alu_ctrl = {instr[30], instr[14:12]}; // R-type
            7'b0010011: alu_ctrl = {1'b0, instr[14:12]}; // I-type
            default: alu_ctrl = 4'b0000; // Default NOP
        endcase
    end

    // **Register Write-Back Now Moved to WB Stage**
    always @(posedge clk) begin
        if (RegWrite_WB && reg_write_addr != 0) begin
            registers[reg_write_addr] <= reg_write_data;
        end
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
endmodulemodule Execute(
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
    output reg [31:0] ALU_result,
    output reg branch_taken,
    output reg [31:0] branch_target
);

    // MUX for Forwarding A (Selecting ALU Operand 1)
    reg [31:0] operand1;
    always @(*) begin
        case (forwardA)
            2'b00: operand1 = read_data1;           // From ID/EX pipeline
            2'b01: operand1 = reg_write_data_WB;   // From WB stage
            2'b10: operand1 = alu_result_MEM;      // From MEM stage
            default: operand1 = read_data1;
        endcase
    end
    
    // MUX for Forwarding B (Selecting ALU Operand 2)
    reg [31:0] operand2;
    always @(*) begin
        case (forwardB)
            2'b00: operand2 = (ALUSrc) ? imm : read_data2;
            2'b01: operand2 = reg_write_data_WB;
            2'b10: operand2 = alu_result_MEM;
            default: operand2 = read_data2;
        endcase
    end
    
    // ALU Operation
    alu_64bit alu_unit (
        .funct3(funct3),
        .funct7(funct7),
        .a(operand1),
        .b(operand2),
        .result(ALU_result)
    );
    
    // Branch Target Calculation
    always @(*) begin
        branch_target = PC + (imm << 1);  // PC-relative addressing
    end
    
    // Branch Decision
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (operand1 == operand2) && Branch;  // BEQ
            3'b001: branch_taken = (operand1 != operand2) && Branch;  // BNE
            3'b100: branch_taken = ($signed(operand1) < $signed(operand2)) && Branch; // BLT
            3'b101: branch_taken = ($signed(operand1) >= $signed(operand2)) && Branch; // BGE
            default: branch_taken = 1'b0;
        endcase
    end
    
endmodule
module Fetch(
    input clk,
    input reset,
    input PCWrite, // Controls whether PC updates or stalls
    input [1:0] PCSrc, // Determines next PC source
    input [31:0] branch_target, // Target for branching
    input [31:0] alu_result, // ALU result for jump and branch calculations
    input Instr_Flush, // Control signal to flush IF/ID register
    input IF_ID_Write, // Control signal to allow IF/ID update
    output reg [31:0] next_PC, // Next PC value
    output reg [31:0] current_PC, // Current PC value
    output [31:0] instr // Fetched instruction
);

    wire [31:0] instr_mem_out;
    
    // Use existing Instruction Memory module
    InstructionMemory instr_mem(
        .instr_addr(current_PC),
        .instr(instr_mem_out)
    );
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_PC <= 32'h0;
        end else if (PCWrite) begin
            case (PCSrc)
                2'b00: next_PC = current_PC + 4; // Normal sequential execution
                2'b01: next_PC = branch_target; // Branch target address
                2'b10: next_PC = alu_result; // Jump target address
                default: next_PC = current_PC + 4;
            endcase
            current_PC <= next_PC;
        end
    end
    
    assign instr = Instr_Flush ? 32'h00000013 : instr_mem_out; // NOP if flushed

endmodulemodule ID_EX(
    input clk,
    input reset,
    input ID_EX_Flush, // Control signal for pipeline stall or flush
    input [31:0] pc_in, // Program counter from Decode
    input [31:0] instr_in, // Instruction
    input [31:0] read_data1_in, // Register data 1 from Decode
    input [31:0] read_data2_in, // Register data 2 from Decode
    input [31:0] imm_in, // Immediate value from Decode
    input [4:0] rs1_in, rs2_in, rd_in, // Register addresses
    input [3:0] alu_ctrl_in, // ALU control signal from Decode
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in, alu_src_in, branch_in, jump_in, auipc_in, // Control signals
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
            // Flush pipeline stage when necessary
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
            // Pass values from Decode stage to Execution stage
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
    input IF_ID_Write,  // Control signal to stall pipeline
    input Instr_Flush,  // Control signal to flush instruction
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
            instr_out <= 32'h00000013; // NOP (ADDI x0, x0, 0)
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
module WriteBack (
    input [31:0] ALU_result,
    input [31:0] mem_read_data,
    input MemtoReg,
    output [31:0] reg_write_data
);
    assign reg_write_data = MemtoReg ? mem_read_data : ALU_result;
endmodulemodule full_adder(
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
endmodulemodule Decode(
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

    
    reg [31:0] registers [0:31];
    integer i;
    initial begin
        for (i = 0; i < 32; i = i + 1)
            registers[i] = 32'h0;
    end

    
    assign rs1 = instr[19:15];
    assign rs2 = instr[24:20];
    assign rd = instr[11:7];
    assign funct3 = instr[14:12];
    assign funct7 = instr[31:25];

    
    assign read_data1 = (rs1 != 0) ? registers[rs1] : 32'h0;
    assign read_data2 = (rs2 != 0) ? registers[rs2] : 32'h0;

    
    ImmGen imm_gen(
        .instr(instr),
        .imm(imm)
    );

    
    ControlUnit control_unit(
        .opcode(instr[6:0]),
        .RegWrite(), 
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg),
        .Jump(Jump),
        .AUIPC(AUIPC)
    );

    
 always @(*) begin
    case (instr[6:0])
        7'b0110011: alu_ctrl = {instr[30], instr[14:12]};
        7'b0010011: alu_ctrl = {1'b0, instr[14:12]};
        default:    alu_ctrl = 4'b0000;
    endcase
end
    
    always @(posedge clk) begin
        if (RegWrite_WB && reg_write_addr != 0) begin
            registers[reg_write_addr] <= reg_write_data;
        end
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
endmodulemodule Execute(
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

    // MUX for Forwarding A (Selecting ALU Operand 1)
    reg [31:0] operand1;
    always @(*) begin
        case (forwardA)
            2'b00: operand1 = read_data1;           // From ID/EX pipeline
            2'b01: operand1 = reg_write_data_WB;   // From WB stage
            2'b10: operand1 = alu_result_MEM;      // From MEM stage
            default: operand1 = read_data1;
        endcase
    end
    
    // MUX for Forwarding B (Selecting ALU Operand 2)
    reg [31:0] operand2;
    always @(*) begin
        case (forwardB)
            2'b00: operand2 = (ALUSrc) ? imm : read_data2;
            2'b01: operand2 = reg_write_data_WB;
            2'b10: operand2 = alu_result_MEM;
            default: operand2 = read_data2;
        endcase
    end

    wire [63:0] alu_result_wire;  // Full 64-bit result
    
     alu_64bit alu_unit (
        .funct3(funct3),
        .funct7(funct7),
        .a({{32{operand1[31]}}, operand1}),  // Sign-extend operand1
        .b({{32{operand2[31]}}, operand2}),  // Sign-extend operand2
        .result(alu_result_wire)
    );

    // Assign only the lower 32 bits to ALU_result
    assign ALU_result = alu_result_wire[31:0];
    
    // Branch Target Calculation
    always @(*) begin
        branch_target = PC + (imm << 1);  // PC-relative addressing
    end
    
    // Branch Decision
    always @(*) begin
        case (funct3)
            3'b000: branch_taken = (operand1 == operand2) && Branch;  // BEQ
            3'b001: branch_taken = (operand1 != operand2) && Branch;  // BNE
            3'b100: branch_taken = ($signed(operand1) < $signed(operand2)) && Branch; // BLT
            3'b101: branch_taken = ($signed(operand1) >= $signed(operand2)) && Branch; // BGE
            default: branch_taken = 1'b0;
        endcase
    end
    
endmodule
module Fetch(
    input clk,
    input reset,
    input PCWrite, // Controls whether PC updates or stalls
    input [1:0] PCSrc, // Determines next PC source
    input [31:0] branch_target, // Target for branching
    input [31:0] alu_result, // ALU result for jump and branch calculations
    input Instr_Flush, // Control signal to flush IF/ID register
    input IF_ID_Write, // Control signal to allow IF/ID update
    output reg [31:0] next_PC, // Next PC value
    output reg [31:0] current_PC, // Current PC value
    output [31:0] instr // Fetched instruction
);

    wire [31:0] instr_mem_out;
    
    // Use existing Instruction Memory module
    InstructionMemory instr_mem(
        .instr_addr(current_PC),
        .instr(instr_mem_out)
    );
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_PC <= 32'h0;
        end else if (PCWrite) begin
            case (PCSrc)
                2'b00: next_PC = current_PC + 4; // Normal sequential execution
                2'b01: next_PC = branch_target; // Branch target address
                2'b10: next_PC = alu_result; // Jump target address
                default: next_PC = current_PC + 4;
            endcase
            current_PC <= next_PC;
        end
    end
    
    assign instr = Instr_Flush ? 32'h00000013 : instr_mem_out; // NOP if flushed

endmodulemodule ID_EX(
    input clk,
    input reset,
    input ID_EX_Flush, // Control signal for pipeline stall or flush
    input [31:0] pc_in, // Program counter from Decode
    input [31:0] instr_in, // Instruction
    input [31:0] read_data1_in, // Register data 1 from Decode
    input [31:0] read_data2_in, // Register data 2 from Decode
    input [31:0] imm_in, // Immediate value from Decode
    input [4:0] rs1_in, rs2_in, rd_in, // Register addresses
    input [3:0] alu_ctrl_in, // ALU control signal from Decode
    input mem_read_in, mem_write_in, reg_write_in, mem_to_reg_in, alu_src_in, branch_in, jump_in, auipc_in, // Control signals
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
            // Flush pipeline stage when necessary
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
            // Pass values from Decode stage to Execution stage
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
    input IF_ID_Write,  // Control signal to stall pipeline
    input Instr_Flush,  // Control signal to flush instruction
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
            instr_out <= 32'h00000013; // NOP (ADDI x0, x0, 0)
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
module WriteBack (
    input [31:0] ALU_result,
    input [31:0] mem_read_data,
    input MemtoReg,
    output [31:0] reg_write_data
);
    assign reg_write_data = MemtoReg ? mem_read_data : ALU_result;
endmodule