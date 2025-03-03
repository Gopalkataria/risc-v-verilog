// Basic building blocks - unchanged from single-cycle design
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

// 64-bit adder module - unchanged
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

// Two's complement module - unchanged
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

// 64-bit subtractor module - unchanged
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

// 64-bit AND module - unchanged
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

// 64-bit OR module - unchanged
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

// 64-bit XOR module - unchanged
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

// 64-bit shift left logical module - unchanged
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

// 64-bit shift right logical module - unchanged
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

// 64-bit shift right arithmetic module - unchanged
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

// 64-bit ALU module - unchanged
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

// Immediate Generator module - unchanged
module ImmGen (
    input [31:0] instr,
    output reg [31:0] imm
);
    always @(*) begin
        case (instr[6:0])  // opcode
            // I-type (ADDI, LW)
            7'b0010011, 7'b0000011: 
                imm = {{20{instr[31]}}, instr[31:20]};  // Sign-extend

            // S-type (SW)
            7'b0100011: 
                imm = {{20{instr[31]}}, instr[31:25], instr[11:7]};

            // B-type (BEQ, BNE)
            7'b1100011: 
                imm = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0};

            // J-type (JAL)
            7'b1101111: 
                imm = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0};

            // U-type (LUI, AUIPC)
            7'b0110111, 7'b0010111: 
                imm = {instr[31:12], 12'b0};

            default: imm = 32'b0;
        endcase
    end
endmodule

//-------------------- MULTI-CYCLE SPECIFIC MODULES --------------------
module MultiCycleControlUnit (
    input clk,
    input reset,
    input [6:0] opcode,
    input [2:0] funct3,
    input [6:0] funct7,
    input zero,      // ALU zero flag
    
    // State outputs
    output reg [3:0] state,
    output reg [3:0] next_state,
    
    // Control signals
    output reg PCWrite,        // Enable PC update
    output reg PCSource,       // 0: PC+4, 1: branch/jump target
    output reg ALUSrcA,        // 0: PC, 1: RS1
    output reg [1:0] ALUSrcB,  // 00: RS2, 01: 4, 10: immediate, 11: unused
    output reg [2:0] ALUOp,    // ALU operation
    output reg RegWrite,       // Register file write enable
    output reg MemRead,        // Memory read enable
    output reg MemWrite,       // Memory write enable
    output reg IRWrite,        // Instruction register write enable
    output reg MemtoReg,       // 0: ALU result, 1: Memory data
    output reg [1:0] BranchCtrl // 00: No branch, 01: BEQ, 10: BNE, 11: other branches
);
    // State definitions
    parameter S_FETCH    = 4'd0;   // Instruction fetch
    parameter S_DECODE   = 4'd1;   // Instruction decode and register fetch
    parameter S_EXECUTE  = 4'd2;   // Execution or address computation
    parameter S_MEMORY   = 4'd3;   // Memory access
    parameter S_WRITEBACK = 4'd4;  // Write back to register
    parameter S_BRANCH   = 4'd5;   // Branch completion
    parameter S_JUMP     = 4'd6;   // Jump instruction
    parameter S_AUIPC    = 4'd7;   // Add upper immediate to PC
    parameter S_LUI      = 4'd8;   // Load upper immediate
    
    // Opcode definitions
    parameter OP_R_TYPE   = 7'b0110011;
    parameter OP_I_TYPE   = 7'b0010011;
    parameter OP_LOAD     = 7'b0000011;
    parameter OP_STORE    = 7'b0100011;
    parameter OP_BRANCH   = 7'b1100011;
    parameter OP_JAL      = 7'b1101111;
    parameter OP_JALR     = 7'b1100111;
    parameter OP_LUI      = 7'b0110111;
    parameter OP_AUIPC    = 7'b0010111;
    
    // State register
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= S_FETCH;
        else
            state <= next_state;
    end
    
    // Next state logic
    always @(*) begin
        case (state)
            S_FETCH: begin
                next_state = S_DECODE;
            end
            
            S_DECODE: begin
                case (opcode)
                    OP_R_TYPE, OP_I_TYPE:
                        next_state = S_EXECUTE;
                    OP_LOAD, OP_STORE:
                        next_state = S_EXECUTE;  // Address calculation
                    OP_BRANCH:
                        next_state = S_BRANCH;
                    OP_JAL, OP_JALR:
                        next_state = S_JUMP;
                    OP_LUI:
                        next_state = S_LUI;
                    OP_AUIPC:
                        next_state = S_AUIPC;
                    default:
                        next_state = S_FETCH;  // Invalid instruction, restart
                endcase
            end
            
            S_EXECUTE: begin
                case (opcode)
                    OP_R_TYPE, OP_I_TYPE:
                        next_state = S_WRITEBACK;
                    OP_LOAD:
                        next_state = S_MEMORY;
                    OP_STORE:
                        next_state = S_MEMORY;
                    default:
                        next_state = S_FETCH;
                endcase
            end
            
            S_MEMORY: begin
                case (opcode)
                    OP_LOAD:
                        next_state = S_WRITEBACK;
                    OP_STORE:
                        next_state = S_FETCH;  // After store, go back to fetch
                    default:
                        next_state = S_FETCH;
                endcase
            end
            
            S_WRITEBACK: begin
                next_state = S_FETCH;  // After writeback, go back to fetch
            end
            
            S_BRANCH, S_JUMP, S_AUIPC, S_LUI: begin
                next_state = S_FETCH;  // After these specialized states, go back to fetch
            end
            
            default: begin
                next_state = S_FETCH;  // Any unknown state goes back to fetch
            end
        endcase
    end
    
    // Control signal generation
    always @(*) begin
        // Default values
        PCWrite = 1'b0;
        PCSource = 1'b0;
        ALUSrcA = 1'b0;
        ALUSrcB = 2'b00;
        ALUOp = 3'b000;
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        IRWrite = 1'b0;
        MemtoReg = 1'b0;
        BranchCtrl = 2'b00;
        
        case (state)
            S_FETCH: begin
                MemRead = 1'b1;    // Read from memory
                IRWrite = 1'b1;    // Write to instruction register
                ALUSrcA = 1'b0;    // ALU input A is PC
                ALUSrcB = 2'b01;   // ALU input B is 4 (PC + 4)
                ALUOp = 3'b000;    // Addition
                PCWrite = 1'b1;    // Update PC
                PCSource = 1'b0;   // PC source is ALU result (PC + 4)
            end
            
            S_DECODE: begin
                ALUSrcA = 1'b0;    // ALU input A is PC
                ALUSrcB = 2'b10;   // ALU input B is immediate
                ALUOp = 3'b000;    // Addition (for branch/jump target calculation)
            end
            
            S_EXECUTE: begin
                ALUSrcA = 1'b1;    // ALU input A is register
                
                if (opcode == OP_R_TYPE)
                    ALUSrcB = 2'b00;  // ALU input B is register
                else
                    ALUSrcB = 2'b10;  // ALU input B is immediate
                
                // ALU operation based on function code
                if (opcode == OP_R_TYPE || opcode == OP_I_TYPE) begin
                    case (funct3)
                        3'b000: ALUOp = (opcode == OP_R_TYPE && funct7[5]) ? 3'b001 : 3'b000; // ADD/SUB
                        3'b001: ALUOp = 3'b010; // SLL
                        3'b010: ALUOp = 3'b011; // SLT
                        3'b011: ALUOp = 3'b100; // SLTU
                        3'b100: ALUOp = 3'b101; // XOR
                        3'b101: ALUOp = (funct7[5]) ? 3'b111 : 3'b110; // SRA/SRL
                        3'b110: ALUOp = 3'b000; // OR
                        3'b111: ALUOp = 3'b000; // AND
                        default: ALUOp = 3'b000;
                    endcase
                end else begin
                    ALUOp = 3'b000; // Default to addition for load/store
                end
            end
            
            S_MEMORY: begin
                if (opcode == OP_LOAD) begin
                    MemRead = 1'b1;  // Read from memory
                end else if (opcode == OP_STORE) begin
                    MemWrite = 1'b1; // Write to memory
                end
            end
            
            S_WRITEBACK: begin
                RegWrite = 1'b1;  // Write to register file
                
                if (opcode == OP_LOAD)
                    MemtoReg = 1'b1; // Write from memory
                else
                    MemtoReg = 1'b0; // Write from ALU
            end
            
            S_BRANCH: begin
                ALUSrcA = 1'b1;     // ALU input A is register
                ALUSrcB = 2'b00;    // ALU input B is register
                ALUOp = 3'b001;     // Subtraction for comparison
                PCWrite = 1'b1;     // Update PC
                PCSource = 1'b1;    // PC source is branch target
                
                // Branch control based on funct3
                case (funct3)
                    3'b000: BranchCtrl = 2'b01; // BEQ
                    3'b001: BranchCtrl = 2'b10; // BNE
                    3'b100, 3'b101, 3'b110, 3'b111: BranchCtrl = 2'b11; // Other branches
                    default: BranchCtrl = 2'b00;
                endcase
            end
            
            S_JUMP: begin
                PCWrite = 1'b1;    // Update PC
                PCSource = 1'b1;   // PC source is jump target
                RegWrite = 1'b1;   // Write PC+4 to register
                MemtoReg = 1'b0;   // ALU result (PC+4)
            end
            
            S_AUIPC: begin
                ALUSrcA = 1'b0;    // ALU input A is PC
                ALUSrcB = 2'b10;   // ALU input B is immediate
                ALUOp = 3'b000;    // Addition
                RegWrite = 1'b1;   // Write to register
                MemtoReg = 1'b0;   // ALU result
            end
            
            S_LUI: begin
                RegWrite = 1'b1;   // Write to register
                MemtoReg = 1'b0;   // Write immediate directly (handled specially)
            end
            
            default: begin
                // Default case - all signals off
            end
        endcase
    end
endmodule


// Memory module for multi-cycle design
module Memory (
    input clk,
    input reset,
    input [31:0] address,     // Combined address for instruction and data
    input [31:0] write_data,  // Data to write
    input MemRead,            // Control signal for read
    input MemWrite,           // Control signal for write
    output reg [31:0] read_data // Data output
);
    // Combined memory (64KB total - 16K words)
    reg [31:0] mem [0:16383];  // 16K words of 32 bits each
    
    // Initialize memory
    integer i;
    initial begin
        for (i = 0; i < 16384; i = i + 1)
            mem[i] = 32'h0;
            
        You can add initial program code here:
        Example: Simple program that adds numbers
        mem[0] = 32'h00500093;  // addi x1, x0, 5
        mem[1] = 32'h00A00113;  // addi x2, x0, 10
        mem[2] = 32'h002081B3;  // add x3, x1, x2
    end

    // Memory read (combinational)
    always @(*) begin
        read_data = MemRead ? mem[address[15:2]] : 32'h0;
    end

    // Memory write (synchronous)
    always @(posedge clk) begin
        if (MemWrite)
            mem[address[15:2]] <= write_data;
    end
endmodule

// Register module - Multi-purpose register with enable
module Register #(parameter WIDTH = 32) (
    input clk,
    input reset,
    input enable,
    input [WIDTH-1:0] d,
    output reg [WIDTH-1:0] q
);
    always @(posedge clk or posedge reset) begin
        if (reset)
            q <= {WIDTH{1'b0}};
        else if (enable)
            q <= d;
    end
endmodule

// Multi-Cycle CPU Datapath
module MultiCycleDatapath (
    input clk,
    input reset,
    
    // Control signals from Control Unit
    input PCWrite,
    input PCSource,
    input ALUSrcA,
    input [1:0] ALUSrcB,
    input [2:0] ALUOp,
    input RegWrite,
    input MemRead,
    input MemWrite,
    input IRWrite,
    input MemtoReg,
    input [1:0] BranchCtrl,
    
    // Outputs to Control Unit
    output [6:0] opcode,
    output [2:0] funct3,
    output [6:0] funct7,
    output zero,
    
    // For debugging
    output [31:0] PC_debug,
    output [31:0] IR_debug
);
    // ---- Internal wires and registers ----
    
    // Program Counter
    wire [31:0] PC_next;
    wire [31:0] PC;
    wire PC_enable;
    
    // Memory interface
    wire [31:0] memory_address;
    wire [31:0] memory_write_data;
    wire [31:0] memory_read_data;
    
    // Instruction Register
    wire [31:0] IR;
    
    // Register File
    wire [4:0] rs1_addr, rs2_addr, rd_addr;
    wire [31:0] rd_data;
    wire [31:0] rs1_data, rs2_data;
    
    // Immediate Generator
    wire [31:0] imm_data;
    
    // ALU
    wire [31:0] alu_input_a, alu_input_b;
    wire [31:0] alu_result;
    wire [63:0] alu_result_64;
    
    // Temporary registers
    wire [31:0] A, B;  // Register outputs
    wire [31:0] ALUOut;
    wire [31:0] MDR;  // Memory Data Register
    
    // Branch control
    wire branch_taken;
    
    // ---- Component Connections ----
    
    // Instruction fields extraction
    assign opcode = IR[6:0];
    assign funct3 = IR[14:12];
    assign funct7 = IR[31:25];
    assign rs1_addr = IR[19:15];
    assign rs2_addr = IR[24:20];
    assign rd_addr = IR[11:7];
    
    // PC Logic
    assign PC_enable = PCWrite | (branch_taken & BranchCtrl != 2'b00);
    assign PC_next = PCSource ? ALUOut : alu_result;
    
    // Branch decision logic
    assign branch_taken = (BranchCtrl == 2'b01 & zero) |               // BEQ
                         (BranchCtrl == 2'b10 & ~zero) |              // BNE
                         (BranchCtrl == 2'b11 & (funct3 == 3'b100 & alu_result[0]) |  // BLT
                                          (funct3 == 3'b101 & ~alu_result[0]) |        // BGE
                                          (funct3 == 3'b110 & alu_result[0]) |         // BLTU
                                          (funct3 == 3'b111 & ~alu_result[0]));        // BGEU
    
    // ALU input muxes
    assign alu_input_a = ALUSrcA ? A : PC;
    
    always @(*) begin
        case (ALUSrcB)
            2'b00: alu_input_b = B;
            2'b01: alu_input_b = 32'd4;
            2'b10: alu_input_b = imm_data;
            2'b11: alu_input_b = 32'd0;
        endcase
    end
    
    // Memory address and data selection
    assign memory_address = (state == S_FETCH) ? PC : ALUOut;
    assign memory_write_data = B;
    
    // Register write data selection
    assign rd_data = MemtoReg ? MDR : ALUOut;
    
    // Debug outputs
    assign PC_debug = PC;
    assign IR_debug = IR;
    
    // ---- Component Instantiations ----
    
    // Program Counter register
    Register PC_reg (
        .clk(clk),
        .reset(reset),
        .enable(PC_enable),
        .d(PC_next),
        .q(PC)
    );
    
    // Instruction Register
    Register IR_reg (
        .clk(clk),
        .reset(reset),
        .enable(IRWrite),
        .d(memory_read_data),
        .q(IR)
    );
    
    // Register File
    reg [31:0] reg_file [0:31];
    
    // Register read
    assign rs1_data = (rs1_addr == 5'b0) ? 32'b0 : reg_file[rs1_addr];
    assign rs2_data = (rs2_addr == 5'b0) ? 32'b0 : reg_file[rs2_addr];
    
    // Register write
    always @(posedge clk) begin
        if (RegWrite && rd_addr != 5'b0)
            reg_file[rd_addr] <= rd_data;
    end
    
    // A and B temporary registers
    Register A_reg (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),  // Always enabled in decode stage
        .d(rs1_data),
        .q(A)
    );
    
    Register B_reg (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),  // Always enabled in decode stage
        .d(rs2_data),
        .q(B)
    );
    
    // ALU Output register
    Register ALUOut_reg (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),  // Always enabled in execute stage
        .d(alu_result),
        .q(ALUOut)
    );
    
    // Memory Data Register
    Register MDR_reg (
        .clk(clk),
        .reset(reset),
        .enable(1'b1),  // Always enabled in memory stage
        .d(memory_read_data),
        .q(MDR)
    );
    
    // Immediate Generator
    ImmGen imm_gen (
        .instr(IR),
        .imm(imm_data)
    );
    
    wire [63:0] alu_a_64 = {{32{alu_input_a[31]}}, alu_input_a};
    wire [63:0] alu_b_64 = {{32{alu_input_b[31]}}, alu_input_b};

    alu_64bit alu (
        .funct3(funct3),
        .funct7(funct7),
        .a(alu_a_64),
        .b(alu_b_64),
        .result(alu_result_64)
    );

    assign alu_result = alu_result_64[31:0];  // Use lower 32 bits
    assign zero = (alu_result == 32'b0);      // Zero flag for branches


        // Memory module
    Memory memory (
        .clk(clk),
        .reset(reset),
        .address(memory_address),
        .write_data(memory_write_data),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .read_data(memory_read_data)
    );

    

    module MultiCycleCPU (
    input clk,
    input reset
);
    // Control signals
    wire PCWrite, PCSource, ALUSrcA, RegWrite, MemRead, MemWrite, IRWrite, MemtoReg;
    wire [1:0] ALUSrcB, BranchCtrl;
    wire [2:0] ALUOp;
    
    // Datapath outputs
    wire [6:0] opcode;
    wire [2:0] funct3;
    wire [6:0] funct7;
    wire zero;
    
    // Debug outputs
    wire [31:0] PC_debug, IR_debug;

    // Instantiate Control Unit
    MultiCycleControlUnit control_unit (
        .clk(clk),
        .reset(reset),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .zero(zero),
        .PCWrite(PCWrite),
        .PCSource(PCSource),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ALUOp(ALUOp),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .IRWrite(IRWrite),
        .MemtoReg(MemtoReg),
        .BranchCtrl(BranchCtrl)
    );

    // Instantiate Datapath
    MultiCycleDatapath datapath (
        .clk(clk),
        .reset(reset),
        .PCWrite(PCWrite),
        .PCSource(PCSource),
        .ALUSrcA(ALUSrcA),
        .ALUSrcB(ALUSrcB),
        .ALUOp(ALUOp),
        .RegWrite(RegWrite),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .IRWrite(IRWrite),
        .MemtoReg(MemtoReg),
        .BranchCtrl(BranchCtrl),
        .opcode(opcode),
        .funct3(funct3),
        .funct7(funct7),
        .zero(zero),
        .PC_debug(PC_debug),
        .IR_debug(IR_debug)
    );
endmodule

