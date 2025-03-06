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

// 64-bit adder module
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

// Two's complement module
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

// 64-bit subtractor module
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

// 64-bit AND module
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

// 64-bit OR module
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

// 64-bit XOR module
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

// 64-bit shift left logical module
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
                {shift_stage[i-1][63-(2*i):0], {(2*i){1'b0}}} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule

// 64-bit shift right logical module
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
                {{(2*i){1'b0}}, shift_stage[i-1][63:2*i]} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule

// 64-bit shift right arithmetic module
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
                {{(2*i){sign_bit}}, shift_stage[i-1][63:2*i]} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule

// 64-bit ALU module
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

//------------------------------------

// Control Unit module - fixed to ensure all outputs are assigned
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
        // Default values
        RegWrite = 1'b0;
        MemRead = 1'b0;
        MemWrite = 1'b0;
        ALUSrc = 1'b0;
        Branch = 1'b0;
        MemtoReg = 1'b0;
        Jump = 1'b0;
        AUIPC = 1'b0;

        case (opcode)
            // R-type (ADD, SUB, AND, OR, etc.)
            7'b0110011: begin
                RegWrite = 1'b1;
            end

            // I-type (ADDI, ORI, ANDI, etc.)
            7'b0010011: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;  // Use immediate
            end

            // Load (LW)
            7'b0000011: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
                MemRead = 1'b1;
                MemtoReg = 1'b1; // Write data from memory
            end

            // Store (SW)
            7'b0100011: begin
                ALUSrc = 1'b1;
                MemWrite = 1'b1;
            end

            // Branch (BEQ, BNE, etc.)
            7'b1100011: begin
                Branch = 1'b1;
            end

            // Jump (JAL)
            7'b1101111: begin
                Jump = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            // JALR (Jump and Link Register)
            7'b1100111: begin
                Jump = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            // LUI (Load Upper Immediate)
            7'b0110111: begin
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end

            // AUIPC (Add Upper Immediate to PC)
            7'b0010111: begin
                AUIPC = 1'b1;
                RegWrite = 1'b1;
                ALUSrc = 1'b1;
            end
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

module Memory (
    input clk,
    input reset,
    input [31:0] instr_addr,   // Address for instruction fetch
    input [31:0] data_addr,    // Address for data access
    input [31:0] write_data,   // Data to write
    input MemRead,             // Control signal for data read
    input MemWrite,            // Control signal for data write
    output reg [31:0] instr,   // Instruction output (changed to reg)
    output reg [31:0] read_data // Data output (changed to reg)
);
    // Combined memory (64KB total - 16K words)
    reg [31:0] mem [0:16383];  // 16K words of 32 bits each
    
    // Parameters to divide memory spaces
    parameter INSTR_BASE = 0;      
    parameter DATA_BASE = 8192;    

    // Initialize memory
    integer i;
    initial begin
        for (i = 0; i < 16384; i = i + 1)
            mem[i] = 32'h0;
    end

    // Instruction read (sequential for consistency)
    always @(*) begin
        instr = mem[INSTR_BASE + (instr_addr >> 2)];
    end

    // Data read (sequential for consistency)
    always @(*) begin
        read_data = MemRead ? mem[DATA_BASE + (data_addr >> 2)] : 32'h0;
    end

    // Data write (synchronous)
    always @(posedge clk) begin
        if (MemWrite)
            mem[DATA_BASE + (data_addr >> 2)] <= write_data;
    end
endmodule


module Fetch (
    input clk,
    input reset,
    input PCSrc,               // Branch taken signal
    input [31:0] branch_target, // Branch target address
    output [31:0] PC,          // Program Counter
    output [31:0] instr,       // Fetched instruction
    output [31:0] instr_addr,  // Instruction address (PC)
    input [31:0] instr_data    // Instruction data from memory
);
    reg [31:0] PC_reg;         // Internal PC register
    wire [31:0] next_PC;       // Next PC value

    // Next PC logic
    assign next_PC = PCSrc ? branch_target : (PC_reg + 4);

    // Update PC on clock edge
    always @(posedge clk or posedge reset) begin
        if (reset)
            PC_reg <= 32'h0;   // Reset PC to 0
        else
            PC_reg <= next_PC; // Update PC to next_PC
    end

    // Assign outputs
    assign PC = PC_reg;        // Output the current PC
    assign instr = instr_data; // Output the fetched instruction
    assign instr_addr = PC_reg; // Output the instruction address (PC)
endmodule

// Fixed Decode module
module Decode (
    input clk,
    input rst,                 // Reset input
    input [31:0] instr,
    input [31:0] reg_write_data,
    input RegWrite,            // External control signal
    output [31:0] read_data1,
    output [31:0] read_data2,
    output [31:0] imm,
    output MemRead,
    output MemWrite,
    output ALUSrc,
    output Branch,
    output MemtoReg,
    output Jump,               // Added output for Jump signal
    output AUIPC,              // Added output for AUIPC signal
    output [2:0] funct3        // Added output for funct3
);
    reg [31:0] reg_file [31:0];  // Register file
    integer i;                   // Module level declaration for loop variable
    
    // Control Unit
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

    // Immediate Generator
    ImmGen imm_gen (
        .instr(instr),
        .imm(imm)
    );

    // Read registers
    assign read_data1 = reg_file[instr[19:15]];  // rs1
    assign read_data2 = reg_file[instr[24:20]];  // rs2
    
    // Extract funct3 field from instruction
    assign funct3 = instr[14:12];

    // Register Write
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1)
                reg_file[i] <= 32'b0;
        end else if (RegWrite && instr[11:7] != 5'b00000) begin
            reg_file[instr[11:7]] <= reg_write_data;  // Avoid writing to x0
        end
    end
endmodule

// Fixed Execute module
module Execute (
    input [31:0] PC,
    input [31:0] read_data1,
    input [31:0] read_data2,
    input [31:0] imm,
    input ALUSrc,
    input Branch,
    input [2:0] funct3,
    input [6:0] funct7,         // From instruction[31:25]
    output [31:0] ALU_result,
    output branch_taken,
    output [31:0] branch_target
);
    // Sign-extend 32-bit inputs to 64-bit for the ALU
    wire [63:0] read_data1_64 = {{32{read_data1[31]}}, read_data1};
    wire [63:0] read_data2_64 = {{32{read_data2[31]}}, read_data2};
    wire [63:0] imm_64 = {{32{imm[31]}}, imm};
    wire [63:0] operand2_64 = ALUSrc ? imm_64 : read_data2_64;
    
    wire [63:0] alu_result_64;
    
    // ALU instance
    alu_64bit alu (
        .funct3(funct3),
        .funct7(funct7),
        .a(read_data1_64),
        .b(operand2_64),
        .result(alu_result_64)
    );
    
    // Take the lower 32 bits of the result
    assign ALU_result = alu_result_64[31:0];

    // Branch logic - expanded to handle all RISC-V branch conditions
    assign branch_target = PC + imm;
    assign branch_taken = Branch & (
        (funct3 == 3'b000 & (read_data1 == read_data2)) |                // beq
        (funct3 == 3'b001 & (read_data1 != read_data2)) |                // bne
        (funct3 == 3'b100 & ($signed(read_data1) < $signed(read_data2))) |  // blt
        (funct3 == 3'b101 & ($signed(read_data1) >= $signed(read_data2))) | // bge
        (funct3 == 3'b110 & (read_data1 < read_data2)) |                 // bltu
        (funct3 == 3'b111 & (read_data1 >= read_data2))                  // bgeu
    );
endmodule

// WriteBack module remains unchanged
module WriteBack (
    input [31:0] ALU_result,
    input [31:0] mem_read_data,
    input MemtoReg,
    output [31:0] reg_write_data
);
    assign reg_write_data = MemtoReg ? mem_read_data : ALU_result;
endmodule
module RISC_V_Multi_Cycle (
    input clk,
    input reset
);
    // State definition
    reg [3:0] state, next_state;
    localparam FETCH = 4'b0000,
               DECODE = 4'b0001,
               EXECUTE = 4'b0010,
               MEMORY = 4'b0011,
               WRITEBACK = 4'b0100,
               BRANCH_COMPLETION = 4'b0101;

    // Register file
    reg [31:0] RegisterFile [31:0]; 
    integer i;

    // Control signals
    reg PCWrite, IRWrite, RegWrite, ALUSrcA, MemtoReg, IorD;
    reg [1:0] ALUSrcB, PCSrc;
    wire MemRead, MemWrite, Branch, Jump, AUIPC;

    // Intermediate registers and wires
    reg [31:0] PC, IR, A, B, ALUOut, MDR;
    wire [31:0] imm;
    wire [31:0] mem_data;
    wire [31:0] mem_address = IorD ? ALUOut : PC;
    wire [31:0] alu_a = ALUSrcA ? A : PC;
    wire [31:0] alu_b;
    wire [31:0] reg_write_data;
    wire [31:0] branch_target;
    wire branch_taken;
    wire [2:0] funct3;

    // ALU inputs
    wire [6:0] funct7 = IR[31:25];
    wire [31:0] read_data1 = A;
    wire [31:0] read_data2 = B;
    wire [31:0] ALU_result;

    // Instruction fields
    wire [6:0] opcode = IR[6:0];

    // Registers for supporting multi-cycle operation
    reg [31:0] PC_next;

    // Immediate generation
    ImmGen imm_generator (
        .instr(IR),
        .imm(imm)
    );

    // Control Unit
    ControlUnit control_unit (
        .opcode(opcode),
        .RegWrite(RegWrite_ctrl),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .MemtoReg(MemtoReg_ctrl),
        .Jump(Jump),
        .AUIPC(AUIPC)
    );

    // ALU source B selection
    assign alu_b = ALUSrcB[1] ? (ALUSrcB[0] ? imm << 2 : imm) : 
                               (ALUSrcB[0] ? 32'h4 : B);

    // Execute module for ALU operations and branch calculations
    Execute execute_unit (
        .PC(PC),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .imm(imm),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .funct3(IR[14:12]),
        .funct7(funct7),
        .ALU_result(ALU_result),
        .branch_taken(branch_taken),
        .branch_target(branch_target)
    );

    // Memory access
    Memory memory (
        .clk(clk),
        .IorD(IorD),
        .address(mem_address),
        .write_data(B),
        .MemRead(MemRead),
        .MemWrite(MemWrite),
        .read_data(mem_data)
    );

    // Write back
    WriteBack writeback (
        .ALU_result(ALUOut),
        .mem_read_data(MDR),
        .MemtoReg(MemtoReg),
        .reg_write_data(reg_write_data)
    );

    // State machine for control
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= FETCH;
        else
            state <= next_state;
    end

    // Next state logic
    always @(*) begin
        case (state)
            FETCH:      next_state = DECODE;
            DECODE:     next_state = EXECUTE;
            EXECUTE:    begin
                if (Branch && branch_taken)
                    next_state = BRANCH_COMPLETION;
                else if (MemRead || MemWrite)
                    next_state = MEMORY;
                else
                    next_state = WRITEBACK;
            end
            MEMORY:     next_state = WRITEBACK;
            WRITEBACK:  next_state = FETCH;
            BRANCH_COMPLETION: next_state = FETCH;
            default:    next_state = FETCH;
        endcase
    end

    // Control signals based on current state
    always @(*) begin
        // Default control signals
        PCWrite = 1'b0;
        IRWrite = 1'b0;
        RegWrite = 1'b0;
        ALUSrcA = 1'b0;
        ALUSrcB = 2'b00;
        PCSrc = 2'b00;
        MemtoReg = 1'b0;
        IorD = 1'b0;

        case (state)
            FETCH: begin
                PCWrite = 1'b1;
                IRWrite = 1'b1;
                ALUSrcA = 1'b0;    // Use PC
                ALUSrcB = 2'b01;   // Use 4 (PC+4)
                PCSrc = 2'b00;     // Select ALU result (PC+4)
            end
            
            DECODE: begin
                ALUSrcA = 1'b0;    // Use PC
                ALUSrcB = 2'b10;   // Use immediate
            end
            
            EXECUTE: begin
                ALUSrcA = 1'b1;    // Use A
                if (ALUSrc)
                    ALUSrcB = 2'b10;  // Use immediate
                else
                    ALUSrcB = 2'b00;  // Use B
                
                if (Jump) begin
                    PCWrite = 1'b1;
                    PCSrc = 2'b01;    // Use branch_target
                end
            end
            
            MEMORY: begin
                IorD = 1'b1;       // Use ALUOut for address
            end
            
            WRITEBACK: begin
                RegWrite = 1'b1;
                MemtoReg = MemtoReg_ctrl;
            end
            
            BRANCH_COMPLETION: begin
                PCWrite = 1'b1;
                PCSrc = 2'b01;     // Use branch_target
            end
        endcase
    end

    // PC update
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 32'h0;
        end else if (PCWrite) begin
            case (PCSrc)
                2'b00: PC <= PC + 4;                // PC+4
                2'b01: PC <= branch_target;         // Branch/Jump target
                2'b10: PC <= ALU_result;            // Direct ALU result (for JALR)
                default: PC <= PC + 4;
            endcase
        end
    end

    // IR, A, B, ALUOut, and MDR updates
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IR <= 32'h0;
            A <= 32'h0;
            B <= 32'h0;
            ALUOut <= 32'h0;
            MDR <= 32'h0;
            
            // Initialize register file
            for (i = 0; i < 32; i = i + 1)
                RegisterFile[i] <= 32'h0;
        end else begin
            // IR update
            if (IRWrite)
                IR <= mem_data;
            
            // Register values update (during DECODE)
            if (state == DECODE) begin
                A <= RegisterFile[IR[19:15]]; // rs1
                B <= RegisterFile[IR[24:20]]; // rs2
            end
            
            // ALUOut update (during EXECUTE)
            if (state == EXECUTE)
                ALUOut <= ALU_result;
            
            // MDR update (during MEMORY)
            if (state == MEMORY && MemRead)
                MDR <= mem_data;
            
            // Register file write (during WRITEBACK)
            if (RegWrite && (IR[11:7] != 5'b0)) // Don't write to x0
                RegisterFile[IR[11:7]] <= reg_write_data;
        end
    end
endmodule