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



////////////////////////////////////////////////////////////////////////////////////////////////





module register_file(
    input clk,                          
    input rst,                          
    input [4:0] rs1_addr,               
    input [4:0] rs2_addr,               
    input [4:0] rd_addr,                
    input [63:0] rd_data,              
    input reg_write,                    
    output [63:0] rs1_data,             
    output [63:0] rs2_data              
);
    reg [63:0] registers [0:31];        

    
    integer i;
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 64'b0;
            end
        end else if (reg_write && rd_addr != 5'b0) begin
            registers[rd_addr] <= rd_data; 
        end
    end

    
    assign rs1_data = (rs1_addr == 5'b0) ? 64'b0 : registers[rs1_addr];
    assign rs2_data = (rs2_addr == 5'b0) ? 64'b0 : registers[rs2_addr];
endmodule

module instruction_memory(
    input [63:0] pc,                    // Program counter (address)
    output [31:0] instruction           // Output instruction
);
    reg [31:0] mem [0:1023];           // 1KB memory (1024 x 32-bit instructions)

     integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) begin
            mem[i] = 32'h00000000;
        end

        // Program: Add two numbers, store result, and branch
        mem[0] = 32'h00500093; // addi x1, x0, 5   (x1 = 5)
        mem[1] = 32'h00A00113; // addi x2, x0, 10  (x2 = 10)
        mem[2] = 32'h002081B3; // add x3, x1, x2   (x3 = x1 + x2)
        mem[3] = 32'h0041A233; // sw x4, 0(x3)     (Store x4 at address x3)
        mem[4] = 32'h00208463; // beq x1, x2, 8    (Branch if x1 == x2)
        mem[5] = 32'h004000EF; // jal x1, 16       (Jump and link to address 16)
        mem[6] = 32'h00008067; // jalr x0, 0(x1)   (Jump to address in x1)
        mem[7] = 32'h00000013; // nop              (No operation)

    end


    // Output the instruction at the address specified by the PC
    assign instruction = mem[pc[9:0]>>2]; // Use lower 10 bits of PC for 1KB memory

    
endmodule



module data_memory(
    input clk,                          
    input [63:0] address,               
    input [63:0] write_data,            
    input mem_read,                     
    input mem_write,                    
    output [63:0] read_data             
);
    reg [63:0] mem [0:1023];           

    
    integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) begin
            mem[i] <= 64'b0;
        end
    end

    
    assign read_data = (mem_read) ? mem[address[9:0]] : 64'b0;

    
    always @(posedge clk) begin
        if (mem_write) begin
            mem[address[9:0]] <= write_data;
        end
    end
endmodule



////////////////////////////////////////////////////////////////////////////////////////////////



module fetch(
    input clk,                          
    input rst,                          
    input stall,                        
    input [63:0] branch_target,         
    input branch_taken,                 
    output reg [63:0] pc,               
    output wire [31:0] instruction,      
    output reg instruction_valid        
);
    
    instruction_memory imem(
        .pc(pc),                        
        .instruction(instruction)       
    );

    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pc <= 64'b0;                
            instruction_valid <= 1'b0;  
        end else if (stall) begin
            
            pc <= pc;
            instruction_valid <= 1'b0;  
        end else if (branch_taken) begin
            
            pc <= branch_target;
            instruction_valid <= 1'b1;  
        end else begin
            
            pc <= pc + 64'd4;
            instruction_valid <= 1'b1; 
        end
    end
endmodule


module IF_ID (
    input clk,                          
    input rst,                          
    input stall,                        
    input flush,                        
    input [63:0] pc_in,                 
    input [31:0] instruction_in,        
    input instruction_valid_in,         
    output reg [63:0] pc_out,           
    output reg [31:0] instruction_out,  
    output reg instruction_valid_out    
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            
            pc_out <= 64'b0;
            instruction_out <= 32'b0;
            instruction_valid_out <= 1'b0;
        end else if (flush) begin
            
            pc_out <= 64'b0;
            instruction_out <= 32'b0;
            instruction_valid_out <= 1'b0;
        end else if (stall) begin
            
            pc_out <= pc_out;
            instruction_out <= instruction_out;
            instruction_valid_out <= instruction_valid_out;
        end else begin
            
            pc_out <= pc_in;
            instruction_out <= instruction_in;
            instruction_valid_out <= instruction_valid_in;
        end
    end
endmodule


////////////////////////////////////////////////////////////////////////////////////////////////


module decode(
    input clk,                          // Clock signal
    input rst,                          // Reset signal
    input [31:0] instruction,           // Fetched instruction
    input [63:0] pc,                    // Program counter
    input instruction_valid,            // Indicates if the instruction is valid
    input reg_write_back,               // Write-back signal from WB stage
    input [4:0] write_back_addr,        // Write-back address from WB stage
    input [63:0] write_back_data,       // Write-back data from WB stage
    output [63:0] rs1_data,             // Data from source register 1
    output [63:0] rs2_data,             // Data from source register 2
    output [63:0] imm,                  // Immediate value
    output [63:0] branch_target,        // Branch target address
    output mem_read,                    // Memory read signal
    output mem_write,                   // Memory write signal
    output reg_write,                   // Register write signal
    output [4:0] rs1_addr,              // Source register 1 address
    output [4:0] rs2_addr,              // Source register 2 address
    output [4:0] rd_addr,               // Destination register address
    output [2:0] funct3,                // Function code 3 (for ALU and memory operations)
    output [6:0] funct7,                // Function code 7 (for ALU operations)
    output [6:0] opcode,                // Opcode (added for instruction type)
    output alu_src,                     // ALU source select (0: rs2, 1: imm)
    output branch,                      // Branch instruction
    output jump,                        // Jump instruction
    output mem_to_reg,                  // Memory to register signal
    output [1:0] alu_op                 // ALU operation control
);
    // Extract fields from the instruction
    assign opcode = instruction[6:0];       // Opcode field
    assign rs1_addr = instruction[19:15];   // Source register 1 address
    assign rs2_addr = instruction[24:20];   // Source register 2 address
    assign rd_addr = instruction[11:7];     // Destination register address
    assign funct3 = instruction[14:12];     // Function code 3
    assign funct7 = instruction[31:25];     // Function code 7

    // Instantiate the register file
    register_file reg_file(
        .clk(clk),
        .rst(rst),
        .rs1_addr(rs1_addr),
        .rs2_addr(rs2_addr),
        .rd_addr(write_back_addr),
        .rd_data(write_back_data),
        .reg_write(reg_write_back),
        .rs1_data(rs1_data),
        .rs2_data(rs2_data)
    );

    // Immediate value extraction
    wire [63:0] imm_i = {{52{instruction[31]}}, instruction[31:20]}; // I-type immediate
    wire [63:0] imm_s = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type immediate
    wire [63:0] imm_b = {{51{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0}; // B-type immediate
    wire [63:0] imm_u = {instruction[31:12], 12'b0}; // U-type immediate
    wire [63:0] imm_j = {{43{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0}; // J-type immediate

    // Select immediate value based on instruction type
    assign imm = (opcode == 7'b0010011) ? imm_i : // I-type
                 (opcode == 7'b0100011) ? imm_s : // S-type
                 (opcode == 7'b1100011) ? imm_b : // B-type
                 (opcode == 7'b0110111 || opcode == 7'b0010111) ? imm_u : // U-type
                 (opcode == 7'b1101111) ? imm_j : // J-type
                 64'b0; // Default

    // Branch target calculation
    assign branch_target = pc + imm;

    // Control signals
    assign mem_read = (opcode == 7'b0000011); // Load instruction
    assign mem_write = (opcode == 7'b0100011); // Store instruction
    assign reg_write = (opcode == 7'b0110011 || opcode == 7'b0010011 || opcode == 7'b0000011 || opcode == 7'b1101111 || opcode == 7'b0010111); // R-type, I-type, Load, JAL, LUI
    assign alu_src = (opcode == 7'b0010011 || opcode == 7'b0100011 || opcode == 7'b0000011); // ALU source is immediate for I-type, S-type, and Load/Store
    assign branch = (opcode == 7'b1100011); // Branch instruction
    assign jump = (opcode == 7'b1101111 || opcode == 7'b1100111); // Jump instruction (JAL or JALR)
    assign mem_to_reg = (opcode == 7'b0000011); // Memory to register for Load instructions
    assign alu_op = (opcode == 7'b0110011) ? 2'b10 : // R-type: ALU operation based on funct3 and funct7
                    (opcode == 7'b0010011) ? 2'b11 : // I-type: ALU operation based on funct3
                    (opcode == 7'b1100011) ? 2'b01 : // Branch: ALU operation for comparison
                    2'b00; // Default: ALU operation for addition (e.g., address calculation)
endmodule


module id_ex_register(
    input clk,                          // Clock signal
    input rst,                          // Reset signal
    input stall,                        // Stall signal (freeze the register)
    input flush,                        // Flush signal (clear the register)
    input [63:0] pc_in,                 // Program counter from Decode stage
    input [63:0] rs1_data_in,           // Data from source register 1
    input [63:0] rs2_data_in,           // Data from source register 2
    input [63:0] imm_in,                // Immediate value
    input [63:0] branch_target_in,      // Branch target address
    input mem_read_in,                  // Memory read signal
    input mem_write_in,                 // Memory write signal
    input reg_write_in,                 // Register write signal
    input [4:0] rs1_addr_in,            // Source register 1 address
    input [4:0] rs2_addr_in,            // Source register 2 address
    input [4:0] rd_addr_in,             // Destination register address
    input [2:0] funct3_in,              // Function code 3 (for ALU and memory operations)
    input [6:0] funct7_in,              // Function code 7 (for ALU operations)
    input [6:0] opcode_in,              // Opcode (added for instruction type)
    input alu_src_in,                   // ALU source select (0: rs2, 1: imm)
    input branch_in,                    // Branch instruction
    input jump_in,                      // Jump instruction
    input mem_to_reg_in,                // Memory to register signal
    input [1:0] alu_op_in,              // ALU operation control
    output reg [63:0] pc_out,           // Program counter to Execute stage
    output reg [63:0] rs1_data_out,     // Data from source register 1 to Execute stage
    output reg [63:0] rs2_data_out,     // Data from source register 2 to Execute stage
    output reg [63:0] imm_out,          // Immediate value to Execute stage
    output reg [63:0] branch_target_out, // Branch target address to Execute stage
    output reg mem_read_out,            // Memory read signal to Execute stage
    output reg mem_write_out,           // Memory write signal to Execute stage
    output reg reg_write_out,           // Register write signal to Execute stage
    output reg [4:0] rs1_addr_out,      // Source register 1 address to Execute stage
    output reg [4:0] rs2_addr_out,      // Source register 2 address to Execute stage
    output reg [4:0] rd_addr_out,       // Destination register address to Execute stage
    output reg [2:0] funct3_out,        // Function code 3 to Execute stage
    output reg [6:0] funct7_out,        // Function code 7 to Execute stage
    output reg [6:0] opcode_out,        // Opcode to Execute stage
    output reg alu_src_out,             // ALU source select to Execute stage
    output reg branch_out,              // Branch instruction to Execute stage
    output reg jump_out,                // Jump instruction to Execute stage
    output reg mem_to_reg_out,          // Memory to register signal to Execute stage
    output reg [1:0] alu_op_out         // ALU operation control to Execute stage
);
    // Update the pipeline register on the rising edge of the clock
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset all outputs
            pc_out <= 64'b0;
            rs1_data_out <= 64'b0;
            rs2_data_out <= 64'b0;
            imm_out <= 64'b0;
            branch_target_out <= 64'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            rs1_addr_out <= 5'b0;
            rs2_addr_out <= 5'b0;
            rd_addr_out <= 5'b0;
            funct3_out <= 3'b0;
            funct7_out <= 7'b0;
            opcode_out <= 7'b0;
            alu_src_out <= 1'b0;
            branch_out <= 1'b0;
            jump_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            alu_op_out <= 2'b0;
        end else if (flush) begin
            // Flush all outputs
            pc_out <= 64'b0;
            rs1_data_out <= 64'b0;
            rs2_data_out <= 64'b0;
            imm_out <= 64'b0;
            branch_target_out <= 64'b0;
            mem_read_out <= 1'b0;
            mem_write_out <= 1'b0;
            reg_write_out <= 1'b0;
            rs1_addr_out <= 5'b0;
            rs2_addr_out <= 5'b0;
            rd_addr_out <= 5'b0;
            funct3_out <= 3'b0;
            funct7_out <= 7'b0;
            opcode_out <= 7'b0;
            alu_src_out <= 1'b0;
            branch_out <= 1'b0;
            jump_out <= 1'b0;
            mem_to_reg_out <= 1'b0;
            alu_op_out <= 2'b0;
        end else if (stall) begin
            // Stall: keep outputs unchanged
            pc_out <= pc_out;
            rs1_data_out <= rs1_data_out;
            rs2_data_out <= rs2_data_out;
            imm_out <= imm_out;
            branch_target_out <= branch_target_out;
            mem_read_out <= mem_read_out;
            mem_write_out <= mem_write_out;
            reg_write_out <= reg_write_out;
            rs1_addr_out <= rs1_addr_out;
            rs2_addr_out <= rs2_addr_out;
            rd_addr_out <= rd_addr_out;
            funct3_out <= funct3_out;
            funct7_out <= funct7_out;
            opcode_out <= opcode_out;
            alu_src_out <= alu_src_out;
            branch_out <= branch_out;
            jump_out <= jump_out;
            mem_to_reg_out <= mem_to_reg_out;
            alu_op_out <= alu_op_out;
        end else begin
            // Normal operation: pass inputs to outputs
            pc_out <= pc_in;
            rs1_data_out <= rs1_data_in;
            rs2_data_out <= rs2_data_in;
            imm_out <= imm_in;
            branch_target_out <= branch_target_in;
            mem_read_out <= mem_read_in;
            mem_write_out <= mem_write_in;
            reg_write_out <= reg_write_in;
            rs1_addr_out <= rs1_addr_in;
            rs2_addr_out <= rs2_addr_in;
            rd_addr_out <= rd_addr_in;
            funct3_out <= funct3_in;
            funct7_out <= funct7_in;
            opcode_out <= opcode_in;
            alu_src_out <= alu_src_in;
            branch_out <= branch_in;
            jump_out <= jump_in;
            mem_to_reg_out <= mem_to_reg_in;
            alu_op_out <= alu_op_in;
        end
    end
endmodule



////////////////////////////////////////////////////////////////////////////

module execute(
    input clk,                          // Clock signal
    input rst,                          // Reset signal
    input [63:0] pc_in,                 // Program counter from ID/EX register
    input [63:0] rs1_data,              // Data from source register 1
    input [63:0] rs2_data,              // Data from source register 2
    input [63:0] imm,                   // Immediate value
    input [63:0] branch_target,         // Branch target address
    input mem_read,                     // Memory read signal
    input mem_write,                    // Memory write signal
    input reg_write,                    // Register write signal
    input [4:0] rs1_addr,               // Source register 1 address
    input [4:0] rs2_addr,               // Source register 2 address
    input [4:0] rd_addr,                // Destination register address
    input [2:0] funct3,                 // Function code 3 (for ALU and memory operations)
    input [6:0] funct7,                 // Function code 7 (for ALU operations)
    input [6:0] opcode,                 // Opcode (added for instruction type)
    input alu_src,                      // ALU source select (0: rs2, 1: imm)
    input branch,                       // Branch instruction
    input jump,                         // Jump instruction
    input mem_to_reg,                   // Memory to register signal
    input [1:0] alu_op,                 // ALU operation control
    output wire [63:0] alu_result,      // ALU result
    output reg [63:0] mem_address,      // Memory address for load/store
    output reg [63:0] mem_write_data,   // Data to write to memory
    output reg branch_taken,            // Branch taken signal
    output reg [63:0] jump_target,      // Jump target address
    output reg reg_write_out,           // Register write signal (passed to MEM stage)
    output reg [4:0] rd_addr_out        // Destination register address (passed to MEM stage)
);
    // ALU source selection
    wire [63:0] alu_operand2 = alu_src ? imm : rs2_data;

    // Instantiate the ALU
    alu_64bit alu(
        .funct3(funct3),
        .funct7(funct7),
        .a(rs1_data),
        .b(alu_operand2),
        .result(alu_result)
    );

    // Initialize branch and jump signals
    initial begin 
        branch_taken = 1'b0;
        jump_target = 64'b0;
    end

    // Handle branching and jumping
    always @(*) begin
        branch_taken = 1'b0;
        jump_target = 64'b0;

        // Branch instructions (B-type)
        if (branch) begin
            case (funct3)
                3'b000: branch_taken = (rs1_data == rs2_data); // BEQ
                3'b001: branch_taken = (rs1_data != rs2_data); // BNE
                3'b100: branch_taken = ($signed(rs1_data) < $signed(rs2_data)); // BLT
                3'b101: branch_taken = ($signed(rs1_data) >= $signed(rs2_data)); // BGE
                3'b110: branch_taken = (rs1_data < rs2_data); // BLTU
                3'b111: branch_taken = (rs1_data >= rs2_data); // BGEU
            endcase
        end

        // Jump instructions (JAL, JALR)
        if (jump) begin
            if (opcode == 7'b1101111) begin // JAL
                jump_target = pc_in + imm;
                branch_taken = 1'b1;
            end else if (opcode == 7'b1100111 && funct3 == 3'b000) begin // JALR
                jump_target = rs1_data + imm;
                branch_taken = 1'b1;
            end
        end
    end

    // Handle memory address calculation and data preparation
    always @(*) begin
        mem_address = rs1_data + imm; // Base address + offset
        mem_write_data = rs2_data;    // Data to write to memory

        // Adjust memory address and data for different data sizes
        case (funct3)
            3'b000: mem_write_data = {{56{rs2_data[7]}}, rs2_data[7:0]}; // SB (byte)
            3'b001: mem_write_data = {{48{rs2_data[15]}}, rs2_data[15:0]}; // SH (half-word)
            3'b010: mem_write_data = {{32{rs2_data[31]}}, rs2_data[31:0]}; // SW (word)
            3'b011: mem_write_data = rs2_data; // SD (double-word)
            default: mem_write_data = rs2_data; // Default (double-word)
        endcase
    end

    // Pass control signals to the next stage
    always @(*) begin
        reg_write_out = reg_write;
        rd_addr_out = rd_addr;
    end
endmodule


module ex_mem_register(
    input clk,                          // Clock signal
    input rst,                          // Reset signal
    input stall,                        // Stall signal (freeze the register)
    input flush,                        // Flush signal (clear the register)
    input [63:0] alu_result_in,         // ALU result from Execute stage
    input [63:0] mem_address_in,        // Memory address from Execute stage
    input [63:0] mem_write_data_in,     // Data to write to memory from Execute stage
    input branch_taken_in,              // Branch taken signal from Execute stage
    input [63:0] jump_target_in,        // Jump target address from Execute stage
    input reg_write_in,                 // Register write signal from Execute stage
     input [2:0] funct3_in,             // Function code 3 from Execute stage
    input [6:0] funct7_in,              // Function code 7 from Execute stage
    output reg [2:0 ] funct3_out,       // Function code 3 to Memory stage
    output reg [6:0] funct7_out,        // Function code 7 to Memory stage
    input [4:0] rd_addr_in,             // Destination register address from Execute stage
    output reg [63:0] alu_result_out,   // ALU result to Memory stage
    output reg [63:0] mem_address_out,  // Memory address to Memory stage
    output reg [63:0] mem_write_data_out, // Data to write to memory to Memory stage
    output reg branch_taken_out,        // Branch taken signal to Memory stage
    output reg [63:0] jump_target_out,  // Jump target address to Memory stage
    output reg reg_write_out,           // Register write signal to Memory stage
    output reg [4:0] rd_addr_out        // Destination register address to Memory stage
);
    // Update the pipeline register on the rising edge of the clock
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset the pipeline register
            alu_result_out <= 64'b0;
            mem_address_out <= 64'b0;
            mem_write_data_out <= 64'b0;
            branch_taken_out <= 1'b0;
            jump_target_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (flush) begin
            // Flush the pipeline register (clear its contents)
            alu_result_out <= 64'b0;
            mem_address_out <= 64'b0;
            mem_write_data_out <= 64'b0;
            branch_taken_out <= 1'b0;
            jump_target_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (stall) begin
            // Stall: freeze the pipeline register (keep its contents)
            alu_result_out <= alu_result_out;
            mem_address_out <= mem_address_out;
            mem_write_data_out <= mem_write_data_out;
            branch_taken_out <= branch_taken_out;
            jump_target_out <= jump_target_out;
            reg_write_out <= reg_write_out;
            rd_addr_out <= rd_addr_out;
        end else begin
            // Normal operation: pass inputs to outputs
            alu_result_out <= alu_result_in;
            mem_address_out <= mem_address_in;
            mem_write_data_out <= mem_write_data_in;
            branch_taken_out <= branch_taken_in;
            jump_target_out <= jump_target_in;
            reg_write_out <= reg_write_in;
            rd_addr_out <= rd_addr_in;
        end
    end
endmodule




////////////////////////////////////////////////////////////////////////////////////////////////



module memory(
    input clk,                          // Clock signal
    input rst,                          // Reset signal
    input [63:0] alu_result,            // ALU result from EX/MEM register
    input [63:0] mem_address,           // Memory address from EX/MEM register
    input [63:0] mem_write_data,        // Data to write to memory from EX/MEM register
    input branch_taken,                 // Branch taken signal from EX/MEM register
    input [63:0] jump_target,           // Jump target address from EX/MEM register
    input reg_write,                    // Register write signal from EX/MEM register
    input [4:0] rd_addr,                // Destination register address from EX/MEM register
    input [2:0] funct3,                 // Function code 3 (for memory operations)
    output wire [63:0] mem_read_data,    // Data read from memory
    output reg [63:0] mem_result,       // Result to pass to WB stage (ALU result or memory read data)
    output reg reg_write_out,           // Register write signal (passed to WB stage)
    output reg [4:0] rd_addr_out        // Destination register address (passed to WB stage)
);
    // Instantiate the data memory
    data_memory dmem(
        .clk(clk),
        .address(mem_address),
        .write_data(mem_write_data),
        .mem_read(reg_write && (funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b011)), // Load instructions
        .mem_write(!reg_write && (funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b011)), // Store instructions
        .read_data(mem_read_data)
    );

    // Handle memory read data adjustment based on data size
    always @(*) begin
        case (funct3)
            3'b000: mem_result = {{56{mem_read_data[7]}}, mem_read_data[7:0]}; // LB (byte)
            3'b001: mem_result = {{48{mem_read_data[15]}}, mem_read_data[15:0]}; // LH (half-word)
            3'b010: mem_result = {{32{mem_read_data[31]}}, mem_read_data[31:0]}; // LW (word)
            3'b011: mem_result = mem_read_data; // LD (double-word)
            3'b100: mem_result = {56'b0, mem_read_data[7:0]}; // LBU (unsigned byte)
            3'b101: mem_result = {48'b0, mem_read_data[15:0]}; // LHU (unsigned half-word)
            3'b110: mem_result = {32'b0, mem_read_data[31:0]}; // LWU (unsigned word)
            default: mem_result = alu_result; // Default (ALU result)
        endcase
    end

    // Pass control signals to the next stage
    always @(*) begin
        reg_write_out = reg_write;
        rd_addr_out = rd_addr;
    end
endmodule




module mem_wb_register(
    input clk,                          // Clock signal
    input rst,                          // Reset signal
    input stall,                        // Stall signal (freeze the register)
    input flush,                        // Flush signal (clear the register)
    input [63:0] mem_result_in,         // Result from Memory stage
    input reg_write_in,                 // Register write signal from Memory stage
    input [4:0] rd_addr_in,             // Destination register address from Memory stage
    output reg [63:0] mem_result_out,   // Result to Writeback stage
    output reg reg_write_out,           // Register write signal to Writeback stage
    output reg [4:0] rd_addr_out        // Destination register address to Writeback stage
);
    // Update the pipeline register on the rising edge of the clock
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Reset the pipeline register
            mem_result_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (flush) begin
            // Flush the pipeline register (clear its contents)
            mem_result_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (stall) begin
            // Stall: freeze the pipeline register (keep its contents)
            mem_result_out <= mem_result_out;
            reg_write_out <= reg_write_out;
            rd_addr_out <= rd_addr_out;
        end else begin
            // Normal operation: pass inputs to outputs
            mem_result_out <= mem_result_in;
            reg_write_out <= reg_write_in;
            rd_addr_out <= rd_addr_in;
        end
    end
endmodule


module writeback(
    input [63:0] mem_result,            // Result from MEM/WB register (ALU result or memory read data)
    input reg_write,                    // Register write signal from MEM/WB register
    input [4:0] rd_addr,                // Destination register address from MEM/WB register
    output [63:0] write_back_data,      // Data to write back to the register file
    output [4:0] write_back_addr,       // Destination register address for write-back
    output reg_write_back               // Register write signal for write-back
);
    // Assign outputs
    assign write_back_data = mem_result; // Data to write back is the result from MEM/WB register
    assign write_back_addr = rd_addr;    // Destination register address
    assign reg_write_back = reg_write;   // Register write signal
endmodule


///////////////////////////////////////////////


module hazard_detection_unit(
    input [4:0] id_ex_rs1_addr,        // Source register 1 address from ID/EX register
    input [4:0] id_ex_rs2_addr,        // Source register 2 address from ID/EX register
    input [4:0] ex_mem_rd_addr,        // Destination register address from EX/MEM register
    input [4:0] mem_wb_rd_addr,        // Destination register address from MEM/WB register
    input id_ex_mem_read,              // Memory read signal from ID/EX register
    output reg stall                   // Stall signal
);

    initial begin 
        stall = 1'b0;
    end

    always @(*) begin
        // Stall if there is a data hazard (RAW hazard)
        if (id_ex_mem_read && ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr))) begin
            stall = 1'b1;
        end else begin
            stall = 1'b0;
        end
    end
endmodule










/////////////////////////////// main impelmentation /////////////////////////////////





module riscv_processor(
    input clk,                          // Clock signal
    input rst                           // Reset signal
);
    // Pipeline registers - Fetch stage outputs
    wire [63:0] if_pc;                  // PC from fetch stage
    wire [31:0] if_instruction;         // Instruction from fetch stage
    wire if_instruction_valid;          // Instruction valid signal from fetch stage
    
    // IF/ID pipeline register outputs
    wire [63:0] if_id_pc;               // PC from IF/ID register
    wire [31:0] if_id_instruction;      // Instruction from IF/ID register
    wire if_id_instruction_valid;       // Instruction valid from IF/ID register
    
    // Decode stage outputs
    wire [63:0] id_rs1_data, id_rs2_data, id_imm, id_branch_target;
    wire id_mem_read, id_mem_write, id_reg_write;
    wire [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr;
    wire [2:0] id_funct3;
    wire [6:0] id_funct7;
    
    // ID/EX pipeline register outputs
    wire [63:0] id_ex_pc;               // PC from ID/EX register
    wire [63:0] id_ex_rs1_data;         // RS1 data from ID/EX register
    wire [63:0] id_ex_rs2_data;         // RS2 data from ID/EX register
    wire [63:0] id_ex_imm;              // Immediate value from ID/EX register
    wire [63:0] id_ex_branch_target;    // Branch target from ID/EX register
    wire id_ex_mem_read;                // Memory read signal from ID/EX register
    wire id_ex_mem_write;               // Memory write signal from ID/EX register
    wire id_ex_reg_write;               // Register write signal from ID/EX register
    wire [4:0] id_ex_rs1_addr;          // RS1 address from ID/EX register
    wire [4:0] id_ex_rs2_addr;          // RS2 address from ID/EX register
    wire [4:0] id_ex_rd_addr;           // RD address from ID/EX register
    wire [2:0] id_ex_funct3;            // Function3 from ID/EX register
    wire [6:0] id_ex_funct7;            // Function7 from ID/EX register
    
    // Execute stage outputs
    wire [63:0] ex_alu_result;          // ALU result from execute stage
    wire [63:0] ex_mem_address;         // Memory address from execute stage
    wire [63:0] ex_mem_write_data;      // Memory write data from execute stage
    wire ex_branch_taken;               // Branch taken signal from execute stage
    wire [63:0] ex_branch_target;       // Branch target from execute stage
    wire ex_reg_write;                  // Register write signal from execute stage
    wire [4:0] ex_rd_addr;              // Destination register from execute stage
    
    // EX/MEM pipeline register outputs
    wire [63:0] ex_mem_alu_result;      // ALU result from EX/MEM register
    wire [63:0] ex_mem_mem_address;     // Memory address from EX/MEM register
    wire [63:0] ex_mem_mem_write_data;  // Memory write data from EX/MEM register
    wire ex_mem_branch_taken;           // Branch taken from EX/MEM register
    wire [63:0] ex_mem_branch_target;   // Branch target from EX/MEM register
    wire ex_mem_reg_write;              // Register write from EX/MEM register
    wire [4:0] ex_mem_rd_addr;          // Destination register from EX/MEM register
    wire [2:0] ex_mem_funct3;           // Function3 from EX/MEM register
    
    // Memory stage outputs
    wire [63:0] mem_result;             // Result from memory stage
    wire mem_reg_write;                 // Register write from memory stage
    wire [4:0] mem_rd_addr;             // Destination register from memory stage
    
    // MEM/WB pipeline register outputs
    wire [63:0] mem_wb_mem_result;      // Memory result from MEM/WB register
    wire mem_wb_reg_write;              // Register write from MEM/WB register
    wire [4:0] mem_wb_rd_addr;          // Destination register from MEM/WB register
    
    // Writeback outputs
    wire [63:0] write_back_data;        // Data to write back to register file
    wire [4:0] write_back_addr;         // Address to write back to register file
    wire write_back_enable;             // Write enable for register file
    
    // Control signals
    wire stall;                         // Stall signal from hazard detection unit
    wire flush;                         // Flush signal for pipeline registers
    
    // Branch control signals (from EX/MEM to fetch)
    wire branch_taken;                  // Branch taken signal to fetch stage
    wire [63:0] branch_target;          // Branch target to fetch stage
    
    // Connect branch control signals from EX/MEM register to fetch stage
    assign branch_taken = ex_mem_branch_taken;
    assign branch_target = ex_mem_branch_target;
    
    // Generate flush signal when branch is taken
    assign flush = branch_taken;
    
    // Instantiate the Hazard Detection Unit
    hazard_detection_unit hdu(
        .id_ex_rs1_addr(id_ex_rs1_addr),
        .id_ex_rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .id_ex_mem_read(id_ex_mem_read),
        .stall(stall)
    );
    
    // Instantiate the Fetch stage
    fetch fetch_stage(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .branch_target(branch_target),
        .branch_taken(branch_taken),
        .pc(if_pc),
        .instruction(if_instruction),
        .instruction_valid(if_instruction_valid)
    );
    
    // Instantiate the IF/ID Pipeline Register
    IF_ID if_id_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .pc_in(if_pc),
        .instruction_in(if_instruction),
        .instruction_valid_in(if_instruction_valid),
        .pc_out(if_id_pc),
        .instruction_out(if_id_instruction),
        .instruction_valid_out(if_id_instruction_valid)
    );
    
    // Instantiate the Decode stage
    decode decode_stage(
        .clk(clk),
        .rst(rst),
        .instruction(if_id_instruction),
        .pc(if_id_pc),
        .instruction_valid(if_id_instruction_valid),
        .reg_write_back(write_back_enable),
        .write_back_addr(write_back_addr),
        .write_back_data(write_back_data),
        .rs1_data(id_rs1_data),              // Fixed output connection
        .rs2_data(id_rs2_data),              // Fixed output connection
        .imm(id_imm),                        // Fixed output connection
        .branch_target(id_branch_target),    // Fixed output connection
        .mem_read(id_mem_read),              // Fixed output connection
        .mem_write(id_mem_write),            // Fixed output connection
        .reg_write(id_reg_write),            // Fixed output connection
        .rs1_addr(id_rs1_addr),              // Fixed output connection
        .rs2_addr(id_rs2_addr),              // Fixed output connection
        .rd_addr(id_rd_addr),                // Fixed output connection
        .funct3(id_funct3),                  // Fixed output connection
        .funct7(id_funct7)                   // Fixed output connection
    );
    
    // Instantiate the ID/EX Pipeline Register
    id_ex_register id_ex_registerr(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .pc_in(if_id_pc),
        .rs1_data_in(id_rs1_data),           // Fixed input connection
        .rs2_data_in(id_rs2_data),           // Fixed input connection
        .imm_in(id_imm),                     // Fixed input connection
        .branch_target_in(id_branch_target), // Fixed input connection
        .mem_read_in(id_mem_read),           // Fixed input connection
        .mem_write_in(id_mem_write),         // Fixed input connection
        .reg_write_in(id_reg_write),         // Fixed input connection
        .rs1_addr_in(id_rs1_addr),           // Fixed input connection
        .rs2_addr_in(id_rs2_addr),           // Fixed input connection
        .rd_addr_in(id_rd_addr),             // Fixed input connection
        .funct3_in(id_funct3),               // Fixed input connection
        .funct7_in(id_funct7),               // Fixed input connection
        .pc_out(id_ex_pc),
        .rs1_data_out(id_ex_rs1_data),
        .rs2_data_out(id_ex_rs2_data),
        .imm_out(id_ex_imm),
        .branch_target_out(id_ex_branch_target),
        .mem_read_out(id_ex_mem_read),
        .mem_write_out(id_ex_mem_write),
        .reg_write_out(id_ex_reg_write),
        .rs1_addr_out(id_ex_rs1_addr),
        .rs2_addr_out(id_ex_rs2_addr),
        .rd_addr_out(id_ex_rd_addr),
        .funct3_out(id_ex_funct3),
        .funct7_out(id_ex_funct7)
    );
    
    // Update the execute stage instance with the fixed connections
    execute execute_stage(
        .clk(clk),
        .rst(rst),
        .pc_in(id_ex_pc),
        .rs1_data(id_ex_rs1_data),
        .rs2_data(id_ex_rs2_data),
        .imm(id_ex_imm),
        .branch_target(id_ex_branch_target),
        .mem_read(id_ex_mem_read),
        .mem_write(id_ex_mem_write),
        .reg_write(id_ex_reg_write),
        .rs1_addr(id_ex_rs1_addr),
        .rs2_addr(id_ex_rs2_addr),
        .rd_addr(id_ex_rd_addr),
        .funct3(id_ex_funct3),
        .funct7(id_ex_funct7),
        .alu_result(ex_alu_result),           // Fixed output connection
        .mem_address(ex_mem_address),         // Fixed output connection
        .mem_write_data(ex_mem_write_data),   // Fixed output connection
        .branch_taken(ex_branch_taken),       // Fixed output connection
        .jump_target(ex_branch_target),       // Fixed output connection
        .reg_write_out(ex_reg_write),         // Fixed output connection
        .rd_addr_out(ex_rd_addr)              // Fixed output connection
    );
    
    // Pass funct3 directly from ID/EX to EX/MEM
    // This assumes execute stage doesn't change funct3
    
    // Update the EX/MEM register instance with the fixed connections
    ex_mem_register ex_mem_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .alu_result_in(ex_alu_result),        // Fixed input connection
        .mem_address_in(ex_mem_address),      // Fixed input connection
        .mem_write_data_in(ex_mem_write_data),// Fixed input connection
        .branch_taken_in(ex_branch_taken),    // Fixed input connection
        .jump_target_in(ex_branch_target),    // Fixed input connection
        .reg_write_in(ex_reg_write),          // Fixed input connection
        .rd_addr_in(ex_rd_addr),              // Fixed input connection
        .funct3_in(id_ex_funct3),             // Direct connection from ID/EX
        .alu_result_out(ex_mem_alu_result),
        .mem_address_out(ex_mem_mem_address),
        .mem_write_data_out(ex_mem_mem_write_data),
        .branch_taken_out(ex_mem_branch_taken),
        .jump_target_out(ex_mem_branch_target),
        .reg_write_out(ex_mem_reg_write),
        .rd_addr_out(ex_mem_rd_addr),
        .funct3_out(ex_mem_funct3)
    );
    
    // Instantiate the Memory stage
    memory memory_stage(
        .clk(clk),
        .rst(rst),
        .alu_result(ex_mem_alu_result),
        .mem_address(ex_mem_mem_address),
        .mem_write_data(ex_mem_mem_write_data),
        .branch_taken(ex_mem_branch_taken),
        .jump_target(ex_mem_branch_target),   // Fixed to use branch_target for consistency
        .reg_write(ex_mem_reg_write),
        .rd_addr(ex_mem_rd_addr),
        .funct3(ex_mem_funct3),
        .mem_read_data(mem_result),           // Fixed: separate from mem_result
        .mem_result(mem_result),              // Fixed output connection
        .reg_write_out(mem_reg_write),        // Fixed output connection
        .rd_addr_out(mem_rd_addr)             // Fixed output connection
    );
    
    // Instantiate the MEM/WB Pipeline Register
    mem_wb_register mem_wb_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .mem_result_in(mem_result),           // Fixed input connection
        .reg_write_in(mem_reg_write),         // Fixed input connection
        .rd_addr_in(mem_rd_addr),             // Fixed input connection
        .mem_result_out(mem_wb_mem_result),
        .reg_write_out(mem_wb_reg_write),
        .rd_addr_out(mem_wb_rd_addr)
    );
    
    // Instantiate the Writeback stage
    writeback writeback_stage(
        .mem_result(mem_wb_mem_result),
        .reg_write(mem_wb_reg_write),
        .rd_addr(mem_wb_rd_addr),
        .write_back_data(write_back_data),
        .write_back_addr(write_back_addr),
        .reg_write_back(write_back_enable)
    );

endmodule