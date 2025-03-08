//Pipeline processor Wrapper


module riscv_processor(
    input clk,                          
    input rst                           
);
    
    // IF stage signals
    wire [63:0] if_pc;                  
    wire [31:0] if_instruction;         
    wire if_instruction_valid;          
    
    // IF/ID pipeline register signals
    wire [63:0] if_id_pc;               
    wire [31:0] if_id_instruction;      
    wire if_id_instruction_valid;       
    
    // ID stage signals
    wire [63:0] id_rs1_data, id_rs2_data, id_imm, id_branch_target;
    wire id_alu_src, id_branch, id_jump, id_mem_to_reg;
    wire [1:0] id_alu_op;
    wire id_mem_read, id_mem_write, id_reg_write;
    wire [4:0] id_rs1_addr, id_rs2_addr, id_rd_addr;
    wire [2:0] id_funct3;
    wire [6:0] id_funct7, id_opcode;
    
    // ID/EX pipeline register signals
    wire [63:0] id_ex_pc;               
    wire [63:0] id_ex_rs1_data;         
    wire [63:0] id_ex_rs2_data;         
    wire [63:0] id_ex_imm;              
    wire [63:0] id_ex_branch_target;    
    wire id_ex_mem_read;                
    wire id_ex_mem_write;               
    wire id_ex_reg_write;               
    wire [4:0] id_ex_rs1_addr;          
    wire [4:0] id_ex_rs2_addr;          
    wire [4:0] id_ex_rd_addr;           
    wire [2:0] id_ex_funct3;            
    wire [6:0] id_ex_funct7;
    wire [6:0] id_ex_opcode;            
    wire id_ex_alu_src, id_ex_branch, id_ex_jump, id_ex_mem_to_reg;
    wire [1:0] id_ex_alu_op;        
    
    // EX stage signals
    wire [63:0] ex_alu_result;          
    wire [63:0] ex_mem_address;         
    wire [63:0] ex_mem_write_data;      
    wire ex_branch_taken;               
    wire [63:0] ex_jump_target;       
    wire ex_reg_write;                  
    wire [4:0] ex_rd_addr;              
    
    // EX/MEM pipeline register signals
    wire [63:0] ex_mem_alu_result;      
    wire [63:0] ex_mem_mem_address;     
    wire [63:0] ex_mem_mem_write_data;  
    wire ex_mem_branch_taken;           
    wire [63:0] ex_mem_jump_target;   
    wire ex_mem_reg_write;              
    wire [4:0] ex_mem_rd_addr;          
    wire [2:0] ex_mem_funct3;
    wire [6:0] ex_mem_funct7;           
    
    // MEM stage signals
    wire [63:0] mem_read_data;
    wire [63:0] mem_result;             
    wire mem_reg_write;                 
    wire [4:0] mem_rd_addr;             
    
    // MEM/WB pipeline register signals
    wire [63:0] mem_wb_mem_result;      
    wire mem_wb_reg_write;              
    wire [4:0] mem_wb_rd_addr;          
    
    // WB stage signals
    wire [63:0] write_back_data;        
    wire [4:0] write_back_addr;         
    wire write_back_enable;             
    
    // Control signals
    wire stall;                         
    wire flush;                         
    
    // Branch signals
    wire branch_taken;                  
    wire [63:0] branch_target;          
    
    // Connect branch signals
    assign branch_taken = ex_mem_branch_taken;
    assign branch_target = ex_mem_jump_target;
    
    // Flush on branch taken
    assign flush = branch_taken;
    
    // Hazard detection unit
    hazard_detection_unit hdu(
        .id_ex_rs1_addr(id_ex_rs1_addr),
        .id_ex_rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .id_ex_mem_read(id_ex_mem_read),
        .stall(stall)
    );
    
    // Fetch stage
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
    
    // IF/ID pipeline register
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
    
    // Decode stage
    decode decode_stage(
        .clk(clk),
        .rst(rst),
        .instruction(if_id_instruction),
        .pc(if_id_pc),
        .instruction_valid(if_id_instruction_valid),
        .reg_write_back(write_back_enable),
        .write_back_addr(write_back_addr),
        .write_back_data(write_back_data),
        .rs1_data(id_rs1_data),              
        .rs2_data(id_rs2_data),              
        .imm(id_imm),                        
        .branch_target(id_branch_target),    
        .mem_read(id_mem_read),              
        .mem_write(id_mem_write),            
        .reg_write(id_reg_write),            
        .rs1_addr(id_rs1_addr),              
        .rs2_addr(id_rs2_addr),              
        .rd_addr(id_rd_addr),                
        .funct3(id_funct3),                  
        .funct7(id_funct7),
        .opcode(id_opcode),
        .alu_src(id_alu_src),
        .branch(id_branch),
        .jump(id_jump),
        .mem_to_reg(id_mem_to_reg),
        .alu_op(id_alu_op)            
    );
    
    // ID/EX pipeline register
    id_ex_register id_ex_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .pc_in(if_id_pc),
        .rs1_data_in(id_rs1_data),           
        .rs2_data_in(id_rs2_data),           
        .imm_in(id_imm),                     
        .branch_target_in(id_branch_target), 
        .mem_read_in(id_mem_read),           
        .mem_write_in(id_mem_write),         
        .reg_write_in(id_reg_write),         
        .rs1_addr_in(id_rs1_addr),           
        .rs2_addr_in(id_rs2_addr),           
        .rd_addr_in(id_rd_addr),             
        .funct3_in(id_funct3),               
        .funct7_in(id_funct7),
        .opcode_in(id_opcode),
        .alu_src_in(id_alu_src),
        .branch_in(id_branch),
        .jump_in(id_jump),
        .mem_to_reg_in(id_mem_to_reg),
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
        .funct7_out(id_ex_funct7),
        .opcode_out(id_ex_opcode),
        .alu_src_out(id_ex_alu_src),
        .branch_out(id_ex_branch),
        .jump_out(id_ex_jump),
        .mem_to_reg_out(id_ex_mem_to_reg)
    );
    
    // Execute stage
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
        .opcode(id_ex_opcode),
        .alu_src(id_ex_alu_src),
        .branch(id_ex_branch),
        .jump(id_ex_jump),
        .mem_to_reg(id_ex_mem_to_reg),
        .alu_result(ex_alu_result),           
        .mem_address(ex_mem_address),         
        .mem_write_data(ex_mem_write_data),   
        .branch_taken(ex_branch_taken),       
        .jump_target(ex_jump_target),       
        .reg_write_out(ex_reg_write),         
        .rd_addr_out(ex_rd_addr)
    );
    
    // EX/MEM pipeline register
    ex_mem_register ex_mem_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .alu_result_in(ex_alu_result),        
        .mem_address_in(ex_mem_address),      
        .mem_write_data_in(ex_mem_write_data),
        .branch_taken_in(ex_branch_taken),    
        .jump_target_in(ex_jump_target),    
        .reg_write_in(ex_reg_write),          
        .rd_addr_in(ex_rd_addr),              
        .funct3_in(id_ex_funct3),
        .funct7_in(id_ex_funct7),             
        .alu_result_out(ex_mem_alu_result),
        .mem_address_out(ex_mem_mem_address),
        .mem_write_data_out(ex_mem_mem_write_data),
        .branch_taken_out(ex_mem_branch_taken),
        .jump_target_out(ex_mem_jump_target),
        .reg_write_out(ex_mem_reg_write),
        .rd_addr_out(ex_mem_rd_addr),
        .funct3_out(ex_mem_funct3),
        .funct7_out(ex_mem_funct7)
    );
    
    // Memory stage
    memory memory_stage(
        .clk(clk),
        .rst(rst),
        .alu_result(ex_mem_alu_result),
        .mem_address(ex_mem_mem_address),
        .mem_write_data(ex_mem_mem_write_data),
        .branch_taken(ex_mem_branch_taken),
        .jump_target(ex_mem_jump_target),   
        .reg_write(ex_mem_reg_write),
        .rd_addr(ex_mem_rd_addr),
        .funct3(ex_mem_funct3),
        .mem_read_data(mem_read_data),           
        .mem_result(mem_result),              
        .reg_write_out(mem_reg_write),        
        .rd_addr_out(mem_rd_addr)             
    );
    
    // MEM/WB pipeline register
    mem_wb_register mem_wb_register(
        .clk(clk),
        .rst(rst),
        .stall(stall),
        .flush(flush),
        .mem_result_in(mem_result),           
        .reg_write_in(mem_reg_write),         
        .rd_addr_in(mem_rd_addr),             
        .mem_result_out(mem_wb_mem_result),
        .reg_write_out(mem_wb_reg_write),
        .rd_addr_out(mem_wb_rd_addr)
    );
    
    // Writeback stage
    writeback writeback_stage(
        .mem_result(mem_wb_mem_result),
        .reg_write(mem_wb_reg_write),
        .rd_addr(mem_wb_rd_addr),
        .write_back_data(write_back_data),
        .write_back_addr(write_back_addr),
        .reg_write_back(write_back_enable)
    );
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


module decode(
    input clk,
    input rst,
    input [31:0] instruction,
    input [63:0] pc,
    input instruction_valid,
    input reg_write_back,
    input [4:0] write_back_addr,
    input [63:0] write_back_data,
    output [63:0] rs1_data,
    output [63:0] rs2_data,
    output [63:0] imm,
    output [63:0] branch_target,
    output mem_read,
    output mem_write,
    output reg_write,
    output [4:0] rs1_addr,
    output [4:0] rs2_addr,
    output [4:0] rd_addr,
    output [2:0] funct3,
    output [6:0] funct7,
    output [6:0] opcode,
    output alu_src,
    output branch,
    output jump,
    output mem_to_reg,
    output [1:0] alu_op
);
    assign opcode = instruction[6:0];
    assign rs1_addr = instruction[19:15];
    assign rs2_addr = instruction[24:20];
    assign rd_addr = instruction[11:7];
    assign funct3 = instruction[14:12];
    assign funct7 = instruction[31:25];

    // Immediate value extraction
    wire [63:0] imm_i = {{52{instruction[31]}}, instruction[31:20]}; // I-type (lw)
    wire [63:0] imm_s = {{52{instruction[31]}}, instruction[31:25], instruction[11:7]}; // S-type (sw)
    wire [63:0] imm_b = {{51{instruction[31]}}, instruction[31], instruction[7], instruction[30:25], instruction[11:8], 1'b0};
    wire [63:0] imm_u = {{32{instruction[31]}}, instruction[31:12], 12'b0};
    wire [63:0] imm_j = {{43{instruction[31]}}, instruction[31], instruction[19:12], instruction[20], instruction[30:21], 1'b0};

    reg [63:0] imm_reg;
    always @(*) begin
        case (opcode)
            7'b0000011: imm_reg = imm_i; // lw (I-type)
            7'b0100011: imm_reg = imm_s; // sw (S-type)
            default:    imm_reg = 64'b0;
        endcase
    end
    assign imm = imm_reg;

    // Branch target calculation
    assign branch_target = pc + imm;

    // Control signals
    assign mem_read = (opcode == 7'b0000011); // lw
    assign mem_write = (opcode == 7'b0100011); // sw
    assign reg_write = (opcode == 7'b0000011); // lw writes to register
    assign alu_src = (opcode == 7'b0000011 || opcode == 7'b0100011); // lw/sw use immediate
    assign branch = (opcode == 7'b1100011); // branch instructions
    assign jump = (opcode == 7'b1101111 || opcode == 7'b1100111); // jal/jalr
    assign mem_to_reg = (opcode == 7'b0000011); // lw writes memory data to register
    assign alu_op = (opcode == 7'b0110011) ? 2'b10 : // R-type
                    (opcode == 7'b0000011 || opcode == 7'b0100011) ? 2'b00 : // lw/sw
                    2'b00; // Default

    // Register file
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
endmodule


module id_ex_register(
    input clk,                          
    input rst,                          
    input stall,                        
    input flush,                        
    input [63:0] pc_in,                 
    input [63:0] rs1_data_in,           
    input [63:0] rs2_data_in,           
    input [63:0] imm_in,                
    input [63:0] branch_target_in,      
    input mem_read_in,                  
    input mem_write_in,                 
    input reg_write_in,                 
    input [4:0] rs1_addr_in,            
    input [4:0] rs2_addr_in,            
    input [4:0] rd_addr_in,             
    input [2:0] funct3_in,              
    input [6:0] funct7_in,              
    input [6:0] opcode_in,              
    input alu_src_in,                   
    input branch_in,                    
    input jump_in,                      
    input mem_to_reg_in,                
    output reg [63:0] pc_out,           
    output reg [63:0] rs1_data_out,     
    output reg [63:0] rs2_data_out,     
    output reg [63:0] imm_out,          
    output reg [63:0] branch_target_out, 
    output reg mem_read_out,            
    output reg mem_write_out,           
    output reg reg_write_out,           
    output reg [4:0] rs1_addr_out,      
    output reg [4:0] rs2_addr_out,      
    output reg [4:0] rd_addr_out,       
    output reg [2:0] funct3_out,        
    output reg [6:0] funct7_out,        
    output reg [6:0] opcode_out,        
    output reg alu_src_out,             
    output reg branch_out,              
    output reg jump_out,                
    output reg mem_to_reg_out   
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            
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
        end else if (flush) begin
            
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
        end else if (stall) begin
            
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
        end else begin
            
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
        end
    end
endmodule


module execute(
    input clk,
    input rst,
    input [63:0] pc_in,
    input [63:0] rs1_data,
    input [63:0] rs2_data,
    input [63:0] imm,
    input [63:0] branch_target,
    input mem_read,
    input mem_write,
    input reg_write,
    input [4:0] rs1_addr,
    input [4:0] rs2_addr,
    input [4:0] rd_addr,
    input [2:0] funct3,
    input [6:0] funct7,
    input [6:0] opcode,
    input alu_src,
    input branch,
    input jump,
    input mem_to_reg,
    output wire [63:0] alu_result,
    output wire [63:0] mem_address,
    output wire [63:0] mem_write_data,
    output wire branch_taken,
    output wire [63:0] jump_target,
    output wire reg_write_out,
    output wire [4:0] rd_addr_out
);
    // ALU operand selection
    wire [63:0] alu_operand2 = alu_src ? imm : rs2_data;

    // ALU instantiation
    alu_64bit alu(
        .funct3(funct3),
        .funct7(funct7),
        .a(rs1_data),
        .b(alu_operand2),
        .result(alu_result)
    );

    // Memory address calculation for lw/sw
    assign mem_address = rs1_data + imm;

    // Memory write data for sw
    assign mem_write_data = rs2_data;

    // Branch and jump logic (unchanged)
    assign branch_taken = branch && (funct3 == 3'b000 && rs1_data == rs2_data); // Example: beq
    assign jump_target = jump ? (pc_in + imm) : 64'b0;

    // Pipeline signals
    assign reg_write_out = reg_write;
    assign rd_addr_out = rd_addr;
endmodule

module ex_mem_register(
    input clk,
    input rst,
    input stall,
    input flush,
    input [63:0] alu_result_in,
    input [63:0] mem_address_in,
    input [63:0] mem_write_data_in,
    input branch_taken_in,
    input [63:0] jump_target_in,
    input reg_write_in,
    input [2:0] funct3_in,
    input [6:0] funct7_in,
    input [4:0] rd_addr_in,
    output reg [63:0] alu_result_out,
    output reg [63:0] mem_address_out,
    output reg [63:0] mem_write_data_out,
    output reg branch_taken_out,
    output reg [63:0] jump_target_out,
    output reg reg_write_out,
    output reg [4:0] rd_addr_out
);
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            alu_result_out <= 64'b0;
            mem_address_out <= 64'b0;
            mem_write_data_out <= 64'b0;
            branch_taken_out <= 1'b0;
            jump_target_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (!stall && !flush) begin
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
module hazard_detection_unit(
    input [4:0] id_ex_rs1_addr,        
    input [4:0] id_ex_rs2_addr,        
    input [4:0] ex_mem_rd_addr,        
    input [4:0] mem_wb_rd_addr,        
    input id_ex_mem_read,              
    output reg stall                   
);

    initial begin 
        stall = 1'b0;
    end

    always @(*) begin
        
        if (id_ex_mem_read && ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr))) begin
            stall = 1'b1;
        end else begin
            stall = 1'b0;
        end
    end
endmodule


module memory(
    input clk,
    input rst,
    input [63:0] alu_result,
    input [63:0] mem_address,
    input [63:0] mem_write_data,
    input branch_taken,
    input [63:0] jump_target,
    input reg_write,
    input [4:0] rd_addr,
    input [2:0] funct3,
    output wire [63:0] mem_read_data,
    output reg [63:0] mem_result,
    output reg reg_write_out,
    output reg [4:0] rd_addr_out
);
    // Data memory instantiation
    data_memory dmem(
        .clk(clk),
        .address(mem_address),
        .write_data(mem_write_data),
        .mem_read(reg_write && (funct3 == 3'b010)), // lw (funct3 = 010 for word)
        .mem_write(!reg_write && (funct3 == 3'b010)), // sw (funct3 = 010 for word)
        .read_data(mem_read_data)
    );

    // Memory result selection
    always @(*) begin
        if (reg_write) begin
            mem_result = mem_read_data; // lw: write memory data to register
        end else begin
            mem_result = alu_result; // Default: use ALU result
        end
    end

    // Pipeline signals
    always @(*) begin
        reg_write_out = reg_write;
        rd_addr_out = rd_addr;
    end
endmodule

module mem_wb_register(
    input clk,                          
    input rst,                          
    input stall,                        
    input flush,                        
    input [63:0] mem_result_in,         
    input reg_write_in,                 
    input [4:0] rd_addr_in,             
    output reg [63:0] mem_result_out,   
    output reg reg_write_out,           
    output reg [4:0] rd_addr_out        
);
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            
            mem_result_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (flush) begin
            
            mem_result_out <= 64'b0;
            reg_write_out <= 1'b0;
            rd_addr_out <= 5'b0;
        end else if (stall) begin
            
            mem_result_out <= mem_result_out;
            reg_write_out <= reg_write_out;
            rd_addr_out <= rd_addr_out;
        end else begin
            
            mem_result_out <= mem_result_in;
            reg_write_out <= reg_write_in;
            rd_addr_out <= rd_addr_in;
        end
    end
endmodule

module register_file(
    input clk,                          
    input rst,                          
    input [4:0] rs1_addr,               
    input [4:0] rs2_addr,               
    input [4:0] rd_addr,                
    input [63:0] rd_data,              
    input reg_write,                    
    output reg [63:0] rs1_data,             
    output reg [63:0] rs2_data              
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

    
    always @(*) begin
        
        
        rs1_data = (rs1_addr == 5'b0) ? 64'b0 : registers[rs1_addr];
        rs2_data = (rs2_addr == 5'b0) ? 64'b0 : registers[rs2_addr];
    end

    
    wire [63:0] r1_debug = registers[1];
    wire [63:0] r2_debug = registers[2];
    wire [63:0] r3_debug = registers[3];
    wire [63:0] r4_debug = registers[4];
    wire [63:0] r5_debug = registers[5];
    wire [63:0] r6_debug = registers[6];
    wire [63:0] r7_debug = registers[7];
    wire [63:0] r8_debug = registers[8];

endmodule


module instruction_memory(
    input [63:0] pc,                    
    output [31:0] instruction           
);
    reg [31:0] mem [0:1023];           

     integer i;
    initial begin
        for (i = 0; i < 1024; i = i + 1) begin
            mem[i] = 32'h00000000;
        end

        
        mem[0] = 32'h00500093; 
        mem[1] = 32'h00A00113; 
        mem[2] = 32'h002081B3; 
        mem[3] = 32'h0041A233; 
        mem[4] = 32'h00208463; 
        mem[5] = 32'h004000EF; 
        mem[6] = 32'h00008067; 
        mem[7] = 32'h00000013; 

    end


    
    assign instruction = mem[pc[11:2]]; 

    
endmodule



module data_memory(
    input clk,
    input [63:0] address,
    input [63:0] write_data,
    input mem_read,
    input mem_write,
    output [63:0] read_data
);
    reg [63:0] mem [0:1023]; // 1KB memory

    // Read operation
    assign read_data = (mem_read) ? mem[address[9:0]] : 64'b0;

    // Write operation
    always @(posedge clk) begin
        if (mem_write) begin
            mem[address[9:0]] <= write_data;
        end
    end
endmodule


module writeback(
    input [63:0] mem_result,            
    input reg_write,                    
    input [4:0] rd_addr,                
    output [63:0] write_back_data,      
    output [4:0] write_back_addr,       
    output reg_write_back               
);
    
    assign write_back_data = mem_result; 
    assign write_back_addr = rd_addr;    
    assign reg_write_back = reg_write;   
endmodule
