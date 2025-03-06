module RISC_V_Multi_Cycle (
    input clk,
    input reset
);
    
    reg [3:0] state, next_state;
    localparam FETCH = 4'b0000,
               DECODE = 4'b0001,
               EXECUTE = 4'b0010,
               MEMORY = 4'b0011,
               WRITEBACK = 4'b0100,
               BRANCH_COMPLETION = 4'b0101;

    
    reg [31:0] RegisterFile [31:0]; 
    integer i;

    
    reg PCWrite, IRWrite, RegWrite, ALUSrcA, MemtoReg, IorD;
    reg [1:0] ALUSrcB, PCSrc;
    wire MemRead, MemWrite, Branch, Jump, AUIPC, ALUSrc;
    wire RegWrite_ctrl, MemtoReg_ctrl;

    
    reg [31:0] PC, IR, A, B, ALUOut, MDR;
    wire [31:0] imm;
    wire [31:0] instr_data;     
    wire [31:0] read_data;      
    wire [31:0] mem_address = IorD ? ALUOut : PC;
    wire [31:0] alu_a = ALUSrcA ? A : PC;
    wire [31:0] alu_b;
    wire [31:0] reg_write_data;
    wire [31:0] branch_target;
    wire branch_taken;
    wire [31:0] next_PC;

    
    wire [6:0] funct7 = IR[31:25];
    wire [31:0] read_data1 = A;
    wire [31:0] read_data2 = B;
    wire [31:0] ALU_result;

    
    wire [6:0] opcode = IR[6:0];

    
    ImmGen imm_generator (
        .instr(IR),
        .imm(imm)
    );

    
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

    
    assign alu_b = ALUSrcB[1] ? (ALUSrcB[0] ? imm << 2 : imm) : 
                               (ALUSrcB[0] ? 32'h4 : B);

    
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

    
    Memory memory (
        .clk(clk),
        .reset(reset),
        .instr_addr(PC),            
        .data_addr(ALUOut),         
        .write_data(B),             
        .MemRead(MemRead),          
        .MemWrite(MemWrite),        
        .instr(instr_data),         
        .read_data(read_data)       
    );

    
    WriteBack writeback (
        .ALU_result(ALUOut),
        .mem_read_data(MDR),
        .MemtoReg(MemtoReg),
        .reg_write_data(reg_write_data)
    );

    
    Fetch fetch_unit (
        .clk(clk),
        .reset(reset),
        .PCWrite(PCWrite),
        .PCSrc(PCSrc),
        .branch_target(branch_target),
        .alu_result(ALU_result),
        .next_PC(next_PC),
        .current_PC(PC)
    );

    
    always @(posedge clk or posedge reset) begin
        if (reset)
            state <= FETCH;
        else
            state <= next_state;
    end

    
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

    
    always @(*) begin
        
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
                ALUSrcA = 1'b0;    
                ALUSrcB = 2'b01;   
                PCSrc = 2'b00;     
            end
            
            DECODE: begin
                ALUSrcA = 1'b0;    
                ALUSrcB = 2'b10;   
            end
            
            EXECUTE: begin
                ALUSrcA = 1'b1;    
                if (ALUSrc)
                    ALUSrcB = 2'b10;  
                else
                    ALUSrcB = 2'b00;  
                
                if (Jump) begin
                    PCWrite = 1'b1;
                    PCSrc = 2'b01;    
                end
            end
            
            MEMORY: begin
                IorD = 1'b1;       
            end
            
            WRITEBACK: begin
                RegWrite = 1'b1;
                MemtoReg = MemtoReg_ctrl;
            end
            
            BRANCH_COMPLETION: begin
                PCWrite = 1'b1;
                PCSrc = 2'b01;     
            end
        endcase
    end

    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            PC <= 32'h0;
        end else if (PCWrite) begin
            PC <= next_PC;
        end
    end

    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            IR <= 32'h0;
            A <= 32'h0;
            B <= 32'h0;
            ALUOut <= 32'h0;
            MDR <= 32'h0;
            
            
            for (i = 0; i < 32; i = i + 1)
                RegisterFile[i] <= 32'h0;
        end else begin
            
            if (IRWrite)
                IR <= instr_data;  
            
            
            if (state == DECODE) begin
                A <= RegisterFile[IR[19:15]]; 
                B <= RegisterFile[IR[24:20]]; 
            end
            
            
            if (state == EXECUTE)
                ALUOut <= ALU_result;
            
            
            if (state == MEMORY && MemRead)
                MDR <= read_data;  
            
            
            if (RegWrite && (IR[11:7] != 5'b0)) 
                RegisterFile[IR[11:7]] <= reg_write_data;
        end
    end
endmodule
