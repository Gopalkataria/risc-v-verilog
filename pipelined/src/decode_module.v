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
