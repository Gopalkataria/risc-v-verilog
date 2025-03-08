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

    // rs1_data = 64'b0;
    // rs2_data = 64'b0;

    // Reset logic moved inside always block
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

    // Read ports
    always @(*) begin
        // rs1_data = 64'b1010101001010100101100101001;
        // rs2_data = 64'b1010101001010100101100101001;
        rs1_data = (rs1_addr == 5'b0) ? 64'b0 : registers[rs1_addr];
        rs2_data = (rs2_addr == 5'b0) ? 64'b0 : registers[rs2_addr];
    end

    // Debugging signals
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


    
    assign instruction = mem[pc[9:0]>>2]; 

    
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
