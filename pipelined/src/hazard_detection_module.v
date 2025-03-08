
 module hazard_detection_unit(
    input [4:0] id_ex_rs1_addr,        // Source register 1 address from ID/EX pipeline register
    input [4:0] id_ex_rs2_addr,        // Source register 2 address from ID/EX pipeline register
    input [4:0] ex_mem_rd_addr,        // Destination register address from EX/MEM pipeline register
    input [4:0] mem_wb_rd_addr,        // Destination register address from MEM/WB pipeline register
    input id_ex_mem_read,              // Control signal indicating memory read in EX stage
    input ex_mem_reg_write,            // Control signal indicating register write in MEM stage
    input mem_wb_reg_write,            // Control signal indicating register write in WB stage
    output reg stall                   // Output signal to stall the pipeline
);
    // Initialize stall to 0
    initial begin 
        stall = 1'b0;
    end
    
    always @(*) begin
        // Default: no stall
        stall = 1'b0;
        
        // Check for load-use hazard (when a load instruction is followed by an instruction that uses the loaded value)
        if (id_ex_mem_read && ex_mem_reg_write && 
            (ex_mem_rd_addr != 5'b00000) &&  // Only stall if destination register is not x0
            ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr))) begin
            stall = 1'b1;
        end
        
      
       
    end
endmodule