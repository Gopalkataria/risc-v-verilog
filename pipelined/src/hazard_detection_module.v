module hazard_detection_unit(
    input [4:0] id_ex_rs1_addr,       // Source register 1 address in ID/EX
    input [4:0] id_ex_rs2_addr,       // Source register 2 address in ID/EX
    input [4:0] ex_mem_rd_addr,       // Destination register address in EX/MEM
    input [4:0] mem_wb_rd_addr,       // Destination register address in MEM/WB
    input id_ex_mem_read,             // Memory read signal in ID/EX (load instruction)
    input ex_mem_reg_write,           // Register write signal in EX/MEM
    input mem_wb_reg_write,           // Register write signal in MEM/WB
    output reg stall                  // Stall signal
);

    always @(*) begin
        // Default: No stall
        stall = 1'b0;

        // Load-Use Hazard: Stall if a load instruction is followed by an instruction
        // that depends on the loaded data.
        if (id_ex_mem_read && 
            ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr)) &&
            (ex_mem_rd_addr != 5'b00000)) begin
            stall = 1'b1;
        end

        // RAW Hazard: Stall if an instruction depends on the result of a previous
        // instruction in the EX/MEM stage.
        else if (ex_mem_reg_write && 
                 ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr)) &&
                 (ex_mem_rd_addr != 5'b00000)) begin
            stall = 1'b1;
        end

        // No need to check for RAW hazard with MEM/WB stage because the value is already available.
    end
endmodule