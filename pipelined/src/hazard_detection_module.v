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

    // always @(*) begin
        
    //     if (id_ex_mem_read && ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr))) begin
    //         stall = 1'b1;
    //     end else begin
    //         stall = 1'b0;
    //     end
    // end
endmodule
