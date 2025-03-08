module hazard_detection_unit(
    input [4:0] id_ex_rs1_addr,       
    input [4:0] id_ex_rs2_addr,       
    input [4:0] ex_mem_rd_addr,       
    input [4:0] mem_wb_rd_addr,       
    input id_ex_mem_read,             
    input ex_mem_reg_write,           
    input mem_wb_reg_write,           
    output reg stall                  
);
   
    initial begin 
        stall = 1'b0;
    end
    
    always @(*) begin
       
        stall = 1'b0;
        
       
        if (id_ex_mem_read && ex_mem_reg_write && 
            (ex_mem_rd_addr != 5'b00000) && 
            ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr))) begin
            stall = 1'b1;
        end
        
       
       
        else if (!id_ex_mem_read && ex_mem_reg_write && 
                (ex_mem_rd_addr != 5'b00000) &&
                ((id_ex_rs1_addr == ex_mem_rd_addr) || (id_ex_rs2_addr == ex_mem_rd_addr))) begin
            stall = 1'b1;
        end
        
       
       
        else if (mem_wb_reg_write && 
                (mem_wb_rd_addr != 5'b00000) &&
                ((id_ex_rs1_addr == mem_wb_rd_addr) || (id_ex_rs2_addr == mem_wb_rd_addr))) begin
            stall = 1'b1;
        end
    end
endmodule