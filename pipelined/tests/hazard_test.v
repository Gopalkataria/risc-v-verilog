`timescale 1ns / 1ps

module hazard_detection_unit_tb;
    reg clk;
    reg reset;
    
    // Signals for hazard detection unit
    reg [4:0] id_ex_rs1_addr;
    reg [4:0] id_ex_rs2_addr;
    reg [4:0] ex_mem_rd_addr;
    reg [4:0] mem_wb_rd_addr;
    reg id_ex_mem_read;
    reg ex_mem_reg_write;
    reg mem_wb_reg_write;
    wire stall;

    // Instantiate the hazard detection unit
    hazard_detection_unit hdu (
        .id_ex_rs1_addr(id_ex_rs1_addr),
        .id_ex_rs2_addr(id_ex_rs2_addr),
        .ex_mem_rd_addr(ex_mem_rd_addr),
        .mem_wb_rd_addr(mem_wb_rd_addr),
        .id_ex_mem_read(id_ex_mem_read),
        .ex_mem_reg_write(ex_mem_reg_write),
        .mem_wb_reg_write(mem_wb_reg_write),
        .stall(stall)
    );

    // Clock Generation (2ns period)
    initial begin
        clk = 0;
        forever #1 clk = ~clk;
    end

    // Debugging
    initial begin
        $display("Time | id_ex_rs1_addr | id_ex_rs2_addr | ex_mem_rd_addr | mem_wb_rd_addr | id_ex_mem_read | ex_mem_reg_write | mem_wb_reg_write | stall");
        $monitor("%4t | %h | %h | %h | %h | %b | %b | %b | %b",
                 $time,
                 id_ex_rs1_addr,
                 id_ex_rs2_addr,
                 ex_mem_rd_addr,
                 mem_wb_rd_addr,
                 id_ex_mem_read,
                 ex_mem_reg_write,
                 mem_wb_reg_write,
                 stall);
    end

        initial begin
        $dumpfile("hazard_test.vcd");
        $dumpvars(0, hazard_detection_unit_tb);
    end

    // Test sequence
    initial begin
        // Initialize all signals
        reset = 1;
        id_ex_rs1_addr = 5'b00000;
        id_ex_rs2_addr = 5'b00000;
        ex_mem_rd_addr = 5'b00000;
        mem_wb_rd_addr = 5'b00000;
        id_ex_mem_read = 0;
        ex_mem_reg_write = 0;
        mem_wb_reg_write = 0;
        #6 reset = 0;

        // Test Case: Load-Use Hazard
        id_ex_rs1_addr = 5'b00001; // rs1 = x1
        id_ex_rs2_addr = 5'b00010; // rs2 = x2
        ex_mem_rd_addr = 5'b00001; // rd = x1 (overlap with rs1)
        mem_wb_rd_addr = 5'b00100; // rd = x4 (no overlap)
        id_ex_mem_read = 1; // Load instruction
        ex_mem_reg_write = 1;
        mem_wb_reg_write = 1;
        #2;
        $display("Stall: %b (Expected: 1)", stall);

        // End simulation
        #10;
        $display("\n===== Test completed =====");
        $finish;
    end
endmodule