`include "execution_module.v"
`timescale 1ns/1ps

module Execute_tb;
    reg [31:0] PC;
    reg [31:0] read_data1;
    reg [31:0] read_data2;
    reg [31:0] imm;
    reg [2:0] funct3;
    reg [6:0] funct7;
    reg ALUSrc;
    reg Branch;

    wire [31:0] ALU_result;
    wire branch_taken;
    wire [31:0] branch_target;

    // Instantiate the Execute module
    Execute uut (
        .PC(PC),
        .read_data1(read_data1),
        .read_data2(read_data2),
        .imm(imm),
        .ALUSrc(ALUSrc),
        .Branch(Branch),
        .funct3(funct3),
        .funct7(funct7),
        .ALU_result(ALU_result),
        .branch_taken(branch_taken),
        .branch_target(branch_target)
    );

    initial begin
        // Print Header
        $display("---------------------------------------------------------------------------------------------------");
        $display("|   Time  |   PC    | Read1   | Read2   | Imm     | ALUSrc | Branch | Funct3 | Funct7  | Result  | Branch Taken | Branch Target |");
        $display("---------------------------------------------------------------------------------------------------");

        // Test Case 1: ADD Operation
        PC = 32'h00000004;
        read_data1 = 32'h00000010;
        read_data2 = 32'h00000020;
        imm = 32'h00000008;
        ALUSrc = 0;
        Branch = 0;
        funct3 = 3'b000;
        funct7 = 7'b0000000;
        #10;
        print_results();

        // Test Case 2: SUB Operation
        funct7 = 7'b0100000;  // SUB is identified with funct7[5] = 1
        #10;
        print_results();

        // Test Case 3: AND Operation
        funct3 = 3'b111;
        funct7 = 7'b0000000;
        #10;
        print_results();

        // Test Case 4: OR Operation
        funct3 = 3'b110;
        #10;
        print_results();

        // Test Case 5: BEQ (should not take branch)
        Branch = 1;
        funct3 = 3'b000;
        read_data1 = 32'h00000010;
        read_data2 = 32'h00000020;
        #10;
        print_results();

        // Test Case 6: BEQ (should take branch)
        read_data2 = 32'h00000010;
        #10;
        print_results();

        // Test Case 7: BLT (should not take branch)
        funct3 = 3'b100;
        read_data1 = 32'h00000030;
        read_data2 = 32'h00000020;
        #10;
        print_results();

        // Test Case 8: BLT (should take branch)
        read_data1 = 32'h00000010;
        #10;
        print_results();

        // End test
        $display("---------------------------------------------------------------------------------------------------");
        $finish;
    end

    // Task to print results in a formatted table
    task print_results;
        begin
            $display("| %6t | %h | %h | %h | %h |   %b   |   %b   |  %b  |  %b | %h |       %b      |     %h    |",
                    $time, PC, read_data1, read_data2, imm, ALUSrc, Branch, funct3, funct7, ALU_result, branch_taken, branch_target);
        end
    endtask
endmodule
