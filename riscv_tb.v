`timescale 1ns / 1ps

module RISC_V_Single_Cycle_tb();
    reg clk;
    reg reset;
    integer i, file;
    reg [31:0] instruction_memory[0:63]; // Temporary storage for instructions
    
    // Instantiate the RISC-V processor
    RISC_V_Single_Cycle uut (
        .clk(clk),
        .reset(reset)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;  // 10ns clock period (100MHz)
    end
    
    // Read program from file and initialize instruction memory
    initial begin
        // Read the program from a file
        $readmemh("riscv_program.hex", instruction_memory);
        
        // Copy to the processor's instruction memory
        for (i = 0; i < 64; i = i + 1) begin
            uut.fetch.instr_mem[i] = instruction_memory[i];
        end
        
        // Initialize data memory
        for (i = 0; i < 64; i = i + 1) begin
            uut.memory.data_mem[i] = 32'h0;
        end
    end
    
    // Test sequence
    initial begin
        // Initialize
        reset = 1;
        #15;  // Assert reset for 15ns
        reset = 0;
        
        // Run simulation for enough cycles to execute all test instructions
        #500;
        
        // Store results in file
        file = $fopen("riscv_results.txt", "w");
        
        // Write header
        $fdisplay(file, "RISC-V Single Cycle Processor Simulation Results");
        $fdisplay(file, "=============================================");
        $fdisplay(file, "Register File Contents:");
        
        // Write register contents
        for (i = 0; i < 32; i = i + 1) begin
            $fdisplay(file, "x%0d = 0x%8h", i, uut.decode.reg_file[i]);
        end
        
        // Write data memory contents
        $fdisplay(file, "\nData Memory Contents (first 16 words):");
        for (i = 0; i < 16; i = i + 1) begin
            $fdisplay(file, "mem[%0d] = 0x%8h", i, uut.memory.data_mem[i]);
        end
        
        // Verification logic
        $fdisplay(file, "\nVerification Results:");
        if (uut.decode.reg_file[5] == 32'h00000008 &&
            uut.decode.reg_file[6] == 32'h00000014 &&
            uut.decode.reg_file[7] == 32'h00000014) begin
            $fdisplay(file, "TEST PASSED: Register values match expected outputs");
        end else begin
            $fdisplay(file, "TEST FAILED: Register values don't match expected outputs");
            $fdisplay(file, "Expected: x5=0x8, x6=0x14, x7=0x14");
            $fdisplay(file, "Got: x5=0x%h, x6=0x%h, x7=0x%h", 
                     uut.decode.reg_file[5], 
                     uut.decode.reg_file[6], 
                     uut.decode.reg_file[7]);
        end
        
        $fclose(file);
        $display("Simulation completed. Results written to riscv_results.txt");
        $finish;
    end
    
    // Monitor important signals to console during simulation
    initial begin
        $monitor("Time=%0t, PC=0x%8h, Instr=0x%8h, RegWrite=%b, ALU_result=0x%8h", 
                 $time, uut.PC, uut.instr, uut.RegWrite, uut.ALU_result);
    end
    
    // Dump waveform file for analysis (if using a simulator that supports it)
    initial begin
        $dumpfile("riscv_sim.vcd");
        $dumpvars(0, RISC_V_Single_Cycle_tb);
    end
    
endmodule