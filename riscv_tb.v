module riscv_tb();

    reg clk;
    reg reset;
    
    // Instantiate the RISC-V processor
    RISC_V_Single_Cycle dut (
        .clk(clk),
        .reset(reset)
    );
    
    // Clock generation
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end
    
    // Reset generation
    initial begin
        reset = 1;
        #10;
        reset = 0;
    end
    
    // Load program into memory
    initial begin
        // Wait for memory initialization
        #10;
        
        // Load test program into instruction memory
        // Memory addresses are word addresses (4 bytes per word)
        dut.mem.mem[0] = 32'h00500093;  // ADDI x1, x0, 5
        dut.mem.mem[1] = 32'h00300113;  // ADDI x2, x0, 3
        dut.mem.mem[2] = 32'h002080b3;  // ADD x3, x1, x2
        dut.mem.mem[3] = 32'h00302023;  // SW x3, 0(x0)
        dut.mem.mem[4] = 32'h00002203;  // LW x4, 0(x0)
        dut.mem.mem[5] = 32'h00418863;  // BEQ x3, x4, 8
        dut.mem.mem[6] = 32'h00100293;  // ADDI x5, x0, 1
        dut.mem.mem[7] = 32'h0000006f;  // JAL x0, 0x1c (infinite loop)
    end
    
    // Monitor signals
    reg [31:0] prev_pc;
    initial begin
        prev_pc = 32'hx;
        $dumpfile("riscv_tb.vcd");
        $dumpvars(0, riscv_tb);
        $display("Time\tPC\t\tInstruction\tx1\tx2\tx3\tx4\tx5\tRegWrite\tMemRead\tMemWrite\tALUSrc\tBranch\tMemtoReg\tJump\tAUIPC\tfunct3\tALU_result\tbranch_taken\tbranch_target");
        forever begin
            @(posedge clk);
            #10; // Wait for signals to propagate

            // Display register values and control signals
            $display("%4d\t%h\t%h\t%h\t%h\t%h\t%h\t%h\t%b\t%b\t%b\t%b\t%b\t%b\t%b\t%b\t%b\t%h\t%b\t%h",
                $time,
                dut.fetch.PC,
                dut.fetch.instr,
                dut.decode.reg_file[1],
                dut.decode.reg_file[2],
                dut.decode.reg_file[3],
                dut.decode.reg_file[4],
                dut.decode.reg_file[5],
                dut.decode.RegWrite,
                dut.decode.MemRead,
                dut.decode.MemWrite,
                dut.decode.ALUSrc,
                dut.decode.Branch,
                dut.decode.MemtoReg,
                dut.decode.Jump,
                dut.decode.AUIPC,
                dut.decode.funct3,
                dut.execute.ALU_result,
                dut.execute.branch_taken,
                dut.execute.branch_target);

            // Check for halt condition
            if (prev_pc === dut.fetch.PC) begin
                $display("\nHALT detected at PC %h", dut.fetch.PC);
                $display("Final register values:");
                $display("x1: %h (5 expected)", dut.decode.reg_file[1]);
                $display("x2: %h (3 expected)", dut.decode.reg_file[2]);
                $display("x3: %h (8 expected)", dut.decode.reg_file[3]);
                $display("x4: %h (8 expected)", dut.decode.reg_file[4]);
                $display("x5: %h (1 not expected - branch should skip)", 
                    dut.decode.reg_file[5]);
                $finish;
            end
            prev_pc = dut.fetch.PC;
        end
    end
    
    // Data memory monitoring
    always @(posedge clk) begin
        if (dut.mem.MemWrite) begin
            $display("Memory Write: Address %h, Data %h", 
                dut.execute.ALU_result,
                dut.decode.read_data2);
        end
    end

endmodule