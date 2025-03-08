module full_adder(
    input a,
    input b,
    input cin,
    output sum,
    output cout
);
    wire w1, w2, w3;
    
    xor(w1, a, b);
    xor(sum, w1, cin);
    and(w2, w1, cin);
    and(w3, a, b);
    or(cout, w2, w3);
endmodule


module adder_64bit(
    input [63:0] a,
    input [63:0] b,
    input cin,
    output [63:0] sum,
    output cout
);
    wire [63:0] carry;
    
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : adder_loop
            if(i == 0)
                full_adder fa(
                    .a(a[0]),
                    .b(b[0]),
                    .cin(cin),
                    .sum(sum[0]),
                    .cout(carry[0])
                );
            else
                full_adder fa(
                    .a(a[i]),
                    .b(b[i]),
                    .cin(carry[i-1]),
                    .sum(sum[i]),
                    .cout(carry[i])
                );
        end
    endgenerate
    
    assign cout = carry[63];
endmodule


module twos_complement_64bit(
    input [63:0] in,
    output [63:0] out
);
    wire [63:0] not_in;
    wire dummy_cout;
    
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : complement_loop
            not(not_in[i], in[i]);
        end
    endgenerate
    
    adder_64bit add_one(
        .a(not_in),
        .b(64'b1),
        .cin(1'b0),
        .sum(out),
        .cout(dummy_cout)
    );
endmodule


module subtractor_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] diff
);
    wire [63:0] b_complement;
    wire dummy_cout;
    
    twos_complement_64bit comp(
        .in(b),
        .out(b_complement)
    );
    
    adder_64bit sub(
        .a(a),
        .b(b_complement),
        .cin(1'b0),
        .sum(diff),
        .cout(dummy_cout)
    );
endmodule


module and_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] result
);
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : and_loop
            and(result[i], a[i], b[i]);
        end
    endgenerate
endmodule


module or_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] result
);
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : or_loop
            or(result[i], a[i], b[i]);
        end
    endgenerate
endmodule


module xor_64bit(
    input [63:0] a,
    input [63:0] b,
    output [63:0] result
);
    genvar i;
    generate
        for(i = 0; i < 64; i = i + 1) begin : xor_loop
            xor(result[i], a[i], b[i]);
        end
    endgenerate
endmodule


module sll_64bit(
    input [63:0] a,
    input [5:0] shift_amt,
    output [63:0] result
);
    wire [63:0] shift_stage [5:0];
    
    assign shift_stage[0] = shift_amt[0] ? {a[62:0], 1'b0} : a;
    
    genvar i;
    generate
        for(i = 1; i < 6; i = i + 1) begin : shift_loop
            assign shift_stage[i] = shift_amt[i] ? 
                {shift_stage[i-1][63-(2**i):0], {(2**i){1'b0}}} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule


module srl_64bit(
    input [63:0] a,
    input [5:0] shift_amt,
    output [63:0] result
);
    wire [63:0] shift_stage [5:0];
    
    assign shift_stage[0] = shift_amt[0] ? {1'b0, a[63:1]} : a;
    
    genvar i;
    generate
        for(i = 1; i < 6; i = i + 1) begin : shift_loop
            assign shift_stage[i] = shift_amt[i] ? 
                {{(2**i){1'b0}}, shift_stage[i-1][63:2**i]} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule


module sra_64bit(
    input [63:0] a,
    input [5:0] shift_amt,
    output [63:0] result
);
    wire [63:0] shift_stage [5:0];
    wire sign_bit;
    
    assign sign_bit = a[63];
    
    assign shift_stage[0] = shift_amt[0] ? {sign_bit, a[63:1]} : a;
    
    genvar i;
    generate
        for(i = 1; i < 6; i = i + 1) begin : shift_loop
            assign shift_stage[i] = shift_amt[i] ? 
                {{(2**i){sign_bit}}, shift_stage[i-1][63:2**i]} : 
                shift_stage[i-1];
        end
    endgenerate
    
    assign result = shift_stage[5];
endmodule


    
module alu_64bit(
    input [2:0] funct3,     
    input [6:0] funct7,     
    input [63:0] a,         
    input [63:0] b,         
    output reg [63:0] result 
);
    wire [63:0] add_result;
    wire [63:0] sub_result;
    wire [63:0] and_result;
    wire [63:0] or_result;
    wire [63:0] xor_result;
    wire [63:0] sll_result;
    wire [63:0] srl_result;
    wire [63:0] sra_result;
    wire [63:0] slt_result;
    wire [63:0] sltu_result;

    adder_64bit add_op(
        .a(a),
        .b(b),
        .cin(1'b0),
        .sum(add_result)
    );

    subtractor_64bit sub_op(
        .a(a),
        .b(b),
        .diff(sub_result)
    );

    and_64bit and_op(
        .a(a),
        .b(b),
        .result(and_result)
    );

    or_64bit or_op(
        .a(a),
        .b(b),
        .result(or_result)
    );

    xor_64bit xor_op(
        .a(a),
        .b(b),
        .result(xor_result)
    );

    sll_64bit sll_op(
        .a(a),
        .shift_amt(b[5:0]),
        .result(sll_result)
    );

    srl_64bit srl_op(
        .a(a),
        .shift_amt(b[5:0]),
        .result(srl_result)
    );

    sra_64bit sra_op(
        .a(a),
        .shift_amt(b[5:0]),
        .result(sra_result)
    );

    assign slt_result = {63'b0, $signed(a) < $signed(b)};
    assign sltu_result = {63'b0, a < b};

    always @(*) begin
        case(funct3)
            3'b000: begin  
                if (funct7[5]) 
                    result = sub_result;
                else           
                    result = add_result;
            end
            3'b001: result = sll_result;   
            3'b010: result = slt_result;   
            3'b011: result = sltu_result;  
            3'b100: result = xor_result;   
            3'b101: begin  
                if (funct7[5]) 
                    result = sra_result;
                else           
                    result = srl_result;
            end
            3'b110: result = or_result;    
            3'b111: result = and_result;   
            default: result = 64'b0;
        endcase
    end
endmodule




















