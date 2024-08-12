`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/02/2023 12:46:51 AM
// Design Name: 
// Module Name: modules
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


module part1 #(parameter N = 8) (
    input [0:0] clk,
    input [0:0] en,
    input [N-1:0] inp,
    input [1:0] funSel,
    output reg [N-1:0] out);
    
    initial out = 0;
    
    always@ (posedge clk) begin
        if(en) begin
            case(funSel)
                2'b00: begin
                    out <= 0;
                end
                
                2'b01: begin
                    out <= inp;
                end
                
                2'b10: begin
                    out <= out - 1;
                end
                
                2'b11: begin
                    out <= out + 1;
                end
            endcase
        end
        
        else begin
            out <= out;
        end
    end
endmodule


module part2_a (
    input [0:0] clk,
    input [7:0] inp,
    input [1:0] funSel,
    input [0:0] en,
    input [0:0] lh,
    output reg [15:0] out);
    
    initial out = 16'b0;

    always@ (posedge clk) begin
        if(en) begin
            case(funSel)
                2'b00: out <= 0;
                2'b01: begin
                    if(lh) begin
                        out[15:8] <= inp;
                    end else begin
                        out[7:0] <= inp;
                    end
                end
                2'b10: out <= out-1;
                2'b11: out <= out+1;
            endcase
        end else begin
            out = out;
        end
    end
endmodule



module part_2b(
    input [0:0] clk, //done
    input [7:0] inp, //done
    input [2:0] o1Sel, o2Sel, //
    input [1:0] funSel, //done
    input [3:0] rSel, tSel, //done
    output reg [7:0] o1, //
    output reg [7:0] o2); //
    
    initial o1 = 8'b0;
    initial o2 = 8'b0;
    
    wire [7:0] w0, w1, w2, w3, w4, w5, w6, w7;
    part1 #(8) t_l( .clk(clk), .en( tSel[3] ), .inp( inp ), .funSel(funSel), .out(w0) );
    part1 #(8) t_2( .clk(clk), .en( tSel[2] ), .inp( inp ), .funSel(funSel), .out(w1) );
    part1 #(8) t_3( .clk(clk), .en( tSel[1] ), .inp( inp ), .funSel(funSel), .out(w2) );
    part1 #(8) t_4( .clk(clk), .en( tSel[0] ), .inp( inp ), .funSel(funSel), .out(w3) );
    part1 #(8) r_l( .clk(clk), .en( rSel[3] ), .inp( inp ), .funSel(funSel), .out(w4) );
    part1 #(8) r_2( .clk(clk), .en( rSel[2] ), .inp( inp ), .funSel(funSel), .out(w5) );
    part1 #(8) r_3( .clk(clk), .en( rSel[1] ), .inp( inp ), .funSel(funSel), .out(w6) );
    part1 #(8) r_4( .clk(clk), .en( rSel[0] ), .inp( inp ), .funSel(funSel), .out(w7) );
    
    always@ (*) begin
         
        case( o1Sel )
            3'b000: o1 <= w0;
            3'b001: o1 <= w1;
            3'b010: o1 <= w2;
            3'b011: o1 <= w3;
            3'b100: o1 <= w4;
            3'b101: o1 <= w5;
            3'b110: o1 <= w6;
            3'b111: o1 <= w7;
        endcase
        
        case( o2Sel )
            3'b000: o2 <= w0;
            3'b001: o2 <= w1;
            3'b010: o2 <= w2;
            3'b011: o2 <= w3;
            3'b100: o2 <= w4;
            3'b101: o2 <= w5;
            3'b110: o2 <= w6;
            3'b111: o2 <= w7;
        endcase
        
    end
endmodule



module part_2c(
    input [0:0] clk, //
    input [7:0] inp, //
    input [1:0] oASel, oBSel, //
    input [1:0] funSel, //done
    input [3:0] rSel, //done
    output reg [7:0] oA, //
    output reg [7:0] oB); //
    
    initial oA = 8'b0;
    initial oB = 8'b0;
    
    wire [7:0] w0, w1, w2, w3;
    part1 #(8) pc( .clk(clk), .en( rSel[3] ), .inp( inp ), .funSel(funSel), .out(w0) );
    part1 #(8) ar( .clk(clk), .en( rSel[2] ), .inp( inp ), .funSel(funSel), .out(w1) );
    part1 #(8) sp( .clk(clk), .en( rSel[1] ), .inp( inp ), .funSel(funSel), .out(w2) );
    part1 #(8) pcPast( .clk(clk), .en( rSel[0] ), .inp( inp ), .funSel(funSel), .out(w3) );
    
    always@ (*) begin
         
        case( oASel )
            2'b00: oA <= w1;
            2'b01: oA <= w2;
            2'b10: oA <= w3;
            2'b11: oA <= w0;
        endcase
        
        case( oBSel )
            2'b00: oB <= w1;
            2'b01: oB <= w2;
            2'b10: oB <= w3;
            2'b11: oB <= w0;
        endcase
        
    end
endmodule


module adder1bit (
    input [0:0] A, B, cin,
    output [0:0] Sum, carry
);

    wire w1, w2, w3;

    assign w1 = A ^ B;
    assign Sum = w1 ^ cin;
    assign w2 = A & B;
    assign w3 = w1 & cin;
    assign carry = w2 | w3;
endmodule

module is_zero(
    input [7:0] inp, //+
    output wire [0:0] out); //+
    
    assign out = ~inp[0]&~inp[1]&~inp[2]&~inp[3]&~inp[4]&~inp[5]&~inp[6]&~inp[7];
endmodule


module full_adder_8bit( // passed test
    input [7:0] A, B,
    input [0:0] Cin,
    output [7:0] Sum,
    output [0:0] carry,
    output [0:0] overflow,
    output [0:0] zero);

    wire [7:0] S;
    wire C0, C1, C2, C3, C4, C5, C6, C7;
    adder1bit FA0(A[0], B[0], Cin, S[0], C0);
    adder1bit FA1(A[1], B[1], C0, S[1], C1);
    adder1bit FA2(A[2], B[2], C1, S[2], C2);
    adder1bit FA3(A[3], B[3], C2, S[3], C3);
    adder1bit FA4(A[4], B[4], C3, S[4], C4);
    adder1bit FA5(A[5], B[5], C4, S[5], C5);
    adder1bit FA6(A[6], B[6], C5, S[6], C6);
    adder1bit FA7(A[7], B[7], C6, S[7], C7);
 
    assign carry = C7;
    assign overflow = ~A[7]&~B[7]&Sum[7] | A[7]&B[7]&~Sum[7];   
    assign Sum = S;
    is_zero z0(Sum, zero);
endmodule


module subtracter8bit( // passed test
    input [7:0] A, B,
    output [0:0] carry,
    output [7:0] result,
    output [0:0] overflow,
    output [0:0] zero);
    
    wire [7:0] comp_B = ~B;
    wire [0:0] cin = 1'b1;
    full_adder_8bit FA0(.A(A), .B(comp_B), .Cin(cin), .Sum(result), .carry(carry));
    assign overflow = ~A[7]&B[7]&result[7] | A[7]&~B[7]&~result[7];
    is_zero z0(result, zero);
endmodule


module compare( // passed test
    input [7:0] inp_A,
    input [7:0] inp_B,
    output [7:0] result,
    output [0:0] zero_flag,
    output [0:0] overflow_flag,
    output [0:0] carry_flag);
    
    wire [7:0] res;
    subtracter8bit s0(.A(inp_A), .B(inp_B), .carry(carry_flag), .overflow(overflow_flag), .result(res));
    
    is_zero z0(res, zero_flag);
    
    reg signed [7:0] temp_A;
    reg signed [7:0] temp_B;
    reg [7:0] temp;
        
    always@ (*) begin
        temp_A <= inp_A;
        temp_B <= inp_B;
        if (temp_A > temp_B)
            temp <= temp_A;
        else
            temp <= 8'b00000000;    
    end
    assign result = temp;
endmodule


module get_and(
    input [7:0] inp_A, 
    input [7:0] inp_B, 
    output [0:0] is_zero, 
    output wire [7:0] out);
    
    assign out = inp_A & inp_B;
    is_zero z1(out, is_zero);
    
endmodule

module get_or(
    input [7:0] inp_A, 
    input [7:0] inp_B, 
    output [0:0] is_zero, 
    output wire [7:0] out);
    
    assign out = inp_A | inp_B;
    is_zero z1(out, is_zero);
    
endmodule

module get_nand(
    input [7:0] inp_A, 
    input [7:0] inp_B, 
    output [0:0] is_zero, 
    output wire [7:0] out);
    
    assign out = ~(inp_A & inp_B);
    is_zero z1(out, is_zero);
    
endmodule

module get_xor(
    input [7:0] inp_A, 
    input [7:0] inp_B, 
    output [0:0] is_zero, 
    output wire [7:0] out);
    
    assign out = inp_A ^ inp_B;
    is_zero z1(out, is_zero);
    
endmodule



module ASR( // passed test
    input [7:0] inp_A,
    output wire [7:0] outALU,
    output wire [0:0] zero_flag
    );
    
    wire [7:0] shift;
    
    assign shift[0] = inp_A[1];
    assign shift[1] = inp_A[2];
    assign shift[2] = inp_A[3];
    assign shift[3] = inp_A[4];
    assign shift[4] = inp_A[5];
    assign shift[5] = inp_A[6];
    assign shift[6] = inp_A[7];
    assign shift[7] = inp_A[7];
    
    is_zero ASR_zero(shift, zero_flag);
    
    assign outALU = shift;
    
endmodule



module ASL( // passed test
    input [7:0] inp_A,
    output wire [7:0] outALU,
    output wire [0:0] zero_flag,
    output wire [0:0] overflow_flag
    );
    
    wire [7:0] shift;
    
    assign shift[0] = 1'b0;
    assign shift[1] = inp_A[0];
    assign shift[2] = inp_A[1];
    assign shift[3] = inp_A[2];
    assign shift[4] = inp_A[3];
    assign shift[5] = inp_A[4];
    assign shift[6] = inp_A[5];
    assign shift[7] = inp_A[6];
    
    is_zero ASL_zero(shift, zero_flag);
    
    assign overflow_flag = inp_A[7] ^ inp_A[6];
    
    assign outALU = shift;
    
endmodule


module LSR( // passed test
    input [7:0] inp_A,
    output wire [7:0] outALU,
    output wire [0:0] zero_flag,
    output wire [0:0] carry_flag
);

    wire [7:0] shift;
    
    assign shift[0] = inp_A[1];
    assign shift[1] = inp_A[2];
    assign shift[2] = inp_A[3];
    assign shift[3] = inp_A[4];
    assign shift[4] = inp_A[5];
    assign shift[5] = inp_A[6];
    assign shift[6] = inp_A[7];
    assign shift[7] = 1'b0;
    
    is_zero LSR_zero(shift, zero_flag);
    
    assign carry_flag = inp_A[0];
    assign outALU = shift;
endmodule



module LSL(
    input [7:0] inp_A,
    output wire [7:0] outALU,
    output wire [0:0] zero_flag,
    output wire [0:0] carry_flag
);

    wire [7:0] shift;
    
    assign shift[0] = 1'b0;
    assign shift[1] = inp_A[0];
    assign shift[2] = inp_A[1];
    assign shift[3] = inp_A[2];
    assign shift[4] = inp_A[3];
    assign shift[5] = inp_A[4];
    assign shift[6] = inp_A[5];
    assign shift[7] = inp_A[6]; 
    
    is_zero LSL_zero(shift, zero_flag);
    
    assign carry_flag = inp_A[7];
    
    assign outALU = shift;

endmodule



module CSR( // passed test
    input [7:0] inp_A,
    input [0:0] c_in,
    output wire [7:0] outALU,
    output wire [0:0] zero_flag,
    output wire [0:0] carry_flag
);

    wire [7:0] shift;
    
    assign shift[0] = inp_A[1];
    assign shift[1] = inp_A[2];
    assign shift[2] = inp_A[3];
    assign shift[3] = inp_A[4];
    assign shift[4] = inp_A[5];
    assign shift[5] = inp_A[6];
    assign shift[6] = inp_A[7];
    assign shift[7] = c_in; 
    
    is_zero CSR_zero(shift, zero_flag);
    
    assign carry_flag = inp_A[0];
    
    assign outALU = shift;
endmodule


module part_3(
    input [0:0] clk, //+
    input [7:0] inp_A, inp_B, //+
    input [3:0] funSel, //+
    output reg [7:0] outALU,
    output reg [3:0] flags); //+
    
    initial outALU = 8'b0;
    initial flags = 4'b0;
    wire [7:0] w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15;
    wire [0:0] w0_z, w1_z, w4_z, w5_z, w6_z, w7_z, w8_z, w9_z, w10_z, w11_z, w12_z, w13_z, w14_z, w15_z;
    wire [0:0] w4_c, w5_c, w6_c, w11_c, w12_c, w15_c;
    wire [0:0] w4_o, w5_o, w6_o, w13_o;
    
    is_zero A(inp_A, w0_z); // same for A and ~A
    is_zero B(inp_B, w1_z); // same for B and ~B
    full_adder_8bit m4( .A(inp_A), .B(inp_B), .Cin(flags[2]), .Sum(w4), .carry(w4_c), .overflow(w4_o), .zero(w4_z) );
    subtracter8bit m5( .A(inp_A), .B(inp_B), .carry(w5_c), .result(w5), .overflow(w5_o), .zero(w5_z) );
    compare m6( .inp_A(inp_A), .inp_B(inp_B), .result(w6), .zero_flag(w6_z), .overflow_flag(w6_o), .carry_flag(w6_c) );
    get_and m7( .inp_A(inp_A), .inp_B(inp_B), .is_zero(w7_z), .out(w7) );
    get_or m8( .inp_A(inp_A), .inp_B(inp_B), .is_zero(w8_z), .out(w8) );
    get_nand m9( .inp_A(inp_A), .inp_B(inp_B), .is_zero(w9_z), .out(w9) );
    get_xor m10( .inp_A(inp_A), .inp_B(inp_B), .is_zero(w10_z), .out(w10) );
    LSL m11( .inp_A(inp_A), .outALU(w11), .zero_flag(w11_z), .carry_flag(w11_c) );
    LSR m12( .inp_A(inp_A), .outALU(w12), .zero_flag(w12_z), .carry_flag(w12_c) );
    ASL m13( .inp_A(inp_A), .outALU(w13), .zero_flag(w13_z), .overflow_flag(w13_o) );
    ASR m14( .inp_A(inp_A), .outALU(w14), .zero_flag(w14_z) );
    CSR m15( .inp_A(inp_A), .c_in(flags[2]), .outALU(w15), .zero_flag(w15_z), .carry_flag(w15_c) );
    
    
    
    always@ (*) begin
         
        case( funSel ) //caseleri dÃ¼zelt
            4'b0000: begin //+
                outALU <= inp_A;
                flags[3] <= w0_z; //Z
                flags[1] <= inp_A[7]; //N
            end
            4'b0001: begin //+
                outALU <= inp_B;
                flags[3] <= w1_z; //Z
                flags[1] <= inp_B[7]; //N
            end
            4'b0010: begin //+
                outALU <= ~inp_A;
                flags[3] <= w0_z; //Z
                flags[1] <= ~inp_A[7]; //N
            end
            4'b0011: begin //+
                outALU <= ~inp_B;
                flags[3] <= w1_z; //Z
                flags[1] <= ~inp_B[7]; //N
            end
            4'b0100: begin
                outALU <= w4;
                flags[3] <= w4_z; //Z
                flags[2] <= w4_c; //C
                flags[1] <= w4[7]; //N
                flags[0] <= w4_o; //O
            end
            4'b0101: begin
                outALU <= w5;
                flags[3] <= w5_z; //Z
                flags[2] <= w5_c; //C
                flags[1] <= w5[7]; //N
                flags[0] <= w5_o; //O
            end
            4'b0110: begin
                outALU <= w6;
                flags[3] <= w6_z; //Z
                flags[2] <= w6_c; //C
                flags[1] <= w6[7]; //N
                flags[0] <= w6_o; //O
            end
            4'b0111: begin
                outALU <= w7;
                flags[3] <= w7_z; //Z
                flags[1] <= w7[7]; //N
            end
            4'b1000: begin
                outALU <= w8;
                flags[3] <= w8_z; //Z
                flags[1] <= w8[7]; //N
            end
            4'b1001: begin
                outALU <= w9;
                flags[3] <= w9_z; //Z
                flags[1] <= w9[7]; //N
            end
            4'b1010: begin
                outALU <= w10;
                flags[3] <= w10_z; //Z
                flags[1] <= w10[7]; //N
            end
            4'b1011: begin
                outALU <= w11;
                flags[3] <= w11_z; //Z
                flags[2] <= w11_c; //C
                flags[1] <= w11[7]; //N
            end
            4'b1100: begin
                outALU <= w12;
                flags[3] <= w12_z; //Z
                flags[2] <= w12_c; //C
                flags[1] <= w12[7]; //N
            end
            4'b1101: begin
                outALU <= w13;
                flags[3] <= w13_z; //Z
                flags[1] <= w13[7]; //N
            end
            4'b1110: begin
                outALU <= w14;
                flags[3] <= w14_z; //Z
            end
            4'b1111: begin
                outALU <= w15;
                flags[3] <= w15_z; //Z
                flags[2] <= w15_c; //C
                flags[1] <= w15[7]; //N
            end
        endcase
        
    end
endmodule


module Memory(
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    input wire clock,
    output reg[7:0] o // Output
);
    //Declaration oýf the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] <= data; 
        end
    end
endmodule


module ALU_System(
    input [2:0] RF_OutASel, //+
    input [2:0] RF_OutBSel,//+
    input [1:0] RF_FunSel, //+
    input [3:0] RF_RSel, //+
    input [3:0] RF_TSel, //+
    input [3:0] ALU_FunSel, //+
    input [1:0] ARF_OutCSel, //+
    input [1:0] ARF_OutDSel, //+
    input [1:0] ARF_FunSel, //+
    input [3:0] ARF_RegSel, //+
    input [0:0] IR_LH, //+
    input [0:0] IR_Enable, //+
    input [1:0] IR_Funsel, //+
    input [0:0] Mem_WR, //+
    input [0:0] Mem_CS, //+
    input [1:0] MuxASel, //+
    input [1:0] MuxBSel, //+
    input [0:0] MuxCSel, //+
    input [0:0] Clock, //+
    
    output wire[7:0] AOut, // |
    output wire[7:0] BOut, // |
    output wire[7:0] ALUOut, // |
    output wire[3:0] ALUOutFlag, // |
    output wire[7:0] ARF_AOut, // |
    output wire[7:0] Address, //ARF_BOut | done
    output wire[7:0] MemoryOut, // |
    output reg[7:0] MuxAOut, // |
    output reg[7:0] MuxBOut, // |
    output reg[7:0] MuxCOut, // |
    output wire[15:0] IROut // |
    
    );
        
        Memory mem(.address(Address), .data(ALUOut), .clock(Clock), .wr(Mem_WR), .cs(Mem_CS), .o(MemoryOut)); //done
        part2_a p2a(.clk(Clock), .en(IR_Enable), .inp(MemoryOut), .funSel(IR_Funsel), .out(IROut), .lh(IR_LH)); //done
        part_2b p2b(.clk(Clock), .inp(MuxAOut), .o1Sel(RF_OutASel), .o2Sel(RF_OutBSel), .funSel(RF_FunSel), .rSel(RF_RSel), .tSel(RF_TSel), .o1(AOut), .o2(BOut)); //DONE
        part_2c p2c(.clk(Clock), .inp(MuxBOut), .oASel(ARF_OutCSel), .oBSel(ARF_OutDSel), .funSel(ARF_FunSel), .rSel(ARF_RegSel), .oA(ARF_AOut), .oB(Address)); //done
        part_3 p3(.clk(Clock), .inp_A(MuxCOut), .inp_B(BOut), .funSel(ALU_FunSel), .outALU(ALUOut), .flags(ALUOutFlag));     
    
    always@ (*) begin //Probably done
        case (MuxASel)
            2'b00: MuxAOut <= ALUOut;
            2'b01: MuxAOut <= MemoryOut;
            2'b10: MuxAOut <= IROut[7:0];
            2'b11: MuxAOut <= ARF_AOut;
        endcase
        
        case (MuxBSel)
            2'b00: MuxBOut <= ALUOut;
            2'b01: MuxBOut <= MemoryOut;
            2'b10: MuxBOut <= IROut[7:0];
            2'b11: MuxBOut <= ARF_AOut;
        endcase
        
        case (MuxCSel)
            1'b0: MuxCOut <= AOut;
            1'b1: MuxCOut <= ARF_AOut;
        endcase
    end
   
endmodule

module CPUSystem(
    input wire[0:0] Reset,
    input wire[0:0] Clock,
    output reg[7:0] T
    );
    
    reg[2:0] RF_OutASel, RF_OutBSel;
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RSel, RF_TSel, ALU_FunSel;
    reg[1:0] ARF_OutCSel, ARF_OutDSel, ARF_FunSel;
    reg[3:0] ARF_RegSel;
    reg[0:0] IR_LH, IR_Enable;
    reg[1:0] IR_Funsel;
    reg[0:0] Mem_WR, Mem_CS;
    reg[1:0] MuxASel, MuxBSel;
    reg[0:0] MuxCSel;
    
    wire[15:0] IROut;
    wire[3:0] ALUOutFlag;
    
    reg[3:0] seg_counter;
    initial seg_counter = 4'b0000;
    reg[3:0] current_ALU_flag;
    
    ALU_System potato_pc(
        .RF_OutASel(RF_OutASel),
        .RF_OutBSel(RF_OutBSel),
        .RF_FunSel(RF_FunSel),
        .RF_RSel(RF_RSel),
        .RF_TSel(RF_TSel),
        .ALU_FunSel(ALU_FunSel),
        .ARF_OutCSel(ARF_OutCSel),
        .ARF_OutDSel(ARF_OutDSel),
        .ARF_FunSel(ARF_FunSel),
        .ARF_RegSel(ARF_RegSel),
        .IR_LH(IR_LH),
        .IR_Enable(IR_Enable),
        .IR_Funsel(IR_Funsel),
        .Mem_WR(Mem_WR),
        .Mem_CS(Mem_CS),
        .MuxASel(MuxASel),
        .MuxBSel(MuxBSel),
        .MuxCSel(MuxCSel),
        .Clock(Clock),
        .AOut(),
        .BOut(),
        .ALUOut(),
        .ALUOutFlag(ALUOutFlag),
        .ARF_AOut(),
        .Address(),
        .MemoryOut(),
        .MuxAOut(),
        .MuxBOut(),
        .MuxCOut(),
        .IROut(IROut)
    );
    
    //Reset signal comes - reset IR, ARF, RF and seq_counter.
     always@(posedge Reset) begin
     
        seg_counter <= 4'b0000;
        
        //Clear register file
        RF_FunSel <= 2'b00;
        RF_RSel <= 4'b1111;
        RF_TSel <= 4'b1111;
        
        //Clear address register file
        ARF_FunSel <= 2'b00;
        ARF_RegSel <= 4'b1111;
        
        //Clear instruction register
        IR_Enable <= 'b1;
        IR_Funsel <= 2'b00;
        
     end
     
     //Increment sequence counter at clock signal
     always @(posedge Clock) begin
        if(!Reset) begin
            seg_counter <= seg_counter + 1;
            T <= {4'b0000, seg_counter};
        end
     end
     
     //Like Good old days in [REDACTED]
     always @(*) begin
        if(!Reset) begin //If there is no reset signal
        
            //Load LSB of IR from memory
            if(seg_counter == 'h1 ) begin
                
                current_ALU_flag <= ALUOutFlag; //Save current ALU flag
                
                Mem_CS <= 1'b0; //enable memory chip
                Mem_WR <= 1'b0; //read mode
                IR_Enable <= 1'b1;
                IR_Funsel <= 2'b01;
                IR_LH <= 1'b0;
                
                //Memory Address =  PC
                ARF_OutDSel <= 2'b11;
                
                //Increase PC
                ARF_FunSel <= 2'b11;
                ARF_RegSel <= 4'b1000;
                
                //Disable unused registers
                RF_RSel <= 4'b0000;
                RF_TSel <= 4'b0000;
            end
            
            //Load MSB of IR from memory
            else if(seg_counter == 'h2) begin
                
                Mem_CS <= 1'b0; //enable memory chip
                Mem_WR <= 1'b0; //read mode
                IR_Enable <= 1'b1;
                IR_Funsel <= 2'b01;
                IR_LH <= 1'b1;
                
                //Memory Address =  PC
                ARF_OutDSel <= 2'b11;
                
                //Increase PC
                ARF_FunSel <= 2'b11;
                ARF_RegSel <= 4'b1000;
                
                //Disable unused registers
                RF_RSel <= 4'b0000;
                RF_TSel <= 4'b0000;
            end
            
            //EXECUTION TIME
            else begin
                 if (seg_counter == 4'h2) IR_Enable <= 1'b0;
                 
                case(IROut[15:12])
                    4'h0: begin //AND +
                    if (seg_counter == 4'h2) begin
                        RF_TSel = 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 3'b111;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        RF_OutBSel <= {1'b1,IROut[1:0]};
                        ALU_FunSel <= 4'b0111; //+
                    end
                    if(seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'h1: begin //OR +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        RF_OutBSel <= {1'b1,IROut[1:0]};
                        ALU_FunSel <= 4'b1000;
                    end
                    if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'h2: begin //NOT +
                    if (seg_counter == 4'h2) begin
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        ALU_FunSel <= 4'b0010;
                    end
                    if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'h3: begin //ADD +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b0111;
                                    2'b01: RF_RSel <= 4'b1011;
                                    2'b10: RF_RSel <= 4'b1101;
                                    2'b11: RF_RSel <= 4'b1110;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b1: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b0: ARF_OutCSel <= IROut[5:4];
                        endcase
                        RF_OutBSel <= {1'b1, IROut[1:0]};
                        ALU_FunSel <= 4'b0100;
                    end
                    if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'h4: begin //SUB +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 3'b011;
                                    2'b01: ARF_RegSel <= 3'b011;
                                    2'b10: ARF_RegSel <= 3'b101;
                                    2'b11: ARF_RegSel <= 3'b110;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        RF_OutBSel <= {1'b1, IROut[1:0]};
                        ALU_FunSel <= 4'b0101;
                    end
                    // Now we have SRCREG1 - SRCREG2 in DESTREG
                    if (seg_counter == 4'h3) begin
                        MuxCSel <= IROut[10];
                        case (IROut[10])
                            1'b0: RF_OutASel <= {1'b1, IROut[9:8]};
                            1'b1: ARF_OutCSel <= IROut[9:8];
                        endcase
                        ALU_FunSel <= 4'b0010;
                    end
                    if(seg_counter == 4'h4) seg_counter <= 0;
                    end
                    4'h5: begin //LSR +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        ALU_FunSel <= 4'b1100;
                    end
                    if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'h6: begin //LSL +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        ALU_FunSel <= 4'b1010;
                    end
                    if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'h7: begin //INC +
                    if (seg_counter == 4'h2) begin
                        RF_TSel = 4'b0000;
                        case (IROut[10])
                            1'b1: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b0: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        ALU_FunSel <= 4'b0000;
                    end
                    if (seg_counter == 4'h3) begin //Increment the result
                        case (IROut[10])
                            1'b0: RF_FunSel <= 2'b11;
                            1'b1: ARF_FunSel <= 2'b11;
                        endcase
                    end
                    if (seg_counter == 4'h4) begin
                        // Additional cycle required to update the ALUOutFlag as this operation is done on the register instead of ALU.
                        ARF_RegSel <= 4'b0000;
                        ARF_OutCSel <= IROut[9:8];
                        RF_RSel <= 4'b0000;
                        RF_OutASel <= {1'b1, IROut[9:8]};
                        MuxCSel <= IROut[10];
                        ALU_FunSel <= 4'b0000;
                    end
                    if (seg_counter == 4'h5) seg_counter <= 0;
                    end
                    4'h8: begin //DEC +
                    if (seg_counter == 4'h2) begin
                        RF_TSel = 4'b0000;
                        case (IROut[10])
                            1'b1: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b0: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        ALU_FunSel <= 4'b0000;
                    end
                    if (seg_counter == 4'h3) begin //Decrement the result
                        case (IROut[10])
                            1'b0: RF_FunSel <= 2'b10;
                            1'b1: ARF_FunSel <= 2'b10;
                        endcase
                    end
                    if (seg_counter == 4'h4) begin
                        // Additional cycle required to update the ALUOutFlag as this operation is done on the register instead of ALU.
                        ARF_RegSel <= 4'b0000;
                        ARF_OutCSel <= IROut[9:8];
                        RF_RSel <= 4'b0000;
                        RF_OutASel <= {1'b1, IROut[9:8]};
                        MuxCSel <= IROut[10];
                        ALU_FunSel <= 4'b0000;
                    end
                    if (seg_counter == 4'h5) seg_counter <= 0;
                    end
                    4'h9: begin //BRA +
                        if (seg_counter == 4'h2) begin
                            RF_RSel <= 4'b0000;
                            RF_TSel <= 4'b0000;
                            ARF_RegSel <= 4'b1000;
                            ARF_FunSel <= 2'b01;
                            MuxBSel <= 2'b10;
                        end
                        if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'hA: begin //BNE +
                        if (seg_counter == 4'h2) begin
                            RF_RSel <= 4'b0000;
                            RF_TSel <= 4'b0000;
                            MuxBSel = 2'b10;
                            ARF_FunSel <= 2'b01;
                            if (current_ALU_flag[3] == 1) begin
                                ARF_RegSel <= 4'b1000;
                            end
                            else begin
                                ARF_RegSel <= 4'b0000;
                            end
                        end
                        if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'hB: begin //MOV +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        case (IROut[10])
                            1'b0: begin
                                ARF_RegSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: RF_RSel <= 4'b1000;
                                    2'b01: RF_RSel <= 4'b0100;
                                    2'b10: RF_RSel <= 4'b0010;
                                    2'b11: RF_RSel <= 4'b0001;
                                endcase
                                RF_FunSel <= 2'b01;
                            end
                            1'b1: begin
                                RF_RSel <= 4'b0000;
                                case (IROut[9:8])
                                    2'b00: ARF_RegSel <= 4'b1000;
                                    2'b01: ARF_RegSel <= 4'b0100;
                                    2'b10: ARF_RegSel <= 4'b0010;
                                    2'b11: ARF_RegSel <= 4'b0001;
                                endcase
                                ARF_FunSel <= 2'b01;
                            end
                        endcase
                        MuxASel <= 2'b00;
                        MuxBSel <= 2'b00;
                        MuxCSel <= IROut[6];
                        case (IROut[6])
                            1'b0: RF_OutASel <= {1'b1, IROut[5:4]};
                            1'b1: ARF_OutCSel <= IROut[5:4];
                        endcase
                        ALU_FunSel <= 4'b0000;
                    end
                    if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'hC: begin //LD +
                        if (seg_counter == 4'h2) begin
                            RF_TSel <= 4'b0000;
                            ARF_RegSel <= 4'b0000;
                            case (IROut[9:8])
                                2'b00: RF_RSel <= 4'b1000;
                                2'b01: RF_RSel <= 4'b0100;
                                2'b10: RF_RSel <= 4'b0010;
                                2'b11: RF_RSel <= 4'b0001;
                            endcase
                            RF_FunSel <= 2'b01;
                            case (IROut[10])
                                1'b0: MuxASel <= 2'b10; //VALUE
                                1'b1: begin //M[AR]
                                    MuxASel <= 2'b01;
                                    ARF_OutDSel <= 2'b00;
                                end
                            endcase
                            Mem_WR <= 0;
                            Mem_CS <= 0;
                        end
                        if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'hD: begin //ST +
                        if (seg_counter == 4'h2) begin
                            RF_TSel <= 4'b0000;
                            RF_RSel <= 4'b0000; 
                            ARF_RegSel <= 4'b0000;
                            ARF_OutDSel <= 2'b00;
                            RF_OutBSel <= {1'b1, IROut[9:8]};
                            ALU_FunSel <= 4'b0001;
                            Mem_WR <= 1;
                            Mem_CS <= 0;
                        end
                        if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'hE: begin //PUL +
                        if (seg_counter == 4'h2) begin
                            RF_TSel <= 4'b0000; //Disable unused registers
                            MuxASel <= 2'b01;
                            case (IROut[9:8])
                                2'b00: RF_RSel <= 4'b1000; //R1
                                2'b01: RF_RSel <= 4'b0100; //R2
                                2'b10: RF_RSel <= 4'b0010; //R3
                                2'b11: RF_RSel <= 4'b0001; //R4
                            endcase
                            
                            ARF_OutDSel <= 2'b01; //M[SP]
                            //Increment SP
                            ARF_FunSel <= 2'b11;
                            ARF_RegSel <= 4'b0010;
                            
                            RF_FunSel <= 2'b01; //Write to Rx
                            Mem_CS <= 0;
                            Mem_WR <= 0; //Read
                        end
                        if (seg_counter == 4'h3) seg_counter <= 0;
                    end
                    4'hF: begin //PSH +
                    if (seg_counter == 4'h2) begin
                        RF_TSel <= 4'b0000;
                        ARF_FunSel <= 2'b10;
                        ARF_RegSel <= 4'b0010;
                        RF_RSel <= 4'b0000;
                    end
                    if (seg_counter == 4'h3) begin
                        ALU_FunSel <= 4'b0001;
                        RF_OutBSel <= {1'b1, IROut[9:8]};
                        ARF_OutDSel <= 2'b01;
                        ARF_RegSel <= 4'b0000;
                        Mem_CS <= 0; 
                        Mem_WR <= 1; 
                    end
                    if (seg_counter == 4'h4) seg_counter <= 0;
                    end
                endcase
            end
            
        end
        
     end
    
endmodule
