`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/02/2023 12:47:20 AM
// Design Name: 
// Module Name: modules_test
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


//module test1 ();
//    parameter N = 8;

//    reg [0:0] clk;
//    reg [0:0] enable;
//    reg [N-1:0] inp;
//    reg [1:0] funSel;
    
//    wire [N-1:0] out;
    
//    initial clk = 0;
    
//    always #50 clk = ~clk;
    
//    part1 #(N) uut(.clk(clk), .en(enable), .inp(inp), .funSel(funSel), .out(out));
    
//    initial begin
//        inp = 8'b01010101; funSel = 2'b00; enable = 1'b0; #100;
//        inp = 8'b01010101; funSel = 2'b00; enable = 1'b1; #100;
//        inp = 8'b01010101; funSel = 2'b01; enable = 1'b1; #100;
//        inp = 8'b01010101; funSel = 2'b10; enable = 1'b1; #100;
//        inp = 8'b01010101; funSel = 2'b11; enable = 1'b1; #100;
//    end
//endmodule



//module part2_a_test();
//    reg [7:0] inp;
//    reg [0:0] clk;
//    reg [0:0] enable;
//    reg [1:0] funSel;
//    reg [0:0] lh;
    
//    wire [15:0] out;
    
//    initial clk = 0;
//    always #50 clk = ~clk;
    
//    part2_a uut(.clk(clk), .en(enable), .inp(inp), .funSel(funSel), .out(out), .lh(lh));
    
//    initial begin
//        inp = 8'b01010101; funSel = 2'b00; enable = 1'b0; lh = 1'b0; #150;
//        inp = 8'b01010101; funSel = 2'b00; enable = 1'b1; lh = 1'b0; #150;
//        inp = 8'b01010101; funSel = 2'b01; enable = 1'b1; lh = 1'b1; #150;
//        inp = 8'b01010101; funSel = 2'b01; enable = 1'b1; lh = 1'b0; #150;
//        inp = 8'b01010101; funSel = 2'b10; enable = 1'b1; lh = 1'b0; #150;
//        inp = 8'b01010101; funSel = 2'b11; enable = 1'b1; lh = 1'b1; #150;
//    end
//endmodule



//module part2b_test();
//    reg [0:0] clk;
//    reg [7:0] inp;
//    reg [2:0] o1Sel, o2Sel;
//    reg [1:0] funSel;
//    reg [3:0] rSel, tSel;
    
//    wire [7:0] o1;
//    wire [7:0] o2;
    
//    initial clk = 0;
//    always #45 clk = ~clk;
    
//    part_2b uut(.clk(clk), .inp(inp), .o1Sel(o1Sel), .o2Sel(o2Sel), .funSel(funSel), .rSel(rSel), .tSel(tSel), .o1(o1), .o2(o2));
    
//    initial begin
//        inp = 8'b01010101; o1Sel = 3'b000; o2Sel = 3'b100; rSel = 4'b1111; tSel = 4'b1111; funSel = 2'b00; #90;
        
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b011; rSel = 4'b0001; tSel = 4'b0001; funSel = 2'b01; #90; 
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b011; rSel = 4'b0001; tSel = 4'b0001; funSel = 2'b10; #90; 
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b011; rSel = 4'b0001; tSel = 4'b0001; funSel = 2'b11; #90;
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b011; rSel = 4'b0001; tSel = 4'b0001; funSel = 2'b00; #90;
        
//        inp = 8'b01010101; o1Sel = 3'b100; o2Sel = 3'b100; rSel = 4'b0001; tSel = 4'b0100; funSel = 2'b10; #90; 
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b100; rSel = 4'b0000; tSel = 4'b0100; funSel = 2'b01; #90;
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b100; rSel = 4'b0000; tSel = 4'b0100; funSel = 2'b10; #90;
//        inp = 8'b01010101; o1Sel = 3'b100; o2Sel = 3'b100; rSel = 4'b0001; tSel = 4'b0100; funSel = 2'b11; #90;
        
//        inp = 8'b01010101; o1Sel = 3'b000; o2Sel = 3'b000; rSel = 4'b1000; tSel = 4'b1000; funSel = 2'b00; #90; 
//        inp = 8'b01010101; o1Sel = 3'b001; o2Sel = 3'b001; rSel = 4'b1000; tSel = 4'b1000; funSel = 2'b01; #90; 
//        inp = 8'b01010101; o1Sel = 3'b010; o2Sel = 3'b010; rSel = 4'b1010; tSel = 4'b1010; funSel = 2'b10; #90; 
//        inp = 8'b01010101; o1Sel = 3'b011; o2Sel = 3'b011; rSel = 4'b1011; tSel = 4'b1010; funSel = 2'b11; #90;
        
//        inp = 8'b01010101; o1Sel = 3'b100; o2Sel = 3'b100; rSel = 4'b1100; tSel = 4'b1100; funSel = 2'b00; #90; 
//        inp = 8'b01010101; o1Sel = 3'b101; o2Sel = 3'b101; rSel = 4'b1100; tSel = 4'b1100; funSel = 2'b01; #90; 
//        inp = 8'b01010101; o1Sel = 3'b110; o2Sel = 3'b110; rSel = 4'b1110; tSel = 4'b1110; funSel = 2'b10; #90; 
//        inp = 8'b01010101; o1Sel = 3'b111; o2Sel = 3'b011; rSel = 4'b1110; tSel = 4'b1110; funSel = 2'b11; #90; 
//    end
//endmodule



//module part2c_test();
//    reg [0:0] clk;
//    reg [7:0] inp;
//    reg [1:0] oASel, oBSel;
//    reg [1:0] funSel;
//    reg [3:0] rSel;
    
//    wire [7:0] oA, oB;
    
//    initial clk = 0;
//    always #50 clk = ~clk;
    
//    part_2c uut(.clk(clk), .inp(inp), .oASel(oASel), .oBSel(oBSel), .funSel(funSel), .rSel(rSel), .oA(oA), .oB(oB));
    
//    initial begin
//        inp = 8'b01010101; oASel = 2'b00; oBSel = 2'b00; funSel = 2'b00; rSel = 4'b1111; #100; 
//        inp = 8'b01010101; oASel = 2'b00; oBSel = 2'b00; funSel = 2'b01; rSel = 4'b1111; #100; 
//        inp = 8'b01010101; oASel = 2'b00; oBSel = 2'b00; funSel = 2'b10; rSel = 4'b1111; #100;  
//        inp = 8'b01010101; oASel = 2'b00; oBSel = 2'b00; funSel = 2'b11; rSel = 4'b1111; #100;
//        inp = 8'b01010101; oASel = 2'b01; oBSel = 2'b01; funSel = 2'b00; rSel = 4'b0100; #100;
//        inp = 8'b01010101; oASel = 2'b10; oBSel = 2'b10; funSel = 2'b00; rSel = 4'b0000; #100;
//        inp = 8'b01010101; oASel = 2'b11; oBSel = 2'b11; funSel = 2'b00; rSel = 4'b0000; #100;
//        inp = 8'b01010101; oASel = 2'b00; oBSel = 2'b00; funSel = 2'b00; rSel = 4'b0000; #100;
//    end
//endmodule



//module adder_1bit_test();
//    reg [0:0] A, B, cin;
//    wire [0:0] Sum, carry;
    
//    adder1bit uut(.A(A), .B(B), .cin(cin), .Sum(Sum), .carry(carry));
    
//    initial begin
//        A = 1'b0; B = 1'b0; cin = 1'b0; #125;
//        A = 1'b0; B = 1'b0; cin = 1'b1; #125;
//        A = 1'b0; B = 1'b1; cin = 1'b0; #125;
//        A = 1'b0; B = 1'b1; cin = 1'b1; #125;
//        A = 1'b1; B = 1'b0; cin = 1'b0; #125;
//        A = 1'b1; B = 1'b0; cin = 1'b1; #125;
//        A = 1'b1; B = 1'b1; cin = 1'b0; #125;
//        A = 1'b1; B = 1'b1; cin = 1'b1; #125;
//    end
//endmodule



//module fullAdder_8bit_test();
//    reg [7:0] A,B;
//    reg [0:0] Cin;
//    wire [7:0] Sum;
//    wire [0:0] carry, overflow, zero;
    
//    full_adder_8bit uut(.A(A), .B(B), .Cin(Cin), .Sum(Sum), .carry(carry), .overflow(overflow), .zero(zero));
    
//    initial begin
//        A = 8'b01010101; B = 8'b01010101; Cin = 1'b0; #150; // 85 + 55  = 170
//        A = 8'b00000001; B = 8'b11111111; Cin = 1'b0; #150;// 1 + -1 = 0
//        A = 8'b00000000; B = 8'b00000000; Cin = 1'b0; #150;// 0 + 0 = 0
//        A = 8'b00000000; B = 8'b00000000; Cin = 1'b1; #150;// 0 + 0 + 1 = 1
//        A = 8'b01111111; B = 8'b00000001; Cin = 1'b0; #150;// 127 + 1 = -128 O 
//        A = 8'b10000000; B = 8'b11111111; Cin = 1'b0; #150;// -128 + -1 = 127
//        A = 8'b00000100; B = 8'b00000100; Cin = 1'b0; #150;// 4 + 4 = 8
//    end
//endmodule



//module subtracter_test();
//    reg [7:0] A, B;
//    wire [0:0] carry, overflow, zero;
//    wire [7:0] result;
    
//    subtracter8bit uut(.A(A), .B(B), .carry(carry), .overflow(overflow), .zero(zero), .result(result));
    
//    initial begin
//        A = 8'b00000001; B = 8'b11111111; #100; // 1 - -1
//        A = 8'b00000101; B = 8'b11111111; #100; // 5 - -1
//        A = 8'b00010100; B = 8'b00101000; #100; // 20 - 40
//        A = 8'b00000101; B = 8'b00000001; #100; // 5 - 1
//        A = 8'b00000000; B = 8'b00000000; #100;
//        A = 8'b01111111; B = 8'b11111111; #100;
//        A = 8'b11111101; B = 8'b01111111; #100;
//    end
//endmodule


/*
module compare_test();
    reg [7:0] A,B;
    wire [7:0] out;
    wire [0:0] zero, overflow, carry;
    
    compare uut(.inp_A(A), .inp_B(B), .result(out), .zero_flag(zero), .overflow_flag(overflow), .carry_flag(carry));
    
    initial begin
        A = 8'b00000000; B = 8'b00000000; #125;
        A = 8'b00000000; B = 8'b00000001; #125;
        A = 8'b00000001; B = 8'b00000000; #125;
        A = 8'b00000001; B = 8'b11111111; #125;
        A = 8'b01111111; B = 8'b11111111; #100;
        A = 8'b11111101; B = 8'b01111111; #100;
    end
endmodule
*/

/*
module asr_test();
    reg [7:0] A;
    wire [7:0] out;
    wire [0:0] zero;
    
    ASR uut(.inp_A(A), .outALU(out), .zero_flag(zero));
    
    initial begin
        A = 8'b10011100; #200;
        A = 8'b00000000; #200;
        A = 8'b01010101; #200;
    end
endmodule
*/

/*
module asl_test();
    reg [7:0] A;
    wire [7:0] out;
    wire [0:0] zero, overflow;
    
    ASL uut(.inp_A(A), .outALU(out), .zero_flag(zero), .overflow_flag(overflow));
    
    initial begin
        A = 8'b10011101; #200;
        A = 8'b11000111; #200;
        A = 8'b00011111; #200;
        A = 8'b10000000; #200;
        A = 8'b00000000; #200;
    end
endmodule
*/

/*
module lsr_test();
    reg [7:0] A;
    wire [7:0] out;
    wire [0:0] zero, carry;
    
    LSR uut(.inp_A(A), .outALU(out), .zero_flag(zero), .carry_flag(carry));
    
    initial begin
        A = 8'b00000001; #100;
        A = 8'b00000000; #100;
        A = 8'b10101010; #100;
        A = 8'b11011011; #125;
    end
endmodule
*/

/*
module lsl_test();
    reg [7:0] A;
    wire [7:0] out;
    wire [0:0] zero, carry;
    
    LSL uut(.inp_A(A), .outALU(out), .zero_flag(zero), .carry_flag(carry));
    
    initial begin
        A = 8'b11011011; #125;
        A = 8'b10000000; #125;
        A = 8'b00000000; #125;
        A = 8'b01110110; #125;
    end
endmodule
*/

/*
module csr_test();
    reg [7:0] A;
    reg [0:0] cin;
    wire [7:0] out;
    wire [0:0] zero, carry;
    
    CSR uut(.inp_A(A), .c_in(cin),  .outALU(out), .zero_flag(zero), .carry_flag(carry));
    
    initial begin
        A = 8'b00000000; cin = 1'b0; #125;
        A = 8'b10011001; cin = 1'b1; #125;
        A = 8'b00011010; cin = 1'b1; #125;
        A = 8'b11110010; cin = 1'b0; #125;
    end
endmodule
*/


//module part3_test();
//    reg [0:0] clk;
//    reg [7:0] A,B;
//    reg [3:0] funSel;
//    wire [7:0] out;
//    wire [3:0] znoc;
    
//    initial clk = 0;
//    always #25 clk = ~clk;
    
//    part_3 uut(.clk(clk), .inp_A(A), .inp_B(B), .funSel(funSel), .outALU(out), .flags(znoc));
    
//    initial begin
//        funSel = 4'b0000; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0001; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0010; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0011; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0100; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0101; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0110; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b0111; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1000; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1001; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1010; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1011; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1100; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1101; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1110; A = 8'b01001111; B = 8'b11110000; #62.5;
//        funSel = 4'b1111; A = 8'b01001111; B = 8'b11110000; #62.5;
//    end
//endmodule



module Project1Test();
    //Input Registers of ALUSystem
    reg[2:0] RF_O1Sel; 
    reg[2:0] RF_O2Sel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RSel;
    reg[3:0] RF_TSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutASel; 
    reg[1:0] ARF_OutBSel; 
    reg[1:0] ARF_FunSel;
    reg[3:0] ARF_RSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0]      IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg MuxCSel;
    reg      Clock;
    
    wire[7:0] Potato;
    
    //Test Bench Connection of ALU System
    ALU_System _ALUSystem(
    .RF_OutASel(RF_O1Sel), 
    .RF_OutBSel(RF_O2Sel), 
    .RF_FunSel(RF_FunSel),
    .RF_RSel(RF_RSel),
    .RF_TSel(RF_TSel),
    .ALU_FunSel(ALU_FunSel),
    .ARF_OutCSel(ARF_OutASel), 
    .ARF_OutDSel(ARF_OutBSel), 
    .ARF_FunSel(ARF_FunSel),
    .ARF_RegSel(ARF_RSel),
    .IR_LH(IR_LH),
    .IR_Enable(IR_Enable),
    .IR_Funsel(IR_Funsel),
    .Mem_WR(Mem_WR),
    .Mem_CS(Mem_CS),
    .MuxASel(MuxASel),
    .MuxBSel(MuxBSel),
    .MuxCSel(MuxCSel),
    .Clock(Clock),
    .MemoryOut(Potato)
    );
    
    //Test Vector Variables
    reg [41:0] VectorNum, Errors, TotalLine; 
    reg [41:0] TestVectors[3:0];
    reg Reset, Operation;
    initial begin
        Reset = 0;
    end
    //Clock Signal Generation
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    
    //Read Test Bench Values
    initial begin
        $readmemb("TestBench.mem", TestVectors); // Read vectors
        VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
    end
    
    // Apply test vectors on rising edge of clock
    always @(posedge Clock)
    begin
        #1; 
        {Operation, RF_O1Sel, RF_O2Sel, RF_FunSel, 
        RF_RSel, RF_TSel, ALU_FunSel, ARF_OutASel, ARF_OutBSel, 
        ARF_FunSel, ARF_RSel, IR_LH, IR_Enable, IR_Funsel, 
        Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
    end
    
    // Check results on falling edge of clk
    always @(negedge Clock)
        if (~Reset) // skip during reset
        begin
            $display("Input Values:");
            $display("Operation: %d", Operation);
            $display("Register File: O1Sel: %d, O2Sel: %d, FunSel: %d, RSel: %d, TSel: %d", RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel);            
            $display("ALU FunSel: %d", ALU_FunSel);
            $display("Addres Register File: OutASel: %d, OutBSel: %d, FunSel: %d, Regsel: %d", ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel);            
            $display("Instruction Register: LH: %d, Enable: %d, FunSel: %d", IR_LH, IR_Enable, IR_Funsel);            
            $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
            $display("MuxASel: %d, MuxBSel: %d, MuxCSel: %d", MuxASel, MuxBSel, MuxCSel);
            
            $display("");
            $display("Output Values:");
            $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
            $display("ALUOut: %d, ALUOutFlag: %d, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
            $display("Address Register File: AOut: %d, BOut (Address): %d", _ALUSystem.AOut, _ALUSystem.Address);            
            $display("Memory Out: %d", _ALUSystem.MemoryOut);            
            $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
            $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
            // increment array index and read next testvector
            VectorNum = VectorNum + 1;
            if (TestVectors[VectorNum] === 42'bx)
            begin
                $display("%d tests completed.",
                VectorNum);
                $finish; // End simulation
            end
        end
endmodule

module Project2Test();
    reg Clock, Reset;
    wire [7:0] T;
    
    initial Reset = 1'b0;
    
    always 
    begin
        Clock = 1; #5; Clock = 0; #5; // 10ns period
    end
    CPUSystem _CPUSystem( 
            .Clock(Clock),
            .Reset(Reset),
            .T(T)    
        );
endmodule