`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Name: Tyler Rose 
// Project Name: Lab_7_Rose
// Date: 04/19/22
// Description: datapath top level module
//
// Note: Each model was made throughout the semester during different labs, all
// building up to this final lab. All modules are included here.
//
//////////////////////////////////////////////////////////////////////////////////

module pipelined_datapath(
    input RST,
    input CLK
    ); 
	
	wire[31:0] instruction, Regout1, Regout2, ALUout, o_PC, o_inst_mem, o_adder, IDEXOut1, IDEXOut2, IDEX_sign_out, WB_WriteData, o_sign, o_muxALU, o_memWB, o_exmem, o_memALU, o_memDat, o_wbData;
	wire[3:0] ALUctl;
	wire[1:0] mux_ALUOp, ID_ALUOp, EX_ALUOp;
	wire[4:0] MEM_WriteReg, WB_WriteReg, IDEX_rt, IDEX_rd, EX_rt, o_rd;
	wire pc_hazard, IFID_Hazard, IDEX_Hazard;
	wire mux_memRead, mux_RegDst, mux_ALUsrc, mux_mem2reg, mux_idexZero, mux_memWrite, mux_RegWrite;
	wire ID_memRead, ID_RegDst, ID_ALUsrc, ID_mem2reg, ID_idexZero, ID_memWrite, ID_RegWrite;
	wire EX_memRead, EX_RegDst, EX_ALUsrc, EX_mem2reg, EX_memWrite, EX_RegWrite;
	wire MEM_memRead, MEM_mem2reg, MEM_memWrite, MEM_RegWrite;
	wire WB_RegWrite, WB_MemtoReg; 
	
	
	PC PC(
		.nextPC(o_adder[31:0]),
		.RST(RST),
		.CLK(CLK),
		.holdPC(pc_hazard),
		.outPC(o_PC[31:0]));
		
	Adder Adder(
		.A(o_PC[31:0]),
		.B(32'h0000_0004),
		.out(o_adder[31:0]));
		
	inst_mem inst_mem(
		.i_adr(o_PC[31:0]), 
		.o_dat(o_inst_mem[31:0]));		
			
	IF_ID IFID(
		.instrIn(o_inst_mem[31:0]),
		.CLK(CLK),
		.IFIDWrite(IFID_Hazard),
		.instrOut(instruction[31:0]));
		
	HazardDetectionUnit HDU(
		.IDEXMemRead(EX_memRead),
		.EXMEMmemRead(MEM_memRead),
		.IDEX_Rt(IDEX_rt[4:0]),
		.EXMEM_Rt(EX_rt[4:0]),
		.IFID_Instr(instruction[31:0]),
		.PCWrite(pc_hazard),
		.IFIDWrite(IFID_Hazard),
		.IDEXZero(IDEX_Hazard));
		
	Control Control(
		.Op(instruction[31:26]),
		.ALUOp(mux_ALUOp[1:0]),
		.RegWrite(mux_RegWrite),
		.MemWrite(mux_memWrite),
		.MemRead(mux_memRead),
		.MemtoReg(mux_mem2reg),
		.ALUSrc(mux_ALUsrc),
		.RegDst(mux_RegDst));

	RegisterBank RegisterBank(
		.ReadData1(Regout1[31:0]),
		.ReadData2(Regout2[31:0]),
		.ReadRegister1(instruction[25:21]),
		.ReadRegister2(instruction[20:16]),
		.WriteRegister(WB_WriteReg[4:0]),
		.WriteData(WB_WriteData[31:0]),
		.RegWrite(WB_RegWrite),
		.RST(RST),
		.CLK(CLK));	
		
	SignExtend SignExtend(
		.in(instruction[15:0]),
		.out(o_sign[31:0]));
				
	ID_EX IDEX(
	   .RegWrite(mux_RegWrite),
	   .MemtoReg(mux_mem2reg),
	   .MemWrite(mux_memWrite),
	   .MemRead(mux_memRead),
	   .ALUSrc(mux_ALUsrc),
	   .RegDst(mux_RegDst),
	   .ALUOp(mux_ALUOp[1:0]),
	   .ReadData1_in(Regout1[31:0]),
	   .ReadData2_in(Regout2[31:0]),
	   .SignExtendResult_in(o_sign[31:0]),
	   .rt(instruction[20:16]),
	   .rd(instruction[15:11]),
	   .CLK(CLK),
	   .RST(RST | IDEXZero),
	   .RegWriteOut(EX_RegWrite),
	   .MemtoRegOut(EX_mem2reg),
	   .MemWriteOut(EX_memWrite),
	   .MemReadOut(EX_memRead),
	   .ALUSrcOut(EX_ALUsrc),
	   .RegDstOut(EX_RegDst),
	   .ALUOpOut(EX_ALUOp[1:0]),
	   .ReadData1_out(IDEXOut1[31:0]),
	   .ReadData2_out(IDEXOut2[31:0]),
	   .SignExtendResult_out(IDEX_sign_out[31:0]),
	   .rtOut(IDEX_rt[4:0]),
	   .rdOut(IDEX_rd[4:0]));	
		
	mux2to1EX mux2to1EX(
		.in0(IDEX_rt[4:0]),
		.in1(IDEX_rd[4:0]),
		.sel(EX_RegDst),
		.out(o_rd[4:0]));
		
	mux2to1ALU mux2to1ALU(
		.in0(IDEXOut2[31:0]),
		.in1(IDEX_sign_out[31:0]),
		.sel(EX_ALUsrc),
		.out(o_muxALU[31:0]));
	
	ALUcontrol ALUcontrol(
		.ALUOp(EX_ALUOp[1:0]),
		.F(IDEX_sign_out[5:0]),
		.Op(ALUctl[3:0]));

	ALU ALU(
		.A(IDEXOut1[31:0]),
		.B(o_muxALU[31:0]),
		.ALUctl(ALUctl[3:0]),
		.ALUOut(ALUout[31:0]),
		.Zero());
		
	EX_MEM EXMEM(
		.CLK(CLK),
		.RegWrite(EX_RegWrite),
		.MemtoReg(EX_mem2reg),
		.MemWrite(EX_memWrite),
		.MemRead(EX_memRead),
		.ALUOut(ALUout),
		.WriteData(IDEXOut2[31:0]),
		.WriteReg(o_rd[4:0]),
		.rtOut(IDEX_rt[4:0]),
		.RegWriteOut(MEM_RegWrite),
		.MemtoRegOut(MEM_mem2reg),
		.MemWriteOut(MEM_memWrite),
		.MemReadOut(MEM_memRead),
		.ALUOutOut(o_exmem[31:0]),
		.WriteDataOut(o_memDat[31:0]),
		.WriteRegOut(MEM_WriteReg[4:0]),
		.EXMEM_Rt(EX_rt[4:0]));
		
	MEM_WB MEMWB(
		.CLK(CLK),
		.RegWrite(MEM_RegWrite),
		.MemtoReg(MEM_mem2reg),
		.WriteReg(MEM_WriteReg[4:0]),
		.ALUOut(o_wbData[31:0]),
		.ReadData(o_exmem[31:0]),
		.RegWriteOut(WB_RegWrite),
		.MemtoRegOut(WB_MemtoReg),
		.ALUOutOut(o_memALU[31:0]),
		.WriteRegOut(WB_WriteReg[4:0]),
		.ReadDataOut(o_memWB[31:0]));
				
	mux2to1MEM mux2to1MEM(
		.in0(o_memWB[31:0]),
		.in1(o_memALU[31:0]),
		.sel(WB_MemtoReg),
		.out(WB_WriteData));	

	data_mem data_mem(
		.i_adr(o_exmem[31:0]),
		.i_dat(o_memDat[31:0]),
		.o_adr(o_exmem[31:0]),
		.o_dat(o_wbData[31:0]),
		.R(MEM_memRead),	
		.W(MEM_memWrite));
 */
endmodule 


// Instruction Fetch _ Instruction Decode Stage
module IF_ID(
    input [31:0] instrIn,
    input CLK, IFIDWrite,
    output reg [31:0] instrOut
    );

	 always @(posedge CLK)
    begin
      if (IFIDWrite==1'b0) 
      instrOut <= instrIn;
      else begin
          instrOut<=32'b0;
        end   
    end

endmodule


// Instruction Decode _ Execute Stage
module ID_EX(
    input RegWrite, MemtoReg, MemWrite, MemRead, ALUSrc, RegDst,
    input [1:0] ALUOp,
    input [31:0] ReadData1_in, ReadData2_in,
    input [31:0] SignExtendResult_in,
    input [4:0] rt, rd,
    input CLK, RST,
	 output reg RegWriteOut, MemtoRegOut, MemWriteOut, MemReadOut, ALUSrcOut, RegDstOut,
    output reg [1:0] ALUOpOut,
    output reg [31:0] ReadData1_out, ReadData2_out,
    output reg [31:0] SignExtendResult_out,
    output reg [4:0] rtOut, rdOut
    );

	 always @(posedge CLK)
	 begin
		if (RST)
		begin
			RegWriteOut = 1'b0;
			MemtoRegOut = 1'b0;
			MemWriteOut = 1'b0;
			MemReadOut = 1'b0;
			ALUSrcOut = 1'b0;
			RegDstOut = 1'b0;
			ALUOpOut = 1'b0;
			ReadData1_out = 32'h0000_0000;
			ReadData2_out = 32'h0000_0000;
			SignExtendResult_out = 32'h0000_0000;
			rtOut = 5'b00000;
			rdOut = 5'b00000;
		end		 
		else
		begin		
			RegWriteOut <= RegWrite;
			MemtoRegOut <= MemtoReg;
			MemWriteOut <= MemWrite;
			MemReadOut <= MemRead;
			ALUSrcOut <= ALUSrc;
			RegDstOut <= RegDst;
			ALUOpOut <= ALUOp;
			ReadData1_out <= ReadData1_in;
			ReadData2_out <= ReadData2_in;
			SignExtendResult_out <= SignExtendResult_in;
			rtOut <= rt;
			rdOut <= rd;
		end
	end

endmodule


// Execute _ Memory Access Stage
module EX_MEM(
    input CLK, RegWrite, MemtoReg, MemWrite, MemRead,
    input [31:0] ALUOut,
    input [31:0] WriteData,
    input [4:0] WriteReg, rtOut,
    output reg RegWriteOut,
    output reg MemtoRegOut,
    output reg MemWriteOut,
    output reg MemReadOut,
    output reg [31:0] ALUOutOut,
    output reg [31:0] WriteDataOut,
    output reg [4:0] WriteRegOut, EXMEM_Rt
    );

	always@(posedge CLK)
	begin
		RegWriteOut <= RegWrite;
      MemtoRegOut <= MemtoReg;
      MemWriteOut <= MemWrite;
      MemReadOut <= MemRead;
      ALUOutOut <= ALUOut;
      WriteDataOut <= WriteData;
      WriteRegOut <= WriteReg;
		EXMEM_Rt <= rtOut;
	end

endmodule


// Memory Access _ Writeback Stage
module MEM_WB(
    input CLK, RegWrite, MemtoReg,
    input [4:0] WriteReg,
    input [31:0] ALUOut,
    input [31:0] ReadData,
    output reg RegWriteOut,
    output reg MemtoRegOut,
    output reg [31:0] ALUOutOut,
    output reg [4:0] WriteRegOut,
    output reg [31:0] ReadDataOut
    );

	always@(posedge CLK)
	begin
	   RegWriteOut<=RegWrite;
      MemtoRegOut<=MemtoReg;
      ReadDataOut<=ReadData;
		ALUOutOut<=ALUOut;
      WriteRegOut<=WriteReg;
	end

endmodule


// Program Counter
module PC(
    input [31:0] nextPC,
    input RST, CLK,
    input holdPC,
    output reg [31:0] outPC
    );

	always @(posedge CLK)
	begin
		if (holdPC==0) // to support stalls from hazard detection unit
			// stall
		if (RST)
			outPC <= 32'h0000_0000;
		else
			outPC <= nextPC;
	end
endmodule


// adder
module Adder(
    input [31:0] A,
    input [31:0] B,
    output [31:0] out
    );

	assign out = A + B;

endmodule


// sign extend module
module SignExtend(in ,out);

input  [15:0] in;
output [31:0] out;

assign out =  (in[15] == 1)? {16'hffff , in} : 
              (in[15] == 0)? {16'h0000 ,in}  : 16'hxxxx;

endmodule


// mux for rt & rd
module mux2to1EX(
    input [4:0] in0, in1,
    input sel,
    output reg[4:0] out
    );

	always @(in0 or in1 or sel)
	begin
		if(sel) 
		out = in1;
		else
		out = in0;
	end

endmodule


// mux for ALU input
module mux2to1ALU(
    input [31:0] in0, in1,
    input sel,
    output reg[31:0] out
    );

	always @(in0 or in1 or sel)
	begin
		if(sel) 
		out = in1;
		else
		out = in0;
	end

endmodule


// mux for MEM/WB output
module mux2to1MEM(
    input [31:0] in0, in1,
    input sel,
    output reg[31:0] out
    );

	always @(in0 or in1 or sel)
	begin
		if(sel) 
		out = in1;
		else
		out = in0;
	end

endmodule


// Hazard Detection
module HazardDetectionUnit(
   input IDEXMemRead, EXMEMmemRead,
   input [4:0] IDEX_Rt, EXMEM_Rt,
   input [31:0] IFID_Instr,
   output PCWrite, IFIDWrite, IDEXZero
   );
	
		 assign PCWrite = ~((IDEXMemRead && (IDEX_Rt == IFID_Instr[25:21]) || (IDEX_Rt == IFID_Instr[20:16])) || (EXMEMmemRead && (EXMEM_Rt == IFID_Instr[25:21]) || (EXMEM_Rt == IFID_Instr[20:16])));
		 assign IFIDWrite = PCWrite;
		 assign IDEXZero = ~IFIDWrite;
		
endmodule


// Register Bank
module RegisterBank(
	output [31:0] ReadData1,ReadData2,
	input[4:0] ReadRegister1,ReadRegister2,WriteRegister,
	input[31:0] WriteData,
	input RegWrite,RST,CLK
	);

	wire[31:0] WriteEn;
	wire[31:0] Data[0:31];
	
	//----Decoder Block
		decoder decoder1(WriteEn,RegWrite,WriteRegister);
		genreg #(0) ZeroReg (
					.RegIn(0),
					.RST(RST),
					.CLK(CLK),
					.WriteEn(WriteEn[0]),
					.RegOut(Data[0])
				);
				 
			generate 
			genvar i;
			for(i=1;i<=31;i=i+1)begin: N // Block identifier	
				genreg #(i+5) registers (
					.RegIn(WriteData),
					.RST(RST),
					.CLK(CLK),
					.WriteEn(WriteEn[i]),
					.RegOut(Data[i])
				);
			end
		endgenerate
	
	assign ReadData1 = Data[ReadRegister1];
	assign ReadData2 = Data[ReadRegister2];
	
endmodule	


// General Register
module genreg #(parameter value = 0)(
    input [31:0] RegIn,
    input WriteEn, RST, CLK,
    output reg[31:0] RegOut
    );
	
	reg[31:0] data;
	
	always @ (posedge CLK) begin
		assign RegOut = data;    
	end		
	
	always @ (negedge CLK, posedge RST) begin
		if(RST) data <= value;
		else if (WriteEn)data <= RegIn;   
	end

endmodule


// Decoder
module decoder(
    input RegWrite,
    input [4:0] WriteRegister,
    output [31:0] WriteEn
    );

	wire [31:0] OE; // Output Enable
	
	dec5to32 dec(OE,WriteRegister);
		generate 
		genvar i;
		for(i=0;i<=31;i=i+1)begin: gate // Block identifier	
			and (WriteEn[i],OE[i],RegWrite);
		end
	endgenerate

endmodule


// Decode bits 5 through 32
module dec5to32(
    input [4:0] Adr,	// Address of register
    output [31:0] Out
    );
	
	not Inv4(Nota, Adr[4]);
	not Inv3(Notb, Adr[3]);
	not Inv2(Notc, Adr[2]);
	not Inv1(Notd, Adr[1]);
	not Inv0(Note, Adr[0]);

	assign Out[0] = (Nota & Notb & Notc & Notd & Note); // 00000
	assign Out[1] = (Nota & Notb & Notc & Notd & Adr[0]); // 00001
	assign Out[2] = (Nota & Notb & Notc & Adr[1] & Note); //00010
	assign Out[3] = (Nota & Notb & Notc & Adr[1] & Adr[0]);
	assign Out[4] = (Nota & Notb & Adr[2] & Notd & Note);
	assign Out[5] = (Nota & Notb & Adr[2] & Notd & Adr[0]);
	assign Out[6] = (Nota & Notb & Adr[2] & Adr[1] & Note);
	assign Out[7] = (Nota & Notb & Adr[2] & Adr[1] & Adr[0]);
	assign Out[8] = (Nota & Adr[3] & Notc & Notd & Note);
	assign Out[9] = (Nota & Adr[3] & Notc & Notd & Adr[0]);
	assign Out[10] = (Nota & Adr[3] & Notc & Adr[1] & Note);
	assign Out[11] = (Nota & Adr[3] & Notc & Adr[1] & Adr[0]);
	assign Out[12] = (Nota & Adr[3] & Adr[2] & Notd & Note);
	assign Out[13] = (Nota & Adr[3] & Adr[2] & Notd & Adr[0]);
	assign Out[14] = (Nota & Adr[3] & Adr[2] & Adr[1] & Note);
	assign Out[15] = (Nota & Adr[3] & Adr[2] & Adr[1] & Adr[0]);
	assign Out[16] = (Adr[4] & Notb & Notc & Notd & Note);
	assign Out[17] = (Adr[4] & Notb & Notc & Notd & Adr[0]);
	assign Out[18] = (Adr[4] & Notb & Notc & Adr[1] & Note);
	assign Out[19] = (Adr[4] & Notb & Notc & Adr[1] & Adr[0]);
	assign Out[20] = (Adr[4] & Notb & Adr[2] & Notd & Note);
	assign Out[21] = (Adr[4] & Notb & Adr[2] & Notd & Adr[0]);
	assign Out[22] = (Adr[4] & Notb & Adr[2] & Adr[1] & Note);
	assign Out[23] = (Adr[4] & Notb & Adr[2] & Adr[1] & Adr[0]);
	assign Out[24] = (Adr[4] & Adr[3] & Notc & Notd & Note);
	assign Out[25] = (Adr[4] & Adr[3] & Notc & Notd & Adr[0]);
	assign Out[26] = (Adr[4] & Adr[3] & Notc & Adr[1] & Note);
	assign Out[27] = (Adr[4] & Adr[3] & Notc & Adr[1] & Adr[0]);
	assign Out[28] = (Adr[4] & Adr[3] & Adr[2] & Notd & Note);
	assign Out[29] = (Adr[4] & Adr[3] & Adr[2] & Notd & Adr[0]);
	assign Out[30] = (Adr[4] & Adr[3] & Adr[2] & Adr[1] & Note);
	assign Out[31] = (Adr[4] & Adr[3] & Adr[2] & Adr[1] & Adr[0]); // 11111	

endmodule


// Control
module Control(
    input [5:0] Op,
    output [1:0] ALUOp,
    output RegWrite, MemWrite, MemRead, MemtoReg, ALUSrc, RegDst
    );

	wire Rformat, lw, sw;
	not (not5, Op[5]);
	not (not4, Op[4]);
	not (not3, Op[3]);
	not (not2, Op[2]);
	not (not1, Op[1]);
	not (not0, Op[0]);

	assign Rformat = (not5 & not4 & not3 & not2 & not1 & not0);
	assign lw = (Op[5] & not4 & not3 & not2 & Op[1] & Op[0]);
	assign sw = (Op[5] & not4 & Op[3] & not2 & Op[1] & Op[0]);

	assign RegDst = Rformat;
	assign ALUSrc = (lw | sw);
	assign MemtoReg = lw;
	assign RegWrite = (Rformat | lw);
	assign MemRead = lw;
	assign MemWrite = sw;
	assign ALUOp[1] = Rformat;

endmodule


// ALU Control
module ALUcontrol(
    input [1:0] ALUOp,
    input [5:0] F,
    output [3:0] Op
    );
	 
	wire or1, and1, notF1, notF2;
	
	not (notop0, ALUOp[0]);
	not (notF2, F[2]);
	not (notF1, F[1]);
	
	or (or1, F[0], F[3]);
	and (and1, F[1], ALUOp[1]);
	and (Op[0], or1, ALUOp[1]);
	and (Op[3], notop0, ALUOp[0]);
	or (Op[2], ALUOp[0], and1);
	or (Op[1], notF2, notF1);	 

endmodule


// Arithmetic Logic Unit
module ALU(
    input [31:0] A,
    input [31:0] B,
    input [31:0] ALUctl,
    output [31:0] ALUOut,
    output Zero,
    output Overflow,
    output cout
    );
	 
	wire[31:0] tempcout;
	wire zf;
	
	ALU1bit A0(A[0],B[0],ALUctl[2],ALUctl[3:0],ALUOut[0],tempcout[0]);
	ALU1bit A1(A[1],B[1],tempcout[0],ALUctl[3:0],ALUOut[1],tempcout[1]);
	ALU1bit A2(A[2],B[2],tempcout[1],ALUctl[3:0],ALUOut[2],tempcout[2]);
	ALU1bit A3(A[3],B[3],tempcout[2],ALUctl[3:0],ALUOut[3],tempcout[3]);
	ALU1bit A4(A[4],B[4],tempcout[3],ALUctl[3:0],ALUOut[4],tempcout[4]);
	ALU1bit A5(A[5],B[5],tempcout[4],ALUctl[3:0],ALUOut[5],tempcout[5]);
	ALU1bit A6(A[6],B[6],tempcout[5],ALUctl[3:0],ALUOut[6],tempcout[6]);
	ALU1bit A7(A[7],B[7],tempcout[6],ALUctl[3:0],ALUOut[7],tempcout[7]);
	ALU1bit A8(A[8],B[8],tempcout[7],ALUctl[3:0],ALUOut[8],tempcout[8]);
	ALU1bit A9(A[9],B[9],tempcout[8],ALUctl[3:0],ALUOut[9],tempcout[9]);
	ALU1bit A10(A[10],B[10],tempcout[9],ALUctl[3:0],ALUOut[10],tempcout[10]);
	ALU1bit A11(A[11],B[11],tempcout[10],ALUctl[3:0],ALUOut[11],tempcout[11]);
	ALU1bit A12(A[12],B[12],tempcout[11],ALUctl[3:0],ALUOut[12],tempcout[12]);
	ALU1bit A13(A[13],B[13],tempcout[12],ALUctl[3:0],ALUOut[13],tempcout[13]);
	ALU1bit A14(A[14],B[14],tempcout[13],ALUctl[3:0],ALUOut[14],tempcout[14]);
	ALU1bit A15(A[15],B[15],tempcout[14],ALUctl[3:0],ALUOut[15],tempcout[15]);
	ALU1bit A16(A[16],B[16],tempcout[15],ALUctl[3:0],ALUOut[16],tempcout[16]);
	ALU1bit A17(A[17],B[17],tempcout[16],ALUctl[3:0],ALUOut[17],tempcout[17]);
	ALU1bit A18(A[18],B[18],tempcout[17],ALUctl[3:0],ALUOut[18],tempcout[18]);
	ALU1bit A19(A[19],B[19],tempcout[18],ALUctl[3:0],ALUOut[19],tempcout[19]);
	ALU1bit A20(A[20],B[20],tempcout[19],ALUctl[3:0],ALUOut[20],tempcout[20]);
	ALU1bit A21(A[21],B[21],tempcout[20],ALUctl[3:0],ALUOut[21],tempcout[21]);
	ALU1bit A22(A[22],B[22],tempcout[21],ALUctl[3:0],ALUOut[22],tempcout[22]);
	ALU1bit A23(A[23],B[23],tempcout[22],ALUctl[3:0],ALUOut[23],tempcout[23]);
	ALU1bit A24(A[24],B[24],tempcout[23],ALUctl[3:0],ALUOut[24],tempcout[24]);
	ALU1bit A25(A[25],B[25],tempcout[24],ALUctl[3:0],ALUOut[25],tempcout[25]);
	ALU1bit A26(A[26],B[26],tempcout[25],ALUctl[3:0],ALUOut[26],tempcout[26]);
	ALU1bit A27(A[27],B[27],tempcout[26],ALUctl[3:0],ALUOut[27],tempcout[27]);
	ALU1bit A28(A[28],B[28],tempcout[27],ALUctl[3:0],ALUOut[28],tempcout[28]);
	ALU1bit A29(A[29],B[29],tempcout[28],ALUctl[3:0],ALUOut[29],tempcout[29]);
	ALU1bit A30(A[30],B[30],tempcout[29],ALUctl[3:0],ALUOut[30],tempcout[30]);
	ALU1bit A31(A[31],B[31],tempcout[30],ALUctl[3:0],ALUOut[31],cout);
	
	or (zf,ALUOut[0],ALUOut[1],ALUOut[2],ALUOut[3],ALUOut[4],ALUOut[5],ALUOut[6],ALUOut[7],
	ALUOut[8],ALUOut[9],ALUOut[10],ALUOut[11],ALUOut[12],ALUOut[13],ALUOut[14],ALUOut[15],
	ALUOut[16],ALUOut[17],ALUOut[18],ALUOut[19],ALUOut[20],ALUOut[21],ALUOut[22],ALUOut[23],
	ALUOut[24],ALUOut[25],ALUOut[26],ALUOut[27],ALUOut[28],ALUOut[29],ALUOut[30],ALUOut[31]);
	
	not (Zero,zf);	//zero flag
	xor (Overflow,cout,tempcout[30]); //overflow flag

endmodule


// 1 bit ALU
module ALU1bit(
    input a,
    input b,
    input cin,
    input [3:0] ALUctl,
    output ALUOut,
    output cout
    );

	Ainvert m1(a, aNot, ALUctl[3], aMux);	// Ainvert using a 2x1 mux(1)
	Binvert m2(b, bNot, ALUctl[2], bMux);	// Binvert using a 2x1 mux(2)
		
	not (aNot, a);					// not gate for a
	not (bNot, b);					// not gate for b

	and (andGate, aMux, bMux);			// and gate for mux results
	or (orGate, aMux, bMux);			// or gate for mux results
	fullAdder a1(aMux, bMux, cin, adder, cout); // adder for mux results
	
	mux_4x1 m3(andGate,orGate,adder,b,ALUctl[1:0],ALUOut); // 4:1 mux

endmodule


// Ainvert
module Ainvert(a, aNot, select, out);

	input wire a, aNot, select;	// wires
	output reg out;					// output

	always @( a or aNot or select )	// always block for sequential execution
	begin					// begin execution
		if(select)		// if select input
		out <= aNot;	// output aNot
		else				// else (self explanatory)
		out <= a;		// output a
	end					// end execution
endmodule


// Binvert
module Binvert(b, bNot, select, out);		  

	input wire b, bNot, select;	// wires
	output reg out;					// output

	always @( b or bNot or select)	// always block for sequential execution	
	begin					// begin execution
		if(select)		// if select input
		out <= bNot;	// output bNot
		else				// else (self explanatory)
		out <= b;		// output b
	end					// end execution
endmodule


// 4x1 mux
module mux_4x1(andGate, orGate, adder, transferB, select, out);	// 4x1 mux for operation control
	output reg out;												// output
	input wire[1:0] select;										// wires
	input wire andGate, orGate, adder, transferB;		// input wires
	
	always @( andGate, orGate, adder, transferB, select )	// always block
	begin										// begin execution
		case(select)						// case statement
			2'b00: out <= andGate;		// case 00, using andGate
			2'b01: out <= orGate;		// case 01, use orGate		
			2'b10: out <= adder;			// case 10, use adder
			2'b11: out <= transferB;	// case 11, use transferB
			default: out <= andGate;	// case default, use andGate
		endcase								// end case statement
	end										// end execution
endmodule


// Full Adder
module fullAdder(
    input a,
    input b,
    input cin,
    output sum,
    output cout
    );
	
	// logic 
	xor (sum,a,b,cin); 		// xor gate for sum 
	and (c1,a,b); 				// and gate for carry
	and (c2,b,cin);			// and gate for carry
	and (c3,a,cin);			// and gate for carry
	or (c4,c1,c2);				// or gate for carry
	or (cout,c4,c3);			// or gate for carry out
	

endmodule

