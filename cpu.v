
///////////////////////////////////////////////////////////////////////////
// MODULE: CPU for TSC microcomputer: cpu.v
// Author: SNU HPCS
// Description: CPU module.

// DEFINITIONS                           do not touch definition and inclue files.
`define WORD_SIZE 16            // data and address word size
`define MEMORY_SIZE 64 //2^8
// INCLUDE files
`include "opcodes.v"         // "opcode.v" consists of "define" statements for
                                // the opcodes and function codes for all instructions

// MODULE DECLARATION
 module cpu (
  input reset_cpu,                    //reset signal for CPU. acgitve-high. (rest is done when it is 1)
  input clk,                          // clock signal
  input cpu_enable,                  //enables CPU to move PC, and write register
  input wwd_enable,                 //enables wwd. if unasserted then wwd operation should not assign register value to output_port
  input [1:0] register_selection , // selects which register to show on output_port. It should only work when wwd is disabled
  //output reg[`WORD_SIZE-1:0] num_inst,   // number of instruction during execution       !!!!!!!!!!!!!!!!!!!!  importtant!!!!!!  when actual synthesize & implemention,  do not forget to disable this port !!!!!!!!!!!!!!!!!!!!!
       //You should enable num_inst port only for simulation debugging purpose. When doing synthesize and implementation, it should be disabled
  output [`WORD_SIZE-1:0] output_port, // this will be used to show values in register in case of WWD instruction or register_selection.
  output [7:0] PC_below8bit            //lower 8-bit of PC for LED output on ouput_logic.v. You need to assign lower 8bit of current PC to this port

);

  ///////////////////////////////insturction memory//////////////////////////////////////////////
    /// do not touch this part. Otherwise, your CPU will now work properly according to the tsc-ISA
    
    reg [`WORD_SIZE-1:0] memory [0:`MEMORY_SIZE - 1];  //memory where instruction is saved
/*
	 always@(reset_cpu) begin
		 if(reset_cpu == 1'b1) begin                                  //when given reset_cpu, it will be initilized as below.
			memory[0]  <= 16'h6000;	//	LHI $0, 0 (0000, 0000, 0000, 0000)
			memory[1]  <= 16'h6101;	//	LHI $1, 1 (0000, 0100, 0000, 0000)
			memory[2]  <= 16'h6202;	//	LHI $2, 2 (0000, 0100, 0200, 0000)
			memory[3]  <= 16'h6303;	//	LHI $3, 3 (0000, 0100, 0200, 0300)
			memory[4]  <= 16'hf01c;	//	WWD $0 (0000)
			memory[5]  <= 16'hf41c;	//	WWD $1 (0100)
			memory[6]  <= 16'hf81c;	//	WWD $2 (0200)
			memory[7]  <= 16'hfc1c;	//	WWD $3 (0300)
			memory[8]  <= 16'h4204;	//	ADI $2, $0, 4 (0000, 0100, 0004, 0300)
			memory[9]  <= 16'h47fc;	//	ADI $3, $1, -4 (0000, 0100, 0004, 00FC)
			memory[10] <= 16'hf81c;	//	WWD $2 (0004)
			memory[11] <= 16'hfc1c;	//	WWD $3 (00FC)
			memory[12] <= 16'hf6c0;	//	ADD $3, $1, $2 (0000, 0100, 0004, 0104)
			memory[13] <= 16'hf180;	//	ADD $2, $0, $1 (0000, 0100, 0100, 0104)
			memory[14] <= 16'hf81c;	//	WWD $2 (0100)
			memory[15] <= 16'hfc1c;	//	WWD $3 (0104)
			memory[16] <= 16'h9015;	//	JMP 21
			memory[17] <= 16'hf01c;	//	WWD $0
			memory[18] <= 16'hf180;	//	ADD $2, $0, $1
			memory[19] <= 16'hf180;	//	ADD $2, $0, $1
			memory[20] <= 16'hf180;	//	ADD $2, $0, $1 
			memory[21] <= 16'h6000;	//	LHI $0, 0 (0000, 0100, 0100, 0104)
			memory[22] <= 16'h4000;	//	ADI $0, $0, 0 (0000, 0100, 0100, 0104)
			memory[23] <= 16'hfd80;	//	ADD $2, $3, $1 (0000, 0100, 0204, 0104)
			memory[24] <= 16'hf01c;	//	WWD $0 (0000)
			memory[25] <= 16'hf41c;	//	WWD $1 (0100)
			memory[26] <= 16'hf81c;	//	WWD $2 (0204)
			memory[27] <= 16'hfc1c;	//	WWD $3 (0104)
		 end
	 end
*/

// you need to modify MEMORY_SIZE to 64!!!!! do not forget
//`define MEMORY_SIZE 64

	 always@(reset_cpu) begin   // paste this at initialing memory in cpu.v
	 if(reset_cpu == 1'b1) begin
         memory[0] <= 16'h0000; // LHI $0, 0x0 (0000, 0000, 0000, 0000)
         memory[1] <= 16'h430F; // ADI $3, $0, 0xF (0000, 0000, 0000, 000F)
         memory[2] <= 16'h6201; // LHI $2, 0x1 (0000, 0000, 0100, 000F) 
         memory[3] <= 16'h4AF0; // ADI $2, $2, 0xF0 (0000, 0000, 00F0, 000F)
         memory[4] <= 16'h610F; // LHI $1, 0xF (0000, 0F00, 00F0, 000F)
         memory[5] <= 16'h60F0; // LHI $0, 0xF0 (F000, 0F00, 00F0, 000F)
         memory[6] <= 16'hF01C; // WWD $0 (F000)
         memory[7] <= 16'hF41C; // WWD $1 (0F00)
         memory[8] <= 16'h9013; // JMP 0x13
         memory[9] <= 16'h6000; // LHI $0, 0x0 (0000, FF00, 00F0, FFFF)
         memory[10] <= 16'h6101; // LHI $1, 0x1 (0000, 0100, 00F0, FFFF)
         memory[11] <= 16'h45FF; // ADI $1, $1, 0xFF (0000, 00FF, 00F0, FFFF)
         memory[12] <= 16'h42FF; // ADI $2, $0, 0xFF (0000, 00FF, FFFF, FFFF)
         memory[13] <= 16'hF41C; // WWD $1 (00FF)
         memory[14] <= 16'hF81C; // WWD $2 (FFFF)
         memory[15] <= 16'h9010; // JMP 0x10
         memory[16] <= 16'hF5C0; // ADD $3, $1, $1 (0000, 00FF, FFFF, 01FE)
         memory[17] <= 16'hFFC0; // ADD $3, $3, $3 (0000, 00FF, FFFF, 03FC)
         memory[18] <= 16'h9021; // JMP 0x21 
         memory[19] <= 16'hF81C; // WWD $2 (00F0)
         memory[20] <= 16'hFC1C; // WWD $3 (000F)
         memory[21] <= 16'hF140; // ADD $1, $0, $1 (F000, FF00, 00F0, 000F)
         memory[22] <= 16'hFBC0; // ADD $3, $2, $3 (F000, FF00, 00F0, 00FF)
         memory[23] <= 16'hF7C0; // ADD $3, $1, $3 (F000, FF00, 00F0, FFFF)
         memory[24] <= 16'hFC1C; // WWD $3 (FFFF)
         memory[25] <= 16'h9009; // JMP 0x9 
         memory[26] <= 16'hF000; // ADD $0, $0, $0 (AAAA, 5655, 5675, 5678)
         memory[27] <= 16'hF01C; // WWD $0 (AAAA)
         memory[28] <= 16'h901D; // JMP 0x1D 
         memory[29] <= 16'h901E; // JMP 0x1E 
         memory[30] <= 16'h6202; // LHI $2, 0x2 (AAAA, 5655, 0200, 5678)
         memory[31] <= 16'h4AFF; // ADI $2, $2, 0xFF (AAAA, 5655, 01FF, 5678)
			memory[32] <= 16'h902C; // JMP 0x2C
         memory[33] <= 16'hFC1C; // WWD $3 (03FC)
         memory[34] <= 16'h6101; // LHI $1, 0x1 (0000, 0100, FFFF, 03FC)
         memory[35] <= 16'h4220; // ADI $2, $0, 0x20 (0000, 0100, 0020, 03FC)
         memory[36] <= 16'h4303; // ADI $3, $0, 0x3 (0000, 0100, 0020, 0003)
         memory[37] <= 16'h6055; // LHI $0, 0x55 (5500, 0100, 0020, 0003)
         memory[38] <= 16'h4055; // ADI $0, $0, 0x55 (5555, 0100, 0020, 0003)
         memory[39] <= 16'hF140; // ADD $1, $0, $1 (5555, 5655, 0020, 0003)
         memory[40] <= 16'hF680; // ADD $2, $1, $2 (5555, 5655, 5675, 0003)
         memory[41] <= 16'hFBC0; // ADD $3, $2, $3 (5555, 5655, 5675, 5678)
         memory[42] <= 16'hFC1C; // WWD $3 (5678)
         memory[43] <= 16'h901A; // JMP 0x1A 
         memory[44] <= 16'h4A92; // ADI $2, $2, 0x92 (AAAA, 5655, 0191, 5678)
         memory[45] <= 16'hF81C; // WWD $2 (0191)
	 end
	 end

/********************************************/
//wires and registers
	
	reg [15:0] instruction;
	reg [15:0] instruction_prev;
	
	initial begin
		instruction = memory[1];
		instruction_prev = 16'h0000;
	end 
	
   reg [15:0] pc;
	initial pc = 16'h0000;
	
	wire [15:0] r0;
	wire [15:0] r1;
	wire [15:0] r2;
	wire [15:0] r3;
	wire RegDst;
	wire Jump;
	wire ALUOp;
	wire ALUSrc;
	wire RegWrite;
	wire[1:0] reg_addr;
	wire[15:0] read1;
	wire[15:0] read2;
	wire alu_func;
	wire[15:0] alu_input2;
	wire[15:0] add_result;
	wire[15:0] jump_pc;
	wire[15:0] register_out;
	
	reg jumped;
	initial jumped = 0;
	
	reg [15:0] tmppc;
	
/********************************************/
//hierarchial structures - connecting module instance ports by name
	
	control ctrl(
	.func(instruction[5:0]),
	.opcode(instruction[15:12]),
	.RegDest(RegDst),
	.ALUSrc(ALUSrc),
	.RegWrite(RegWrite),
	.Jump(Jump),
	.ALUOp(ALUOp)
	);
	
	reg_MUX regmux(
	.RegDst(RegDst),
	.inst1(instruction[9:8]),
	.inst2(instruction[7:6]),
	.write_reg_address(reg_addr) //output
	);

	register rf(
	.pc(pc),
	.RegWrite(RegWrite),
	.cpu_enable(cpu_enable),
	.RST(reset_cpu), 
	.clk(clk),
	.read_reg1(instruction[11:10]),
	.read_reg2(instruction[9:8]),
	.write_reg(reg_addr),
	.write_data(add_result),
	.data1(read1),
	.data2(read2),
	.r0(r0),
	.r1(r1),
	.r2(r2),
	.r3(r3)
	);
	
	ALU_Control alu_ctrl(
	.ALUOp(ALUOp),
	.inst(instruction[5:0]),
	.ALU_func(alu_func) //output 
	);
	
	ALU_MUX alu_mux(
	.ALUSrc(ALUSrc),
	.read_data2(read2),
	.opcode(instruction[15:12]),
	.instr(instruction[7:0]),
	.ALU_input2(alu_input2)
	);
	
	
	ALU alu(
	.I1(read1),
	.I2(alu_input2),
	.opcode(instruction[15:12]),
	.ALU_control(alu_func),
	.ALU_result(add_result) //output
	);
	
	JMP_MUX jump_mux(
	.JumpControl(Jump),
	.target(instruction[11:0]),
	.pc_4bit(pc[15:12]),
	.jump_pc(jump_pc) //output
	);

	register_memory regfetch(
	.clk(clk),
	.WWD_enable(wwd_enable),
	.instruction(instruction_prev),
	.register_selection(register_selection),
	.r0(r0),
	.r1(r1),
	.r2(r2),
	.r3(r3),
	.RST(reset_cpu),
	.register(register_out) //output
	);
	
/********************************************/
//advancing PC
	
	always @(posedge clk) begin
		if(reset_cpu) begin 
			pc = 16'h0000;
			instruction_prev = memory[0];
			instruction = memory[1];
		end
		else if (cpu_enable && Jump ) begin
			if (jumped == 0) begin
				pc = pc+1;
				instruction_prev = memory[pc];
				instruction = memory[jump_pc];
				jumped = 1;
				tmppc = jump_pc;
			end
			else begin
				pc = tmppc;
				instruction_prev = memory[pc];
				instruction = memory[jump_pc];
				jumped = 1;
				tmppc = jump_pc;
			end
		end
		else if (cpu_enable) begin 
			if(jumped == 0) begin
				pc = pc + 1;
				instruction_prev = memory[pc];
				instruction = memory[pc+1];
			end
			else begin
				pc = tmppc;
				instruction_prev = memory[pc];
				instruction = memory[pc+1];
				jumped = 0;
			end
		end
	end

	assign output_port = register_out;
	assign PC_below8bit = pc[7:0];

endmodule

/*******************************************/
//register memory out module

module register_memory(
	input clk,
	input WWD_enable,
	input [15:0] instruction,
	input [1:0] register_selection,
	input [15:0] r0,
	input [15:0] r1, 
	input [15:0] r2,
	input [15:0] r3,
	input RST,
	output reg [15:0] register
);
	always @(posedge clk) begin
		if(RST) assign register = 16'h0000;
		if(WWD_enable && (instruction[15:12] == 4'b1111) && (instruction[5:0] == 6'b011100)) begin //WWD_EN is ON & WWD instruction
			if(instruction[11:10]==2'b00) assign register = r0;
			else if(instruction[11:10]==2'b01) assign register = r1;
			else if(instruction[11:10]==2'b10) assign register = r2;
			else if(instruction[11:10]==2'b11)  assign register = r3;
		end
		else if(register_selection==2'b00) assign register = r0; 
		else if(register_selection==2'b01) assign register = r1;
		else if(register_selection==2'b10) assign register = r2;
		else if(register_selection==2'b11)  assign register = r3;
	end
endmodule

/********************************************/
//multiplexor for register module

module reg_MUX(
	input RegDst, //control signal
	input [1:0] inst1,
	input [1:0] inst2,
	output reg[1:0] write_reg_address
);
	reg[1:0] r1 = 0;
	reg[1:0] r2 = 0;
always @(*) begin
	r1=inst1;
	r2=inst2;
	if(RegDst==0) write_reg_address <= r1;
	else write_reg_address <= r2;
end 
endmodule

/********************************************/
//register module

module register(
	input [15:0] pc,
	input RegWrite, //control signal
	input cpu_enable,
	input RST, clk,
	input [1:0] read_reg1,
	input [1:0] read_reg2,
	input [1:0] write_reg,
	input [15:0] write_data,
	output [15:0] data1,
	output [15:0] data2,
	output [15:0] r0,
	output [15:0] r1,
	output [15:0] r2,
	output [15:0] r3
);

	reg[15:0] memory[3:0];
	
	always @(posedge clk) begin
		if(RST) begin //set all the register to 0.
			memory[0] <= 16'h0000;
			memory[1] <= 16'h0000;
			memory[2] <= 16'h0000;
			memory[3] <= 16'h0000;
		end
		else if(cpu_enable && RegWrite) begin //ADD, ADI 
			memory[write_reg] <= write_data;
		end
	end
	
assign	data1 = memory[read_reg1];
assign	data2 = memory[read_reg2];
		assign r0 = memory[0];
		assign r1 = memory[1];
		assign r2 = memory[2];
		assign r3 = memory[3];
	

endmodule

/**********************************************************/
//control flow module

module control(
	input [15:12] opcode,
	input [5:0] func,
	output reg RegDest,
	output reg ALUSrc,
	output reg RegWrite,
	output reg Jump,
	output reg ALUOp
);
	always @(*) begin
		case(opcode)
			4'b1111: begin //R type ADD or I type WWD
				if(func==6'b011100) begin //WWD instruction
					ALUSrc = 0;
					RegWrite = 0;
					RegDest = 1;
					Jump = 0;
					ALUOp = 0;
				end
				else begin
					ALUSrc = 0;
					RegWrite = 1;
					RegDest = 1;
					Jump = 0;
					ALUOp = 1;
				end
			end
			4'b0100: begin //I type ADI
				ALUSrc = 1;
				RegWrite = 1;
				RegDest = 0;
				Jump = 0;
				ALUOp = 1;
			end
			4'b0110: begin //I type LHI
				ALUSrc = 1;
				RegWrite = 1;
				RegDest = 0;
				Jump = 0;
				ALUOp = 1;
			end
			4'b1001: begin //J type JMP
				RegWrite = 0; 
				Jump = 1;
				ALUOp = 0;
			end
		endcase		
	end
endmodule

/************************************************/
//ALU control module

module ALU_Control(
input ALUOp, //control signal
input [5:0] inst,
output reg ALU_func
);

	always @(*) begin 
		if(ALUOp==0) ALU_func = 0; //JMP
		else if(inst==6'b011100) ALU_func = 0; //WWD
		else ALU_func = 1;
	end	
endmodule

/*****************************************/
//multiplexor for Jump module

module JMP_MUX (
	input JumpControl, //control signal
	input [11:0] target,
	input [15:12] pc_4bit,
	output [15:0] jump_pc
);
	reg[15:0] tmp;
	always @(*) begin
		if(JumpControl==1) tmp <= {pc_4bit, target};
	end
	assign jump_pc = tmp;
endmodule

/*****************************************/
//multiplexor for ALU module

module ALU_MUX(
	input ALUSrc, //control signal
	input [15:0] read_data2,
	input [7:0] instr,
	input [3:0] opcode,
	output reg [15:0] ALU_input2
);
	always @ (*) begin
		if(ALUSrc == 0) ALU_input2 <= read_data2; //ADD or WWD
		else if(opcode==4'b0100) ALU_input2 <= { {8{instr[7]}}, instr }; //ADI
		else if(opcode==4'b0110) ALU_input2 <= {instr, 8'b00000000}; //LHI
	end
endmodule

/*****************************************/
//module for ALU

module ALU(
	input [15:0] I1,
	input [15:0] I2,
	input ALU_control, //control signal
	input [3:0] opcode,
	input cpu_enable,
	output reg [15:0] ALU_result
);

	always @(*) begin
		if(ALU_control && (opcode == 4'b0110)) ALU_result <= I2; //LHI
		else if(ALU_control && opcode==4'b1111) ALU_result <= I1 + I2; //ADD, WWD
		else if(ALU_control && opcode==4'b0100) ALU_result <= I1 + I2; //ADI
	end
endmodule
