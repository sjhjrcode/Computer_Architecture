// riscvpipelined.sv

// RISC-V pipelined processor
// From Section 7.6 of Digital Design & Computer Architecture: RISC-V Edition
// 27 April 2020
// David_Harris@hmc.edu 
// Sarah.Harris@unlv.edu

// run 210
// Expect simulator to print "Simulation succeeded"
// when the value 25 (0x19) is written to address 100 (0x64)

// Pipelined implementation of RISC-V (RV32I)
// User-level Instruction Set Architecture V2.2 (May 7, 2017)
// Implements a subset of the base integer instructions:
//    lw, sw
//    add, sub, and, or, slt, 
//    addi, andi, ori, slti
//    beq
//    jal
// Exceptions, traps, and interrupts not implemented
// little-endian memory

// 31 32-bit registers x1-x31, x0 hardwired to 0
// R-Type instructions
//   add, sub, and, or, slt
//   INSTR rd, rs1, rs2
//   Instr[31:25] = funct7 (funct7b5 & opb5 = 1 for sub, 0 for others)
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// I-Type Instructions
//   lw, I-type ALU (addi, andi, ori, slti)
//   lw:         INSTR rd, imm(rs1)
//   I-type ALU: INSTR rd, rs1, imm (12-bit signed)
//   Instr[31:20] = imm[11:0]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode
// S-Type Instruction
//   sw rs2, imm(rs1) (store rs2 into address specified by rs1 + immm)
//   Instr[31:25] = imm[11:5] (offset[11:5])
//   Instr[24:20] = rs2 (src)
//   Instr[19:15] = rs1 (base)
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:0]  (offset[4:0])
//   Instr[6:0]   = opcode
// B-Type Instruction
//   beq rs1, rs2, imm (PCTarget = PC + (signed imm x 2))
//   Instr[31:25] = imm[12], imm[10:5]
//   Instr[24:20] = rs2
//   Instr[19:15] = rs1
//   Instr[14:12] = funct3
//   Instr[11:7]  = imm[4:1], imm[11]
//   Instr[6:0]   = opcode
// J-Type Instruction
//   jal rd, imm  (signed imm is multiplied by 2 and added to PC, rd = PC+4)
//   Instr[31:12] = imm[20], imm[10:1], imm[11], imm[19:12]
//   Instr[11:7]  = rd
//   Instr[6:0]   = opcode

//   Instruction  opcode    funct3    funct7
//   add          0110011   000       0000000
//   sub          0110011   000       0100000
//   and          0110011   111       0000000
//   or           0110011   110       0000000
//   slt          0110011   010       0000000
//   addi         0010011   000       immediate
//   andi         0010011   111       immediate
//   ori          0010011   110       immediate
//   slti         0010011   010       immediate
//   beq          1100011   000       immediate
//   lw	          0000011   010       immediate
//   sw           0100011   010       immediate
//   jal          1101111   immediate immediate




// constants defining different privilege modes
// defined in Table 1.1 of the privileged spec
`define M_MODE (2'b11)
`define S_MODE (2'b01)
`define U_MODE (2'b00)

// Virtual Memory Constants
`define VPN_SEGMENT_BITS (`XLEN == 32 ? 10 : 9)
`define VPN_BITS (`XLEN==32 ? (2*`VPN_SEGMENT_BITS) : (4*`VPN_SEGMENT_BITS))
`define PPN_BITS (`XLEN==32 ? 22 : 44)
`define PA_BITS (`XLEN==32 ? 34 : 56)
`define SVMODE_BITS (`XLEN==32 ? 1 : 4)
`define ASID_BASE (`XLEN==32 ? 22 : 44)
`define ASID_BITS (`XLEN==32 ? 9 : 16)

// constants to check SATP_MODE against
// defined in Table 4.3 of the privileged spec
`define NO_TRANSLATE 0
`define SV32 1
`define SV39 8
`define SV48 9

// macros to define supported modes
`define A_SUPPORTED ((`MISA >> 0) % 2 == 1)
`define B_SUPPORTED ((`ZBA_SUPPORTED | `ZBB_SUPPORTED | `ZBC_SUPPORTED | `ZBS_SUPPORTED)) // not based on MISA
`define C_SUPPORTED ((`MISA >> 2) % 2 == 1)
`define D_SUPPORTED ((`MISA >> 3) % 2 == 1)
`define E_SUPPORTED ((`MISA >> 4) % 2 == 1)
`define F_SUPPORTED ((`MISA >> 5) % 2 == 1)
`define I_SUPPORTED ((`MISA >> 8) % 2 == 1)
`define M_SUPPORTED ((`MISA >> 12) % 2 == 1)
`define Q_SUPPORTED ((`MISA >> 16) % 2 == 1)
`define S_SUPPORTED ((`MISA >> 18) % 2 == 1)
`define U_SUPPORTED ((`MISA >> 20) % 2 == 1)
// N-mode user-level interrupts are depricated per Andrew Waterman 1/13/21

// logarithm of XLEN, used for number of index bits to select
`define LOG_XLEN (`XLEN == 32 ? 5 : 6)

// Number of 64 bit PMP Configuration Register entries (or pairs of 32 bit entries)
`define PMPCFG_ENTRIES (`PMP_ENTRIES/8)

// Floating point constants for Quad, Double, Single, and Half precisions
`define Q_LEN 32'd128
`define Q_NE 32'd15
`define Q_NF 32'd112
`define Q_BIAS 32'd16383
`define Q_FMT 2'd3
`define D_LEN 32'd64
`define D_NE 32'd11
`define D_NF 32'd52
`define D_BIAS 32'd1023
`define D_FMT 2'd1
`define S_LEN 32'd32
`define S_NE 32'd8
`define S_NF 32'd23
`define S_BIAS 32'd127
`define S_FMT 2'd0
`define H_LEN 32'd16
`define H_NE 32'd5
`define H_NF 32'd10
`define H_BIAS 32'd15
`define H_FMT 2'd2

// Floating point length FLEN and number of exponent (NE) and fraction (NF) bits
`define FLEN (`Q_SUPPORTED ? `Q_LEN  : `D_SUPPORTED ? `D_LEN  : `S_LEN)
`define NE   (`Q_SUPPORTED ? `Q_NE   : `D_SUPPORTED ? `D_NE   : `S_NE)
`define NF   (`Q_SUPPORTED ? `Q_NF   : `D_SUPPORTED ? `D_NF   : `S_NF)
`define FMT  (`Q_SUPPORTED ? 2'd3    : `D_SUPPORTED ? 2'd1    : 2'd0)
`define BIAS (`Q_SUPPORTED ? `Q_BIAS : `D_SUPPORTED ? `D_BIAS : `S_BIAS)
/* Delete once tested dh 10/10/22

`define FLEN (`Q_SUPPORTED ? `Q_LEN  : `D_SUPPORTED ? `D_LEN  : `F_SUPPORTED ? `S_LEN  : `H_LEN)
`define NE   (`Q_SUPPORTED ? `Q_NE   : `D_SUPPORTED ? `D_NE   : `F_SUPPORTED ? `S_NE   : `H_NE)
`define NF   (`Q_SUPPORTED ? `Q_NF   : `D_SUPPORTED ? `D_NF   : `F_SUPPORTED ? `S_NF   : `H_NF) 
`define FMT  (`Q_SUPPORTED ? 2'd3       : `D_SUPPORTED ? 2'd1       : `F_SUPPORTED ? 2'd0       : 2'd2)
`define BIAS (`Q_SUPPORTED ? `Q_BIAS : `D_SUPPORTED ? `D_BIAS : `F_SUPPORTED ? `S_BIAS : `H_BIAS)*/

// Floating point constants needed for FPU paramerterization
`define FPSIZES ((32)'(`Q_SUPPORTED)+(32)'(`D_SUPPORTED)+(32)'(`F_SUPPORTED)+(32)'(`ZFH_SUPPORTED))
`define FMTBITS ((32)'(`FPSIZES>=3)+1)
`define LEN1  ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_LEN  : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_LEN  : `H_LEN)
`define NE1   ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_NE   : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_NE   : `H_NE)
`define NF1   ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_NF   : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_NF   : `H_NF)
`define FMT1  ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? 2'd1    : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? 2'd0    : 2'd2)
`define BIAS1 ((`D_SUPPORTED & (`FLEN != `D_LEN)) ? `D_BIAS : (`F_SUPPORTED & (`FLEN != `S_LEN)) ? `S_BIAS : `H_BIAS)
`define LEN2  ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_LEN  : `H_LEN)
`define NE2   ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_NE   : `H_NE)
`define NF2   ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_NF   : `H_NF)
`define FMT2  ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? 2'd0    : 2'd2)
`define BIAS2 ((`F_SUPPORTED & (`LEN1 != `S_LEN)) ? `S_BIAS : `H_BIAS)

// largest length in IEU/FPU
`define CVTLEN ((`NF<`XLEN) ? (`XLEN) : (`NF))
`define LLEN (($unsigned(`FLEN)<$unsigned(`XLEN)) ? ($unsigned(`XLEN)) : ($unsigned(`FLEN)))
`define LOGCVTLEN $unsigned($clog2(`CVTLEN+1))
`define NORMSHIFTSZ (((`CVTLEN+`NF+1)>(`DIVb + 1 +`NF+1) & (`CVTLEN+`NF+1)>(3*`NF+6)) ? (`CVTLEN+`NF+1) : ((`DIVb + 1 +`NF+1) > (3*`NF+6) ? (`DIVb + 1 +`NF+1) : (3*`NF+6)))
`define LOGNORMSHIFTSZ ($clog2(`NORMSHIFTSZ))
`define CORRSHIFTSZ (((`CVTLEN+`NF+1)>(`DIVb + 1 +`NF+1) & (`CVTLEN+`NF+1)>(3*`NF+6)) ? (`CVTLEN+`NF+1) : ((`DIVN+1+`NF) > (3*`NF+4) ? (`DIVN+1+`NF) : (3*`NF+4)))

// division constants

`define DIVN        (((`NF<`XLEN) & `IDIV_ON_FPU) ? `XLEN : `NF+2) // standard length of input
`define LOGR        ($clog2(`RADIX))            // r = log(R)
`define RK          (`LOGR*`DIVCOPIES)          // r*k used for intdiv preproc
`define LOGRK       ($clog2(`RK))               // log2(r*k)
`define FPDUR       ((`DIVN+1+(`LOGR*`DIVCOPIES))/(`LOGR*`DIVCOPIES)+(`RADIX/4))
`define DURLEN      ($clog2(`FPDUR+1))
`define DIVb        (`FPDUR*`LOGR*`DIVCOPIES-1) // canonical fdiv size (b)
`define DIVBLEN     ($clog2(`DIVb+1)-1)
`define DIVa        (`DIVb+1-`XLEN)             // used for idiv on fpu

// Disable spurious Verilator warnings

/* verilator lint_off STMTDLY */
/* verilator lint_off ASSIGNDLY */
/* verilator lint_off PINCONNECTEMPTY */


///////////////////////////////////////////
// alu.sv
//
// Written: David_Harris@hmc.edu, Sarah.Harris@unlv.edu, kekim@hmc.edu
// Created: 9 January 2021
// Modified: 3 March 2023
//
// Purpose: RISC-V Arithmetic/Logic Unit
//
// Documentation: RISC-V System on Chip Design Chapter 4 (Figure 4.4)
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


`define FPGA 0
`define QEMU 0

// RV32 or RV64: XLEN = 32 or 64
`define XLEN 32

// IEEE 754 compliance
`define IEEE754 0

// E
`define MISA (32'h00000010) 
`define ZICSR_SUPPORTED 0
`define ZIFENCEI_SUPPORTED 0
`define COUNTERS 0
`define ZICOUNTERS_SUPPORTED 0
`define ZFH_SUPPORTED 0
`define SSTC_SUPPORTED 0

// LSU microarchitectural Features
`define BUS_SUPPORTED 1
`define DCACHE_SUPPORTED 0
`define ICACHE_SUPPORTED 0
`define VIRTMEM_SUPPORTED 0
`define VECTORED_INTERRUPTS_SUPPORTED 0 
`define BIGENDIAN_SUPPORTED 0

// TLB configuration.  Entries should be a power of 2
`define ITLB_ENTRIES 0
`define DTLB_ENTRIES 0

// Cache configuration.  Sizes should be a power of two
// typical configuration 4 ways, 4096 bytes per way, 256 bit or more lines
`define DCACHE_NUMWAYS 4
`define DCACHE_WAYSIZEINBYTES 4096
`define DCACHE_LINELENINBITS 512
`define ICACHE_NUMWAYS 4
`define ICACHE_WAYSIZEINBYTES 4096
`define ICACHE_LINELENINBITS 512

// Integer Divider Configuration
// IDIV_BITSPERCYCLE must be 1, 2, or 4
`define IDIV_BITSPERCYCLE 1
`define IDIV_ON_FPU 0

// Legal number of PMP entries are 0, 16, or 64
`define PMP_ENTRIES 0

// Address space
`define RESET_VECTOR 32'h80000000

// WFI Timeout Wait
`define WFI_TIMEOUT_BIT 16

// Peripheral Addresses
// Peripheral memory space extends from BASE to BASE+RANGE
// Range should be a thermometer code with 0's in the upper bits and 1s in the lower bits
`define DTIM_SUPPORTED 1'b0
`define DTIM_BASE       34'h80000000
`define DTIM_RANGE      34'h007FFFFF
`define IROM_SUPPORTED 1'b0
`define IROM_BASE       34'h80000000
`define IROM_RANGE      34'h007FFFFF
`define BOOTROM_SUPPORTED 1'b1
`define BOOTROM_BASE   34'h00001000 
`define BOOTROM_RANGE  34'h00000FFF
`define UNCORE_RAM_SUPPORTED 1'b1
`define UNCORE_RAM_BASE       34'h80000000
`define UNCORE_RAM_RANGE      34'h07FFFFFF
`define EXT_MEM_SUPPORTED 1'b0
`define EXT_MEM_BASE       34'h80000000
`define EXT_MEM_RANGE      34'h07FFFFFF
`define CLINT_SUPPORTED 1'b0
`define CLINT_BASE  34'h02000000
`define CLINT_RANGE 34'h0000FFFF
`define GPIO_SUPPORTED 1'b0
`define GPIO_BASE   34'h10060000
`define GPIO_RANGE  34'h000000FF
`define UART_SUPPORTED 1'b0
`define UART_BASE   34'h10000000
`define UART_RANGE  34'h00000007
`define PLIC_SUPPORTED 1'b0
`define PLIC_BASE   34'h0C000000
`define PLIC_RANGE  34'h03FFFFFF
`define SDC_SUPPORTED 1'b0
`define SDC_BASE   34'h00012100
`define SDC_RANGE  34'h0000001F

// Bus Interface width
`define AHBW 32

// Test modes

// Tie GPIO outputs back to inputs
`define GPIO_LOOPBACK_TEST 1

// Hardware configuration
`define UART_PRESCALE 1

// Interrupt configuration
`define PLIC_NUM_SRC 10 
// comment out the following if >=32 sources
`define PLIC_NUM_SRC_LT_32
`define PLIC_GPIO_ID 3
`define PLIC_UART_ID 10

`define BPRED_SUPPORTED 0
`define BPRED_TYPE "BP_GSHARE" // BP_GSHARE_BASIC, BP_GLOBAL, BP_GLOBAL_BASIC, BP_TWOBIT
`define BPRED_SIZE 10
`define BTB_SIZE 10

`define SVADU_SUPPORTED 0
`define ZMMUL_SUPPORTED 0

// FPU division architecture
`define RADIX 32'h4
`define DIVCOPIES 32'h4

// bit manipulation
`define ZBA_SUPPORTED 0
`define ZBB_SUPPORTED 0
`define ZBC_SUPPORTED 0
`define ZBS_SUPPORTED 0

// Memory synthesis configuration
`define USE_SRAM 0




module testbench();

   logic        clk;
   logic        reset;

   logic [31:0] WriteData, DataAdr;
   logic        MemWrite;

   // instantiate device to be tested
   top dut(clk, reset, WriteData, DataAdr, MemWrite);

   initial
     begin
	string memfilename;
        memfilename = {"../riscvtest/riscvtest.memfile"};
	$readmemh(memfilename, dut.imem.RAM);
     end
   
   // initialize test
   initial
     begin
	reset <= 1; # 22; reset <= 0;
     end

   // generate clock to sequence tests
   always
     begin
	clk <= 1; # 5; clk <= 0; # 5;
     end

   // check results
   always @(negedge clk)
     begin
	if(MemWrite) begin
           if(DataAdr === 100 & WriteData === 25) begin
              $display("Simulation succeeded");
              $stop;
           end else if (DataAdr !== 96) begin
              $display("Simulation failed");
              $stop;
           end
	end
     end
endmodule

module top(input  logic        clk, reset, 
           output logic [31:0] WriteDataM, DataAdrM, 
           output logic        MemWriteM);

   logic [31:0] 	       PCF, InstrF, ReadDataM;
   
   // instantiate processor and memories
   riscv rv32pipe (clk, reset, PCF, InstrF, MemWriteM, DataAdrM, 
		   WriteDataM, ReadDataM);
   imem imem (PCF, InstrF);
   dmem dmem (clk, MemWriteM, DataAdrM, WriteDataM, ReadDataM);
   
endmodule

module riscv(input  logic        clk, reset,
             output logic [31:0] PCF,
             input logic [31:0]  InstrF,
             output logic 	 MemWriteM,
             output logic [31:0] ALUResultM, WriteDataM,
             input logic [31:0]  ReadDataM);

   logic [6:0] 			 opD;
   logic [2:0] 			 funct3D;
   logic 			 funct7b5D;
   logic [1:0] 			 ImmSrcD;
   logic 			 ZeroE;
   logic 			 PCSrcE;
   logic [2:0] 			 ALUControlE;
   logic 			 ALUSrcE;
   logic 			 ResultSrcEb0;
   logic 			 RegWriteM;
   logic [1:0] 			 ResultSrcW;
   logic 			 RegWriteW;

   logic [1:0] 			 ForwardAE, ForwardBE;
   logic 			 StallF, StallD, FlushD, FlushE,BPDirPredD,BPDirPredWrongE;
   logic [31:0] PCNextF;
   logic [4:0] 			 Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW;
   logic           BPBTAF,BPBTAD,BPBTAE,BTBIClassF,IClassWrongM,IClassWrongE,IEUAdrE,IEUAdrM,InstrClassD,InstrClassE,InstrClassM,InstrClassW;
   controller c(clk, reset,
		opD, funct3D, funct7b5D, ImmSrcD,
		FlushE, ZeroE, PCSrcE, ALUControlE, ALUSrcE, ResultSrcEb0,
		MemWriteM, RegWriteM, 
		RegWriteW, ResultSrcW);

   datapath dp(clk, reset,
               StallF, PCF, InstrF,
	       opD, funct3D, funct7b5D, StallD, FlushD, ImmSrcD,
	       FlushE, ForwardAE, ForwardBE, PCSrcE, ALUControlE, ALUSrcE, ZeroE,
               MemWriteM, WriteDataM, ALUResultM, ReadDataM,
               RegWriteW, ResultSrcW,
               Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,PCNextF);

   hazard  hu(Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              PCSrcE, ResultSrcEb0, RegWriteM, RegWriteW,
              ForwardAE, ForwardBE, StallF, StallD, FlushD, FlushE);
              
   gshare gs(clk, reset, StallF, StallD,StallE,StallM,StallW,FlushD,FlushE,FlushM,FlushW,
              BPDirPredD,BPDirPredWrongE, PCNextF, PCF, PCD, PCE, PCM, BPBranchF,BranchD,BranchE,BranchM,BranchW,PCSrcE);
   btb b(clk, reset, StallF, StallD,StallE,StallM,StallW,FlushD,FlushE,FlushM,FlushW, PCNextF, PCF, PCD, PCE, PCM,
          BPBTAF,BPBTAD,BPBTAE,BTBIClassF,IClassWrongM,IClassWrongE,IEUAdrE,IEUAdrM,InstrClassD,InstrClassE,InstrClassM,InstrClassW);
endmodule


module controller(input  logic		 clk, reset,
                  // Decode stage control signals
                  input logic [6:0]  opD,
                  input logic [2:0]  funct3D,
                  input logic 	     funct7b5D,
                  output logic [1:0] ImmSrcD,
                  // Execute stage control signals
                  input logic 	     FlushE, 
                  input logic 	     ZeroE, 
                  output logic 	     PCSrcE, // for datapath and Hazard Unit
                  output logic [2:0] ALUControlE, 
                  output logic 	     ALUSrcE,
                  output logic 	     ResultSrcEb0, // for Hazard Unit
                  // Memory stage control signals
                  output logic 	     MemWriteM,
                  output logic 	     RegWriteM, // for Hazard Unit				  
                  // Writeback stage control signals
                  output logic 	     RegWriteW, // for datapath and Hazard Unit
                  output logic [1:0] ResultSrcW);

   // pipelined control signals
   logic 			     RegWriteD, RegWriteE;
   logic [1:0] 			     ResultSrcD, ResultSrcE, ResultSrcM;
   logic 			     MemWriteD, MemWriteE;
   logic 			     JumpD, JumpE;
   logic 			     BranchD, BranchE;
   logic [1:0] 			     ALUOpD;
   logic [2:0] 			     ALUControlD;
   logic 			     ALUSrcD;
   
   // Decode stage logic
   maindec md(opD, ResultSrcD, MemWriteD, BranchD,
              ALUSrcD, RegWriteD, JumpD, ImmSrcD, ALUOpD);
   aludec  ad(opD[5], funct3D, funct7b5D, ALUOpD, ALUControlD);
   
   // Execute stage pipeline control register and logic
   floprc #(10) controlregE(clk, reset, FlushE,
                            {RegWriteD, ResultSrcD, MemWriteD, JumpD, BranchD, ALUControlD, ALUSrcD},
                            {RegWriteE, ResultSrcE, MemWriteE, JumpE, BranchE, ALUControlE, ALUSrcE});

   assign PCSrcE = (BranchE & ZeroE) | JumpE;
   assign ResultSrcEb0 = ResultSrcE[0];
   
   // Memory stage pipeline control register
   flopr #(4) controlregM(clk, reset,
                          {RegWriteE, ResultSrcE, MemWriteE},
                          {RegWriteM, ResultSrcM, MemWriteM});
   
   // Writeback stage pipeline control register
   flopr #(3) controlregW(clk, reset,
                          {RegWriteM, ResultSrcM},
                          {RegWriteW, ResultSrcW});     
endmodule

module maindec(input  logic [6:0] op,
               output logic [1:0] ResultSrc,
               output logic 	  MemWrite,
               output logic 	  Branch, ALUSrc,
               output logic 	  RegWrite, Jump,
               output logic [1:0] ImmSrc,
               output logic [1:0] ALUOp);

   logic [10:0] 		  controls;

   assign {RegWrite, ImmSrc, ALUSrc, MemWrite,
           ResultSrc, Branch, ALUOp, Jump} = controls;

   always_comb
     case(op)
       // RegWrite_ImmSrc_ALUSrc_MemWrite_ResultSrc_Branch_ALUOp_Jump
       7'b0000011: controls = 11'b1_00_1_0_01_0_00_0; // lw
       7'b0100011: controls = 11'b0_01_1_1_00_0_00_0; // sw
       7'b0110011: controls = 11'b1_xx_0_0_00_0_10_0; // R-type 
       7'b1100011: controls = 11'b0_10_0_0_00_1_01_0; // beq
       7'b0010011: controls = 11'b1_00_1_0_00_0_10_0; // I-type ALU
       7'b1101111: controls = 11'b1_11_0_0_10_0_00_1; // jal
       7'b0000000: controls = 11'b0_00_0_0_00_0_00_0; // need valid values at reset
       default:    controls = 11'bx_xx_x_x_xx_x_xx_x; // non-implemented instruction
     endcase
endmodule

module aludec(input  logic       opb5,
              input logic [2:0]  funct3,
              input logic 	 funct7b5, 
              input logic [1:0]  ALUOp,
              output logic [2:0] ALUControl);

   logic 			 RtypeSub;
   assign RtypeSub = funct7b5 & opb5;  // TRUE for R-type subtract instruction

   always_comb
     case(ALUOp)
       2'b00:                ALUControl = 3'b000; // addition
       2'b01:                ALUControl = 3'b001; // subtraction
       default: case(funct3) // R-type or I-type ALU
                  3'b000:  if (RtypeSub) 
                    ALUControl = 3'b001; // sub
                  else          
                    ALUControl = 3'b000; // add, addi
                  3'b010:    ALUControl = 3'b101; // slt, slti
                  3'b110:    ALUControl = 3'b011; // or, ori
                  3'b111:    ALUControl = 3'b010; // and, andi
                  default:   ALUControl = 3'bxxx; // ???
		endcase
     endcase
endmodule

module datapath(input logic clk, reset,
                // Fetch stage signals
                input logic 	    StallF,
                output logic [31:0] PCF,
                input logic [31:0]  InstrF,
                // Decode stage signals
                output logic [6:0]  opD,
                output logic [2:0]  funct3D, 
                output logic 	    funct7b5D,
                input logic 	    StallD, FlushD,
                input logic [1:0]   ImmSrcD,
                // Execute stage signals
                input logic 	    FlushE,
                input logic [1:0]   ForwardAE, ForwardBE,
                input logic 	    PCSrcE,
                input logic [2:0]   ALUControlE,
                input logic 	    ALUSrcE,
                output logic 	    ZeroE,
                // Memory stage signals
                input logic 	    MemWriteM, 
                output logic [31:0] WriteDataM, ALUResultM,
                input logic [31:0]  ReadDataM,
                // Writeback stage signals
                input logic 	    RegWriteW, 
                input logic [1:0]   ResultSrcW,
                // Hazard Unit signals 
                output logic [4:0]  Rs1D, Rs2D, Rs1E, Rs2E,
                output logic [4:0]  RdE, RdM, RdW,
                output logic [31:0] PCNextF);

   // Fetch stage signals
   //logic [31:0] 		    PCNextF, PCPlus4F;
  logic [31:0] 		     PCPlus4F;

   // Decode stage signals
   logic [31:0] 		    InstrD;
   logic [31:0] 		    PCD, PCPlus4D;
   logic [31:0] 		    RD1D, RD2D;
   logic [31:0] 		    ImmExtD;
   logic [4:0] 			    RdD;
   // Execute stage signals
   logic [31:0] 		    RD1E, RD2E;
   logic [31:0] 		    PCE, ImmExtE;
   logic [31:0] 		    SrcAE, SrcBE;
   logic [31:0] 		    ALUResultE;
   logic [31:0] 		    WriteDataE;
   logic [31:0] 		    PCPlus4E;
   logic [31:0] 		    PCTargetE;
   // Memory stage signals
   logic [31:0] 		    PCPlus4M;
   // Writeback stage signals
   logic [31:0] 		    ALUResultW;
   logic [31:0] 		    ReadDataW;
   logic [31:0] 		    PCPlus4W;
   logic [31:0] 		    ResultW;

   // Fetch stage pipeline register and logic
   mux2    #(32) pcmux(PCPlus4F, PCTargetE, PCSrcE, PCNextF);
   flopenr #(32) pcreg(clk, reset, ~StallF, PCNextF, PCF);
   adder         pcadd(PCF, 32'h4, PCPlus4F);

   // Decode stage pipeline register and logic
   flopenrc #(96) regD(clk, reset, FlushD, ~StallD, 
                       {InstrF, PCF, PCPlus4F},
                       {InstrD, PCD, PCPlus4D});
   assign opD       = InstrD[6:0];
   assign funct3D   = InstrD[14:12];
   assign funct7b5D = InstrD[30];
   assign Rs1D      = InstrD[19:15];
   assign Rs2D      = InstrD[24:20];
   assign RdD       = InstrD[11:7];
   
   regfile        rf(clk, RegWriteW, Rs1D, Rs2D, RdW, ResultW, RD1D, RD2D);
   extend         ext(InstrD[31:7], ImmSrcD, ImmExtD);
   
   // Execute stage pipeline register and logic
   floprc #(175) regE(clk, reset, FlushE, 
                      {RD1D, RD2D, PCD, Rs1D, Rs2D, RdD, ImmExtD, PCPlus4D}, 
                      {RD1E, RD2E, PCE, Rs1E, Rs2E, RdE, ImmExtE, PCPlus4E});
   
   mux3   #(32)  faemux(RD1E, ResultW, ALUResultM, ForwardAE, SrcAE);
   mux3   #(32)  fbemux(RD2E, ResultW, ALUResultM, ForwardBE, WriteDataE);
   mux2   #(32)  srcbmux(WriteDataE, ImmExtE, ALUSrcE, SrcBE);
   alu           alu(SrcAE, SrcBE, ALUControlE, ALUResultE, ZeroE);
   adder         branchadd(ImmExtE, PCE, PCTargetE);

   // Memory stage pipeline register
   flopr  #(101) regM(clk, reset, 
                      {ALUResultE, WriteDataE, RdE, PCPlus4E},
                      {ALUResultM, WriteDataM, RdM, PCPlus4M});
   
   // Writeback stage pipeline register and logic
   flopr  #(101) regW(clk, reset, 
                      {ALUResultM, ReadDataM, RdM, PCPlus4M},
                      {ALUResultW, ReadDataW, RdW, PCPlus4W});
   mux3   #(32)  resultmux(ALUResultW, ReadDataW, PCPlus4W, ResultSrcW, ResultW);	
endmodule

// Hazard Unit: forward, stall, and flush
module hazard(input  logic [4:0] Rs1D, Rs2D, Rs1E, Rs2E, RdE, RdM, RdW,
              input logic 	 PCSrcE, ResultSrcEb0, 
              input logic 	 RegWriteM, RegWriteW,
              output logic [1:0] ForwardAE, ForwardBE,
              output logic 	 StallF, StallD, FlushD, FlushE);

   logic 			 lwStallD;
   
   // forwarding logic
   always_comb begin
      ForwardAE = 2'b00;
      ForwardBE = 2'b00;
      if (Rs1E != 5'b0)
	if      ((Rs1E == RdM) & RegWriteM) ForwardAE = 2'b10;
	else if ((Rs1E == RdW) & RegWriteW) ForwardAE = 2'b01;
      
      if (Rs2E != 5'b0)
	if      ((Rs2E == RdM) & RegWriteM) ForwardBE = 2'b10;
	else if ((Rs2E == RdW) & RegWriteW) ForwardBE = 2'b01;
   end
   
   // stalls and flushes
   assign lwStallD = ResultSrcEb0 & ((Rs1D == RdE) | (Rs2D == RdE));  
   assign StallD = lwStallD;
   assign StallF = lwStallD;
   assign FlushD = PCSrcE;
   assign FlushE = lwStallD | PCSrcE;
endmodule

module regfile(input  logic        clk, 
               input logic 	   we3, 
               input logic [ 4:0]  a1, a2, a3, 
               input logic [31:0]  wd3, 
               output logic [31:0] rd1, rd2);

   logic [31:0] 		   rf[31:0];

   // three ported register file
   // read two ports combinationally (A1/RD1, A2/RD2)
   // write third port on rising edge of clock (A3/WD3/WE3)
   // write occurs on falling edge of clock
   // register 0 hardwired to 0

   always_ff @(negedge clk)
     if (we3) rf[a3] <= wd3;	

   assign rd1 = (a1 != 0) ? rf[a1] : 0;
   assign rd2 = (a2 != 0) ? rf[a2] : 0;
endmodule

module adder(input  [31:0] a, b,
             output [31:0] y);

   assign y = a + b;
endmodule

module extend(input  logic [31:7] instr,
              input logic [1:0]   immsrc,
              output logic [31:0] immext);
   
   always_comb
     case(immsrc) 
       // I-type 
       2'b00:   immext = {{20{instr[31]}}, instr[31:20]};  
       // S-type (stores)
       2'b01:   immext = {{20{instr[31]}}, instr[31:25], instr[11:7]}; 
       // B-type (branches)
       2'b10:   immext = {{20{instr[31]}}, instr[7], instr[30:25], instr[11:8], 1'b0}; 
       // J-type (jal)
       2'b11:   immext = {{12{instr[31]}}, instr[19:12], instr[20], instr[30:21], 1'b0}; 
       default: immext = 32'bx; // undefined
     endcase             
endmodule

module flopr #(parameter WIDTH = 8)
   (input  logic             clk, reset,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       q <= d;
endmodule

module flopenr #(parameter WIDTH = 8)
   (input  logic             clk, reset, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) q <= d;
endmodule

module flopenrc #(parameter WIDTH = 8)
   (input  logic             clk, reset, clear, en,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset)   q <= 0;
     else if (en) 
       if (clear) q <= 0;
       else       q <= d;
endmodule

module floprc #(parameter WIDTH = 8)
   (input  logic clk,
    input logic 	     reset,
    input logic 	     clear,
    input logic [WIDTH-1:0]  d, 
    output logic [WIDTH-1:0] q);

   always_ff @(posedge clk, posedge reset)
     if (reset) q <= 0;
     else       
       if (clear) q <= 0;
       else       q <= d;
endmodule

module mux2 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, 
    input logic 	     s, 
    output logic [WIDTH-1:0] y);

   assign y = s ? d1 : d0; 
endmodule

module mux3 #(parameter WIDTH = 8)
   (input  logic [WIDTH-1:0] d0, d1, d2,
    input logic [1:0] 	     s, 
    output logic [WIDTH-1:0] y);

   assign y = s[1] ? d2 : (s[0] ? d1 : d0); 
endmodule

module imem (input  logic [31:0] a,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[63:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   
endmodule // imem

module dmem (input  logic        clk, we,
	     input  logic [31:0] a, wd,
	     output logic [31:0] rd);
   
   logic [31:0] 		 RAM[255:0];
   
   assign rd = RAM[a[31:2]]; // word aligned
   always_ff @(posedge clk)
     if (we) RAM[a[31:2]] <= wd;
   
endmodule // dmem

module alu(input  logic [31:0] a, b,
           input logic [2:0]   alucontrol,
           output logic [31:0] result,
           output logic        zero);

   logic [31:0] 	       condinvb, sum;
   logic 		       v;              // overflow
   logic 		       isAddSub;       // true when is add or sub

   assign condinvb = alucontrol[0] ? ~b : b;
   assign sum = a + condinvb + alucontrol[0];
   assign isAddSub = ~alucontrol[2] & ~alucontrol[1] |
                     ~alucontrol[1] &  alucontrol[0];

   always_comb
     case (alucontrol)
       3'b000:  result = sum;         // add
       3'b001:  result = sum;         // subtract
       3'b010:  result = a & b;       // and
       3'b011:  result = a | b;       // or
       3'b100:  result = a ^ b;       // xor
       3'b101:  result = sum[31] ^ v; // slt
       3'b110:  result = a << b[4:0]; // sll
       3'b111:  result = a >> b[4:0]; // srl
       default: result = 32'bx;
     endcase

   assign zero = (result == 32'b0);
   assign v = ~(alucontrol[0] ^ a[31] ^ b[31]) & (a[31] ^ sum[31]) & isAddSub;
   
endmodule

///////////////////////////////////////////
// gshare.sv
//
// Written: Ross Thompson
// Email: ross1728@gmail.com
// Created: 16 March 2021
// Adapted from ssanghai@hmc.edu (Shreya Sanghai)
// Modified: 20 February 2023
//
// Purpose: gshare and Global History Branch predictors
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module gshare #(parameter k = 10,
                parameter integer TYPE = 1) (
  input logic             clk,
  input logic             reset,
  input logic             StallF, StallD, StallE, StallM, StallW,
  input logic             FlushD, FlushE, FlushM, FlushW,
  output logic [1:0]      BPDirPredF, 
  output logic            BPDirPredWrongE,
  // update
  input logic [`XLEN-1:0] PCNextF, PCF, PCD, PCE, PCM,
  input logic             BPBranchF, BranchD, BranchE, BranchM, BranchW, PCSrcE
);

  logic                   MatchF, MatchD, MatchE, MatchM, MatchW;
  logic                   MatchX;

  logic [1:0]             TableBPDirPredF, BPDirPredD, BPDirPredE, FwdNewDirPredF;
  logic [1:0]             NewBPDirPredE, NewBPDirPredM, NewBPDirPredW;

  logic [k-1:0]           IndexNextF, IndexF, IndexD, IndexE, IndexM, IndexW;

  logic [k-1:0]           GHRF, GHRD, GHRE, GHRM;
  logic [k-1:0]           GHRNextM, GHRNextF;
  logic                   PCSrcM;

  if(TYPE == 1) begin
  assign IndexNextF = GHRNextF ^ {PCNextF[k+1] ^ PCNextF[1], PCNextF[k:2]};
  assign IndexF = GHRF ^ {PCF[k+1] ^ PCF[1], PCF[k:2]};
  assign IndexD = GHRD ^ {PCD[k+1] ^ PCD[1], PCD[k:2]};
  assign IndexE = GHRE ^ {PCE[k+1] ^ PCE[1], PCE[k:2]};
  assign IndexM = GHRM ^ {PCM[k+1] ^ PCM[1], PCM[k:2]};
  end else if(TYPE == 0) begin
  assign IndexNextF = GHRNextF;
  assign IndexF = GHRF;
  assign IndexD = GHRD;
  assign IndexE = GHRE;
  assign IndexM = GHRM;
  end

  flopenrc #(k) IndexWReg(clk, reset, FlushW, ~StallW, IndexM, IndexW);

  assign MatchD = BranchD & ~FlushE & (IndexF == IndexD);
  assign MatchE = BranchE & ~FlushM & (IndexF == IndexE);
  assign MatchM = BranchM & ~FlushW & (IndexF == IndexM);
  assign MatchW = BranchW & ~FlushW & (IndexF == IndexW);
  assign MatchX = MatchD | MatchE | MatchM | MatchW;

  assign FwdNewDirPredF = MatchD ? {2{BPDirPredD[1]}} :
                                   MatchE ? {NewBPDirPredE} :
                                   MatchM ? {NewBPDirPredM} :
                   NewBPDirPredW ;
  
  assign BPDirPredF = MatchX ? FwdNewDirPredF : TableBPDirPredF;

  ram2p1r1wbe #(2**k, 2) PHT(.clk(clk),
    .ce1(~StallF), .ce2(~StallW & ~FlushW),
    .ra1(IndexNextF),
    .rd1(TableBPDirPredF),
    .wa2(IndexM),
    .wd2(NewBPDirPredM),
    .we2(BranchM),
    .bwe2(1'b1));

  flopenrc #(2) PredictionRegD(clk, reset,  FlushD, ~StallD, BPDirPredF, BPDirPredD);
  flopenrc #(2) PredictionRegE(clk, reset,  FlushE, ~StallE, BPDirPredD, BPDirPredE);

  satCounter2 BPDirUpdateE(.BrDir(PCSrcE), .OldState(BPDirPredE), .NewState(NewBPDirPredE));
  flopenrc #(2) NewPredictionRegM(clk, reset,  FlushM, ~StallM, NewBPDirPredE, NewBPDirPredM);
  flopenrc #(2) NewPredictionRegW(clk, reset,  FlushW, ~StallW, NewBPDirPredM, NewBPDirPredW);

  assign BPDirPredWrongE = PCSrcE != BPDirPredE[1] & BranchE;

  assign GHRNextF = BPBranchF ? {BPDirPredF[1], GHRF[k-1:1]} : GHRF;
  assign GHRF = BranchD  ? {BPDirPredD[1], GHRD[k-1:1]} : GHRD;
  assign GHRD = BranchE ? {PCSrcE, GHRE[k-1:1]} : GHRE;
  assign GHRE = BranchM ? {PCSrcM, GHRM[k-1:1]} : GHRM;

  assign GHRNextM = {PCSrcM, GHRM[k-1:1]};

  flopenr #(k) GHRReg(clk, reset, ~StallW & ~FlushW & BranchM, GHRNextM, GHRM);
  flopenrc #(1) PCSrcMReg(clk, reset, FlushM, ~StallM, PCSrcE, PCSrcM);
    
endmodule

///////////////////////////////////////////
// btb.sv
//
// Written: Ross Thompson ross1728@gmail.com
// Created: February 15, 2021
// Modified: 24 January 2023 
//
// Purpose: Branch Target Buffer (BTB). The BTB predicts the target address of all control flow instructions.
//          It also guesses the type of instrution; jalr(r), return, jump (jr), or branch.
//
// Documentation: RISC-V System on Chip Design Chapter 10 (Figure ***)
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////


module btb #(parameter Depth = 10 ) (
  input  logic             clk,
  input  logic             reset,
  input  logic             StallF, StallD, StallE, StallM, StallW, FlushD, FlushE, FlushM, FlushW,
  input  logic [`XLEN-1:0] PCNextF, PCF, PCD, PCE, PCM, // PC at various stages
  output logic [`XLEN-1:0] BPBTAF,                      // BTB's guess at PC
  output logic [`XLEN-1:0] BPBTAD,
  output logic [`XLEN-1:0] BPBTAE,
  output logic [3:0]       BTBIClassF,                  // BTB's guess at instruction class
  // update
  input  logic             IClassWrongM,                // BTB's instruction class guess was wrong
  input  logic             IClassWrongE,
  input  logic [`XLEN-1:0] IEUAdrE,                     // Branch/jump target address to insert into btb
  input  logic [`XLEN-1:0] IEUAdrM,                     // Branch/jump target address to insert into btb
  input  logic [3:0]       InstrClassD,                 // Instruction class to insert into btb
  input  logic [3:0]       InstrClassE,                 // Instruction class to insert into btb
  input  logic [3:0]       InstrClassM,                 // Instruction class to insert into btb
  input  logic [3:0]       InstrClassW
);

  logic [Depth-1:0]        PCNextFIndex, PCFIndex, PCDIndex, PCEIndex, PCMIndex, PCWIndex;
  logic [`XLEN-1:0]        ResetPC;
  logic                    MatchD, MatchE, MatchM, MatchW, MatchX;
  logic [`XLEN+3:0]        ForwardBTBPrediction, ForwardBTBPredictionF;
  logic [`XLEN+3:0]        TableBTBPredF;
  logic [`XLEN-1:0]        IEUAdrW;
  logic [`XLEN-1:0]        PCW;
  logic                    BTBWrongE, BPBTAWrongE;
  logic                    BTBWrongM, BPBTAWrongM;
  
  
  // hashing function for indexing the PC
  // We have Depth bits to index, but XLEN bits as the input.
  // bit 0 is always 0, bit 1 is 0 if using 4 byte instructions, but is not always 0 if
  // using compressed instructions.  XOR bit 1 with the MSB of index.
  assign PCFIndex = {PCF[Depth+1] ^ PCF[1], PCF[Depth:2]};
  assign PCDIndex = {PCD[Depth+1] ^ PCD[1], PCD[Depth:2]};
  assign PCEIndex = {PCE[Depth+1] ^ PCE[1], PCE[Depth:2]};
  assign PCMIndex = {PCM[Depth+1] ^ PCM[1], PCM[Depth:2]};
  assign PCWIndex = {PCW[Depth+1] ^ PCW[1], PCW[Depth:2]};

  // must output a valid PC and valid bit during reset.  Because only PCF, not PCNextF is reset, PCNextF is invalid
  // during reset.  The BTB must produce a non X PC1NextF to allow the simulation to run.
  // While the mux could be included in IFU it is not necessary for the IROM/I$/bus.
  // For now it is optimal to leave it here.
  assign ResetPC = `RESET_VECTOR;
  assign PCNextFIndex = reset ? ResetPC[Depth+1:2] : {PCNextF[Depth+1] ^ PCNextF[1], PCNextF[Depth:2]}; 

  assign MatchD = PCFIndex == PCDIndex;
  assign MatchE = PCFIndex == PCEIndex;
  assign MatchM = PCFIndex == PCMIndex;
  assign MatchW = PCFIndex == PCWIndex;
  assign MatchX = MatchD | MatchE | MatchM | MatchW;

  assign ForwardBTBPredictionF = MatchD ? {InstrClassD, BPBTAD} :
                                 MatchE ? {InstrClassE, IEUAdrE} :
                                 MatchM ? {InstrClassM, IEUAdrM} :
                                 {InstrClassW, IEUAdrW} ;

  assign {BTBIClassF, BPBTAF} = MatchX ? ForwardBTBPredictionF : {TableBTBPredF};


  // An optimization may be using a PC relative address.
  ram2p1r1wbe #(2**Depth, `XLEN+4) memory(
    .clk, .ce1(~StallF | reset), .ra1(PCNextFIndex), .rd1(TableBTBPredF),
     .ce2(~StallW & ~FlushW), .wa2(PCMIndex), .wd2({InstrClassM, IEUAdrM}), .we2(BTBWrongM), .bwe2('1));

  flopenrc #(`XLEN) BTBD(clk, reset, FlushD, ~StallD, BPBTAF, BPBTAD);

  // BPBTAE is not strickly necessary.  However it is used by two parts of wally.
  // 1. It gates updates to the BTB when the prediction does not change.  This save power.
  // 2. BPBTAWrongE is used by the performance counters to track when the BTB's BPBTA or instruction class is wrong.
  flopenrc #(`XLEN) BTBTargetEReg(clk, reset, FlushE, ~StallE, BPBTAD, BPBTAE);
  assign BPBTAWrongE = (BPBTAE != IEUAdrE) & (InstrClassE[0] | InstrClassE[1] & ~InstrClassE[2]);

  flopenrc #(1) BPBTAWrongMReg(clk, reset, FlushM, ~StallM, BPBTAWrongE, BPBTAWrongM);  
  assign BTBWrongM = BPBTAWrongM | IClassWrongM;
  
  flopenr #(`XLEN) PCWReg(clk, reset, ~StallW, PCM, PCW);
  flopenr #(`XLEN) IEUAdrWReg(clk, reset, ~StallW, IEUAdrM, IEUAdrW);

endmodule




module satCounter2
  (input logic        BrDir,
   input logic [1:0]  OldState,
   output logic [1:0] NewState
   );

  always_comb begin
    case(OldState)
      2'b00: begin
 if(BrDir) NewState = 2'b01;
 else NewState = 2'b00;
      end
      2'b01: begin
 if(BrDir) NewState = 2'b10;
 else NewState = 2'b00;
      end
      2'b10: begin
 if(BrDir) NewState = 2'b11;
 else NewState = 2'b01;
      end
      2'b11: begin
 if(BrDir) NewState = 2'b11;
 else NewState = 2'b10;
      end
    endcase
  end

endmodule



///////////////////////////////////////////
// 2 port sram.
//
// Written: ross1728@gmail.com May 3, 2021
//          Two port SRAM 1 read port and 1 write port.
//          When clk rises Addr and LineWriteData are sampled.
//          Following the clk edge read data is output from the sampled Addr.
//          Write
// Modified: james.stine@okstate.edu Feb 1, 2023
//           Integration of memories 
//
// Purpose: Storage and read/write access to data cache data, tag valid, dirty, and replacement.
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

// WIDTH is number of bits in one "word" of the memory, DEPTH is number of such words


module ram2p1r1wbe #(parameter DEPTH=1024, WIDTH=68) (
  input  logic                     clk,
  input  logic                     ce1, ce2,
  input  logic [$clog2(DEPTH)-1:0] ra1,
  input  logic [WIDTH-1:0]         wd2,
  input  logic [$clog2(DEPTH)-1:0] wa2,
  input  logic                     we2,
  input  logic [(WIDTH-1)/8:0]     bwe2,
  output logic [WIDTH-1:0]         rd1
);

  logic [WIDTH-1:0]                mem[DEPTH-1:0];
  localparam                      SRAMWIDTH = 32;
  localparam                      SRAMNUMSETS = SRAMWIDTH/WIDTH;      

  // ***************************************************************************
  // TRUE Smem macro
  // ***************************************************************************

  if ((`USE_SRAM == 1) & (WIDTH == 68) & (DEPTH == 1024)) begin
    
    ram2p1r1wbe_1024x68 memory1(.CLKA(clk), .CLKB(clk), 
      .CEBA(~ce1), .CEBB(~ce2),
      .WEBA('0), .WEBB(~we2),            
      .AA(ra1), .AB(wa2),
      .DA('0),
      .DB(wd2),
      .BWEBA('0), .BWEBB('1),
      .QA(rd1),
      .QB());

  end else if ((`USE_SRAM == 1) & (WIDTH == 36) & (DEPTH == 1024)) begin
    
    ram2p1r1wbe_1024x36 memory1(.CLKA(clk), .CLKB(clk), 
      .CEBA(~ce1), .CEBB(~ce2),
      .WEBA('0), .WEBB(~we2),            
      .AA(ra1), .AB(wa2),
      .DA('0),
      .DB(wd2),
      .BWEBA('0), .BWEBB('1),
      .QA(rd1),
      .QB());      

  end else if ((`USE_SRAM == 1) & (WIDTH == 2) & (DEPTH == 1024)) begin

    logic [SRAMWIDTH-1:0]     SRAMReadData;      
    logic [SRAMWIDTH-1:0]     SRAMWriteData;      
    logic [SRAMWIDTH-1:0]     RD1Sets[SRAMNUMSETS-1:0];
    logic [SRAMNUMSETS-1:0]   SRAMBitMaskPre;      
    logic [SRAMWIDTH-1:0]     SRAMBitMask;      
    logic [$clog2(DEPTH)-1:0] RA1Q;
    
    onehotdecoder #($clog2(SRAMNUMSETS)) oh1(wa2[$clog2(SRAMNUMSETS)-1:0], SRAMBitMaskPre);      
    genvar                    index;
    for (index = 0; index < SRAMNUMSETS; index++) begin:readdatalinesetsmux
      assign RD1Sets[index] = SRAMReadData[(index*WIDTH)+WIDTH-1 : (index*WIDTH)];   
      assign SRAMWriteData[index*2+1:index*2] = wd2;
      assign SRAMBitMask[index*2+1:index*2] = {2{SRAMBitMaskPre[index]}};      
    end
    flopen #($clog2(DEPTH)) mem_reg1 (clk, ce1, ra1, RA1Q);      
    assign rd1 = RD1Sets[RA1Q[$clog2(SRAMWIDTH)-1:0]];      
    ram2p1r1wbe_64x32 memory2(.CLKA(clk), .CLKB(clk), 
      .CEBA(~ce1), .CEBB(~ce2),
      .WEBA('0), .WEBB(~we2),            
      .AA(ra1[$clog2(DEPTH)-1:$clog2(SRAMNUMSETS)]), 
      .AB(wa2[$clog2(DEPTH)-1:$clog2(SRAMNUMSETS)]),
      .DA('0),
      .DB(SRAMWriteData),
      .BWEBA('0), .BWEBB(SRAMBitMask),
      .QA(SRAMReadData),
      .QB());

  end else begin
    
    // ***************************************************************************
    // READ first SRAM model
    // ***************************************************************************
    integer i;
    
    // Read
    logic [$clog2(DEPTH)-1:0] ra1d;
    flopen #($clog2(DEPTH)) adrreg(clk, ce1, ra1, ra1d);
    assign rd1 = mem[ra1d];

    /*      // Read
     always_ff @(posedge clk) 
     if(ce1) rd1 <= #1 mem[ra1]; */
    
    // Write divided into part for bytes and part for extra msbs
    // coverage off     
    //   when byte write enables are tied high, the last IF is always taken
    if(WIDTH >= 8) 
      always @(posedge clk) 
        if (ce2 & we2) 
          for(i = 0; i < WIDTH/8; i++) 
            if(bwe2[i]) mem[wa2][i*8 +: 8] <= #1 wd2[i*8 +: 8];
    // coverage on
  
    if (WIDTH%8 != 0) // handle msbs if width not a multiple of 8
      always @(posedge clk) 
        if (ce2 & we2 & bwe2[WIDTH/8])
          mem[wa2][WIDTH-1:WIDTH-WIDTH%8] <= #1 wd2[WIDTH-1:WIDTH-WIDTH%8];
  end
  
endmodule


///////////////////////////////////////////
// ram2p1rwbe_1024x68.sv
//
// Written: james.stine@okstate.edu 28 January 2023
// Modified: 
//
// Purpose: RAM wrapper for instantiating RAM IP
// 
// A component of the CORE-V-WALLY configurable RISC-V project.
// 
// Copyright (C) 2021-23 Harvey Mudd College & Oklahoma State University
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the “License”); you may not use this file 
// except in compliance with the License, or, at your option, the Apache License version 2.0. You 
// may obtain a copy of the License at
//
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work distributed under the 
// License is distributed on an “AS IS” BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, 
// either express or implied. See the License for the specific language governing permissions 
// and limitations under the License.
////////////////////////////////////////////////////////////////////////////////////////////////

module ram2p1r1wbe_1024x68( 
  input  logic          CLKA, 
  input  logic          CLKB, 
  input  logic          CEBA, 
  input  logic          CEBB, 
  input  logic          WEBA,
  input  logic          WEBB,
  input  logic [9:0]    AA, 
  input  logic [9:0]    AB, 
  input  logic [67:0]   DA,
  input  logic [67:0]   DB,
  input  logic [67:0]   BWEBA, 
  input  logic [67:0]   BWEBB, 
  output logic [67:0]   QA,
  output logic [67:0]   QB
);

   // replace "generic1024x68RAM" with "TSDN..1024X68.." module from your memory vendor
   //generic1024x68RAM sramIP (.CLKA, .CLKB, .CEBA, .CEBB, .WEBA, .WEBB, 
   //         .AA, .AB, .DA, .DB, .BWEBA, .BWEBB, .QA, .QB);
  TSDN28HPCPA1024X68M4MW sramIP(.CLKA, .CLKB, .CEBA, .CEBB, .WEBA, .WEBB, 
    .AA, .AB, .DA, .DB, .BWEBA, .BWEBB, .QA, .QB);

endmodule

module flopen #(parameter WIDTH = 8) (
  input  logic             clk, en,
  input  logic [WIDTH-1:0] d, 
  output logic [WIDTH-1:0] q);

  always_ff @(posedge clk)
    if (en) q <= #1 d;
endmodule

