/***************************************************************/
/*                                                             */
/*   RISC-V RV32 Instruction Level Simulator                   */
/*                                                             */
/*   ECEN 4243                                                 */
/*   Oklahoma State University                                 */
/*                                                             */
/***************************************************************/

#ifndef _SIM_ISA_H_
#define _SIM_ISA_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "shell.h"

//
// MACRO: Check sign bit (sb) of (v) to see if negative
//   if no, just give number
//   if yes, sign extend (e.g., 0x80_0000 -> 0xFF80_0000)
//
#define SIGNEXT(v, sb) ((v) | (((v) & (1 << (sb))) ? ~((1 << (sb))-1) : 0))

#define ZeroExtend(V) (0x00000000|(V))





// I Instructions
//int LB (char* i_);
//int LH (char* i_);
//int LW (char* i_);
//int LBU (char* i_);
//int LHU (char* i_);
//int SLLI (char* i_);
//int SLTI (char* i_);
//int SLTIU (char* i_);
//int XORI (char* i_);
//int SRLI (char* i_);
//int SRAI (char* i_);
//int ORI (char* i_);
//int ANDI (char* i_);

int ADDI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] + SIGNEXT(Imm,12);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int XORI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] ^ SIGNEXT(Imm,12);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}
int ORI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] | SIGNEXT(Imm,12);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}
int ANDI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] & SIGNEXT(Imm,12);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}
//set less than imm
int SLTI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = (CURRENT_STATE.REGS[Rs1] < SIGNEXT(Imm,12))?1:0;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
}
//shift left logical imm
int SLLI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = (CURRENT_STATE.REGS[Rs1] << SIGNEXT(Imm,5));
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}

int SLTIU (int Rd, int Rs1, int Imm, int Funct3) {
  //figure out what zero extends means
  int cur = 0;
  cur = ZeroExtend((CURRENT_STATE.REGS[Rs1] < SIGNEXT(Imm,12))?1:0);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}

int LB (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  uint32_t address_data = SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),8);
  cur = mem_read_32(address_data) ;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}



int LH (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  uint32_t address_data = SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),16);
  cur = mem_read_32(address_data) ;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}

int LW (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  uint32_t address_data = (CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12));
  cur = mem_read_32(address_data) ;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}

int LBU (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  uint32_t address_data =ZeroExtend(SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),8));
  cur = mem_read_32(address_data) ;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}



int LHU (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  uint32_t address_data = ZeroExtend(SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),16));
  cur = mem_read_32(address_data) ;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}

//Shift Right Locgical Imm
int SRLI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = (CURRENT_STATE.REGS[Rs1] >> SIGNEXT(Imm,5));
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}
//Shift Right Arith Imm
int SRAI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = (CURRENT_STATE.REGS[Rs1] >> SIGNEXT(Imm,5));
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
  
}



// U Instruction
//int AUIPC (char* i_);
//int LUI (char* i_);
int LUI (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = SIGNEXT(Imm,12)<<12;
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}
int AUIPC (int Rd, int Rs1, int Imm, int Funct3) {

  int cur = 0;
  cur = (CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,12)<<12);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}


// S Instruction
//int SB (char* i_);
//int SH (char* i_);
//int SW (char* i_);

/*

int SB (int Rs1, int Rs2, int Imm, int Funct3) {

  uint32_t address_data = SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),8);
  mem_write_32( address_data,  SIGNEXT(CURRENT_STATE.REGS[Rs2],8));
  return 0;
  
}

int SH (int Rs1, int Rs2, int Imm, int Funct3) {

  uint32_t address_data = SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),16);
  mem_write_32( address_data,  SIGNEXT(CURRENT_STATE.REGS[Rs2],16));
  return 0;
  
}

int SW (int Rs1, int Rs2, int Imm, int Funct3) {

  uint32_t address_data = SIGNEXT((CURRENT_STATE.REGS[Rs1]+SIGNEXT(Imm,12)),32);
  mem_write_32( address_data,  SIGNEXT(CURRENT_STATE.REGS[Rs2],32));
  return 0;
  
}
*/

// R instruction
//int SUB (char* i_);
//int SLL (char* i_);
//int SLT (char* i_);
//int SLTU (char* i_);
//int XOR (char* i_);
//int SRL (char* i_);
//int SRA (char* i_);
//int OR (char* i_);
//int AND (char* i_);

int ADD (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] + CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int SLL (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] << CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int SRL (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] >> CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int SRA (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = SIGNEXT((CURRENT_STATE.REGS[Rs1] >> CURRENT_STATE.REGS[Rs2]),5);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int SLT (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = ((CURRENT_STATE.REGS[Rs1] < CURRENT_STATE.REGS[Rs2])?1:0);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int SLTU (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = ZeroExtend((CURRENT_STATE.REGS[Rs1] < CURRENT_STATE.REGS[Rs2])?1:0);
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int XOR (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] ^ CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}
int OR (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] | CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int AND (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] & CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;

}

int SUB (int Rd, int Rs1, int Rs2, int Funct3) {

  int cur = 0;
  cur = CURRENT_STATE.REGS[Rs1] - CURRENT_STATE.REGS[Rs2];
  NEXT_STATE.REGS[Rd] = cur;
  return 0;
}






// B instructions
//int BEQ (char* i_);
//int BLT (char* i_);
//int BGE (char* i_);
//int BLTU (char* i_);
//int BGEU (char* i_);



int BNE (int Rs1, int Rs2, int Imm, int Funct3) {

  int cur = 0;
  Imm = Imm << 1;
  if (CURRENT_STATE.REGS[Rs1] != CURRENT_STATE.REGS[Rs2])
    NEXT_STATE.PC = (CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,13));
  return 0;

}

int BEQ (int Rs1, int Rs2, int Imm, int Funct3) {

  int cur = 0;
  Imm = Imm << 1;
  if (CURRENT_STATE.REGS[Rs1] == CURRENT_STATE.REGS[Rs2])
    NEXT_STATE.PC = (CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,13));
  return 0;

}

int BLT (int Rs1, int Rs2, int Imm, int Funct3) {

  int cur = 0;
  Imm = Imm << 1;
  if (CURRENT_STATE.REGS[Rs1] < CURRENT_STATE.REGS[Rs2])
    NEXT_STATE.PC = (CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,13));
  return 0;

}

int BGE (int Rs1, int Rs2, int Imm, int Funct3) {

  int cur = 0;
  Imm = Imm << 1;
  if (CURRENT_STATE.REGS[Rs1] >= CURRENT_STATE.REGS[Rs2])
    NEXT_STATE.PC = (CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,13));
  return 0;

}

int BLTU (int Rs1, int Rs2, int Imm, int Funct3) {

  int cur = 0;
  Imm = Imm << 1;
  if (CURRENT_STATE.REGS[Rs1] < CURRENT_STATE.REGS[Rs2])
    NEXT_STATE.PC = ZeroExtend((CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,13)));
  return 0;

}

int BGEU (int Rs1, int Rs2, int Imm, int Funct3) {

  int cur = 0;
  Imm = Imm << 1;
  if (CURRENT_STATE.REGS[Rs1] >= CURRENT_STATE.REGS[Rs2])
    NEXT_STATE.PC = ZeroExtend((CURRENT_STATE.PC + 4) + (SIGNEXT(Imm,13)));
  return 0;

}





// I instruction
int JALR (char* i_);


// J instruction
int JAL (char* i_);

int ECALL (char* i_){return 0;}

#endif
