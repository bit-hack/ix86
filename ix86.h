/***************************************************************************
 *   Copyright (C) 2007 Ryan Schultz, PCSX-df Team, PCSX team              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

// ix86 definitions v0.5.1
//  Authors: linuzappz <linuzappz@pcsx.net>
//           alexey silinov

// nice reference at:
//   https://www.felixcloutier.com/x86/index.html

#pragma once

#include <cstdint>

struct ix86 {

  enum gp_reg_t {
    EAX = 0,
    ECX = 1,
    EDX = 2,
    EBX = 3,
    ESP = 4,
    EBP = 5,
    ESI = 6,
    EDI = 7,
  };

  enum scale_t {
    SCALE1 = 0,
    SCALE2 = 1,
    SCALE4 = 2,
    SCALE8 = 3,
  };

  enum cmov_cc_t {
    CC_OF = 0x40, // overflow         (OF=1)
    CC_NO = 0x41, // not overflow     (OF=0)
    CC_BL = 0x42, // below            (CF=1)
    CC_AE = 0x43, // above or equal   (CF=0)
    CC_EQ = 0x44, // equal            (ZF=1)
    CC_NE = 0x45, // not equal        (ZF=0)
    CC_BE = 0x46, // below or equal   (CF=1 or ZF=1)
    CC_AB = 0x47, // above            (CF=0 and ZF=0)
    CC_SF = 0x48, // sign             (SF=1)
    CC_NS = 0x49, // not sign         (SF=0)
    CC_PF = 0x4a, // parity           (PF=1)
    CC_NP = 0x4b, // parity odd       (PF=0)
    CC_LT = 0x4c, // less             (SF!=OF)
    CC_GE = 0x4d, // greater or equal (SF=OF)
    CC_LE = 0x4e, // less or equal    (ZF=1 or SF!=OF)
    CC_GT = 0x4f, // greater          (ZF=0 and SF=OF)
  };

  ix86(void *ptr, size_t size)
      : x86Ptr((int8_t *)ptr), endPtr((int8_t *)ptr + size) {
  }

  // write value to code stream
  void write8(const uint8_t val);
  void write16(const uint16_t val);
  void write32(const uint32_t val);
  void write32(void *val);
  void write64(const uint64_t val);

  // set jump target
  void setJ8(uint8_t *j8);
  void setJ32(uint32_t *j32);

  // align instruction stream
  void align(int32_t bytes);

  void ModRM(int32_t mod, int32_t rm, int32_t reg);
  void SibSB(int32_t ss, int32_t rm, int32_t index);

  // mov instructions

  // mov r32 to r32
  void MOV32RtoR(gp_reg_t to, gp_reg_t from);
  // mov r32 to m32
  void MOV32RtoM(void *to, gp_reg_t from);
  // mov m32 to r32
  void MOV32MtoR(gp_reg_t to, void *from);
  // mov [r32] to r32
  void MOV32RmtoR(gp_reg_t to, gp_reg_t from);
  // mov [r32][r32*scale] to r32
  void MOV32RmStoR(gp_reg_t to, gp_reg_t from, gp_reg_t from2, scale_t scale);
  // mov r32 to [r32]
  void MOV32RtoRm(gp_reg_t to, gp_reg_t from);
  // mov r32 to [r32][r32*scale]
  void MOV32RtoRmS(gp_reg_t to, gp_reg_t to2, scale_t scale, gp_reg_t from);
  // mov imm32 to r32
  void MOV32ItoR(gp_reg_t to, uint32_t from);
  // mov imm32 to m32
  void MOV32ItoM(void *to, uint32_t from);

  // mov r16 to m16
  void MOV16RtoM(void *to, gp_reg_t from);
  // mov m16 to r16
  void MOV16MtoR(gp_reg_t to, void *from);
  // mov imm16 to m16
  void MOV16ItoM(void *to, uint16_t from);

  // mov r8 to m8
  void MOV8RtoM(void *to, gp_reg_t from);
  // mov m8 to r8
  void MOV8MtoR(gp_reg_t to, void *from);
  // mov imm8 to m8
  void MOV8ItoM(void *to, uint8_t from);

  // mov sign extend r8 to r32
  void MOVSX32R8toR(gp_reg_t to, gp_reg_t from);
  // mov sign extend m8 to r32
  void MOVSX32M8toR(gp_reg_t to, void *from);
  // mov sign extend r16 to r32
  void MOVSX32R16toR(gp_reg_t to, gp_reg_t from);
  // mov sign extend m16 to r32
  void MOVSX32M16toR(gp_reg_t to, void *from);

  // mov zero extend r8 to r32
  void MOVZX32R8toR(gp_reg_t to, gp_reg_t from);
  // mov zero extend m8 to r32
  void MOVZX32M8toR(gp_reg_t to, void *from);
  // mov zero extend r16 to r32
  void MOVZX32R16toR(gp_reg_t to, gp_reg_t from);
  // mov zero extend m16 to r32
  void MOVZX32M16toR(gp_reg_t to, void *from);

  // conditional move
  void CMOV32RtoR(cmov_cc_t cc, gp_reg_t to, gp_reg_t from);
  // conditional move
  void CMOV32MtoR(cmov_cc_t cc, gp_reg_t to, void *from);

  // arithmetic instructions

  // add imm32 to r32
  void ADD32ItoR(gp_reg_t to, uint32_t from);
  // add imm32 to m32
  void ADD32ItoM(void *to, uint32_t from);
  // add r32 to r32
  void ADD32RtoR(gp_reg_t to, gp_reg_t from);
  // add r32 to m32
  void ADD32RtoM(void *to, gp_reg_t from);
  // add m32 to r32
  void ADD32MtoR(gp_reg_t to, void *from);

  // adc imm32 to r32
  void ADC32ItoR(gp_reg_t to, uint32_t from);
  // adc r32 to r32
  void ADC32RtoR(gp_reg_t to, gp_reg_t from);
  // adc m32 to r32
  void ADC32MtoR(gp_reg_t to, void *from);

  // inc r32
  void INC32R(gp_reg_t to);
  // inc m32
  void INC32M(void *to);

  // sub imm32 to r32
  void SUB32ItoR(gp_reg_t to, uint32_t from);
  // sub r32 to r32
  void SUB32RtoR(gp_reg_t to, gp_reg_t from);
  // sub m32 to r32
  void SUB32MtoR(gp_reg_t to, void *from);

  // sbb imm32 to r32
  void SBB32ItoR(gp_reg_t to, uint32_t from);
  // sbb r32 to r32
  void SBB32RtoR(gp_reg_t to, gp_reg_t from);
  // sbb m32 to r32
  void SBB32MtoR(gp_reg_t to, void *from);

  // dec r32
  void DEC32R(gp_reg_t to);
  // dec m32
  void DEC32M(void *to);

  // mul eax by r32 to edx:eax
  void MUL32R(gp_reg_t from);
  // mul eax by m32 to edx:eax
  void MUL32M(void *from);

  // imul eax by r32 to edx:eax
  void IMUL32R(gp_reg_t from);
  // imul eax by m32 to edx:eax
  void IMUL32M(void *from);
  // imul r32 by r32 to r32
  void IMUL32RtoR(gp_reg_t to, gp_reg_t from);

  // div eax by r32 to edx:eax
  void DIV32R(gp_reg_t from);
  // div eax by m32 to edx:eax
  void DIV32M(void *from);

  // idiv eax by r32 to edx:eax
  void IDIV32R(gp_reg_t from);
  // idiv eax by m32 to edx:eax
  void IDIV32M(void *from);

  // shifting instructions

  // rotate carry right
  void RCR32ItoR(int32_t to, int32_t from);

  // shl imm8 to r32
  void SHL32ItoR(gp_reg_t to, uint8_t from);
  // shl cl to r32
  void SHL32CLtoR(gp_reg_t to);

  // shr imm8 to r32
  void SHR32ItoR(gp_reg_t to, uint8_t from);
  // shr cl to r32
  void SHR32CLtoR(gp_reg_t to);

  // sar imm8 to r32
  void SAR32ItoR(gp_reg_t to, uint8_t from);
  // sar cl to r32
  void SAR32CLtoR(gp_reg_t to);

  // logical instructions

  // or imm32 to r32
  void OR32ItoR(gp_reg_t to, uint32_t from);
  // or imm32 to m32
  void OR32ItoM(void *to, uint32_t from);
  // or r32 to r32
  void OR32RtoR(gp_reg_t to, gp_reg_t from);
  // or r32 to m32
  void OR32RtoM(void *to, gp_reg_t from);
  // or m32 to r32
  void OR32MtoR(gp_reg_t to, void *from);

  // xor imm32 to r32
  void XOR32ItoR(gp_reg_t to, uint32_t from);
  // xor imm32 to m32
  void XOR32ItoM(void *to, uint32_t from);
  // xor r32 to r32
  void XOR32RtoR(gp_reg_t to, gp_reg_t from);
  // xor r32 to m32
  void XOR32RtoM(void *to, gp_reg_t from);
  // xor m32 to r32
  void XOR32MtoR(gp_reg_t to, void *from);

  // and imm32 to r32
  void AND32ItoR(gp_reg_t to, uint32_t from);
  // and imm32 to m32
  void AND32ItoM(void *to, uint32_t from);
  // and r32 to r32
  void AND32RtoR(gp_reg_t to, gp_reg_t from);
  // and r32 to m32
  void AND32RtoM(void *to, gp_reg_t from);
  // and m32 to r32
  void AND32MtoR(gp_reg_t to, void *from);

  // not r32
  void NOT32R(gp_reg_t from);
  // neg r32
  void NEG32R(gp_reg_t from);

  // jump instructions

  // jump helpers
  uint8_t *J8Rel(int32_t cc, int32_t to);
  uint32_t *J32Rel(int32_t cc, int32_t to);

  // jmp rel8
  uint8_t *JMP8(uint8_t to);
  // jmp rel32
  uint32_t *JMP32(uint32_t to);
  // jmp r32
  void JMP32R(int32_t to);

  // je rel8
  uint8_t *JE8(uint8_t to);
  // jz rel8
  uint8_t *JZ8(uint8_t to);
  // jg rel8
  uint8_t *JG8(uint8_t to);
  // jge rel8
  uint8_t *JGE8(uint8_t to);
  // jl rel8
  uint8_t *JL8(uint8_t to);
  // jle rel8
  uint8_t *JLE8(uint8_t to);
  // jne rel8
  uint8_t *JNE8(uint8_t to);
  // jnz rel8
  uint8_t *JNZ8(uint8_t to);
  // jng rel8
  uint8_t *JNG8(uint8_t to);
  // jnge rel8
  uint8_t *JNGE8(uint8_t to);
  // jnl rel8
  uint8_t *JNL8(uint8_t to);
  // jnle rel8
  uint8_t *JNLE8(uint8_t to);
  // jo rel8
  uint8_t *JO8(uint8_t to);
  // jno rel8
  uint8_t *JNO8(uint8_t to);

  // je rel32
  uint32_t *JE32(uint32_t to);
  // jz rel32
  uint32_t *JZ32(uint32_t to);
  // jg rel32
  uint32_t *JG32(uint32_t to);
  // jge rel32
  uint32_t *JGE32(uint32_t to);
  // jl rel32
  uint32_t *JL32(uint32_t to);
  // jle rel32
  uint32_t *JLE32(uint32_t to);
  // jne rel32
  uint32_t *JNE32(uint32_t to);
  // jnz rel32
  uint32_t *JNZ32(uint32_t to);
  // jng rel32
  uint32_t *JNG32(uint32_t to);
  // jnge rel32
  uint32_t *JNGE32(uint32_t to);
  // jnl rel32
  uint32_t *JNL32(uint32_t to);
  // jnle rel32
  uint32_t *JNLE32(uint32_t to);
  // jo rel32
  uint32_t *JO32(uint32_t to);
  // jno rel32
  uint32_t *JNO32(uint32_t to);

  // call func
  void CALLFunc(uint32_t func);
  // call rel32
  void CALL32(uint32_t to);
  // call r32
  void CALL32R(gp_reg_t to);
  // call m32
  void CALL32M(void *to);

  // misc instructions

  // bit test
  void BT32ItoR(gp_reg_t to, int32_t from);

  // cmp imm32 to r32
  void CMP32ItoR(gp_reg_t to, uint32_t from);
  // cmp imm32 to m32
  void CMP32ItoM(void *to, uint32_t from);
  // cmp r32 to r32
  void CMP32RtoR(gp_reg_t to, gp_reg_t from);
  // cmp m32 to r32
  void CMP32MtoR(gp_reg_t to, void *from);

  // test imm32 to r32
  void TEST32ItoR(gp_reg_t to, uint32_t from);
  // test r32 to r32
  void TEST32RtoR(gp_reg_t to, gp_reg_t from);

  // set byte on condition
  void SET8R(int32_t cc, gp_reg_t to);
  // set byte if sign (SF=1)
  void SETS8R(gp_reg_t to);
  // set byte if less (SF!=OF)
  void SETL8R(gp_reg_t to);
  // Set byte if below (CF=1)
  void SETB8R(gp_reg_t to);
  // Set byte if not equal (ZF=0)
  void SETNZ8R(gp_reg_t to);

  // convert byte to word
  void CBW();
  // convert word to doubleword
  void CWD();
  // convert doubleword to quadword
  void CDQ();

  // push r32 to stack
  void PUSH32R(gp_reg_t from);
  // push m32 to stack
  void PUSH32M(void *from);
  // push imm32 to stack
  void PUSH32I(uint32_t from);

  // pop r32 from stack
  void POP32R(gp_reg_t from);

  // push All General-Purpose Registers
  void PUSHA32();
  // pop All General-Purpose Registers
  void POPA32();

  // return
  void RET();

protected:
  int8_t *x86Ptr;
  const int8_t *endPtr;

}; // struct ix86
