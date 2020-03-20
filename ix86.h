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

/*
 * ix86 definitions v0.5.1
 *  Authors: linuzappz <linuzappz@pcsx.net>
 *           alexey silinov
 */
#pragma once

#include <cstdint>


struct ix86 {

  enum {
    // x86Flags defines
    X86FLAG_FPU       = 0x00000001,
    X86FLAG_VME       = 0x00000002,
    X86FLAG_DEBUGEXT  = 0x00000004,
    X86FLAG_4MPAGE    = 0x00000008,
    X86FLAG_TSC       = 0x00000010,
    X86FLAG_MSR       = 0x00000020,
    X86FLAG_PAE       = 0x00000040,
    X86FLAG_MCHKXCP   = 0x00000080,
    X86FLAG_CMPXCHG8B = 0x00000100,
    X86FLAG_APIC      = 0x00000200,
    X86FLAG_SYSENTER  = 0x00000800,
    X86FLAG_MTRR      = 0x00001000,
    X86FLAG_GPE       = 0x00002000,
    X86FLAG_MCHKARCH  = 0x00004000,
    X86FLAG_CMOV      = 0x00008000,
    X86FLAG_PAT       = 0x00010000,
    X86FLAG_PSE36     = 0x00020000,
    X86FLAG_PN        = 0x00040000,
    X86FLAG_MMX       = 0x00800000,
    X86FLAG_FXSAVE    = 0x01000000,
    X86FLAG_SSE       = 0x02000000,

    // x86EFlags defines

    X86EFLAG_MMXEXT   = 0x00400000,
    X86EFLAG_3DNOWEXT = 0x40000000,
    X86EFLAG_3DNOW    = 0x80000000,

    EAX = 0,
    EBX = 3,
    ECX = 1,
    EDX = 2,
    ESI = 6,
    EDI = 7,
    EBP = 5,
    ESP = 4,

    MM0 = 0,
    MM1 = 1,
    MM2 = 2,
    MM3 = 3,
    MM4 = 4,
    MM5 = 5,
    MM6 = 6,
    MM7 = 7,

    XMM0 = 0,
    XMM1 = 1,
    XMM2 = 2,
    XMM3 = 3,
    XMM4 = 4,
    XMM5 = 5,
    XMM6 = 6,
    XMM7 = 7,
  };

  ix86(void *ptr, size_t size)
      : x86Ptr((int8_t *)ptr), endPtr((int8_t *)ptr + size) {
  }

  void write8(const uint8_t val);
  void write16(const uint16_t val);
  void write32(const uint32_t val);
  void write64(const uint64_t val);

  void x86SetJ8(uint8_t *j8);
  void x86SetJ32(uint32_t *j32);
  void x86Align(int32_t bytes);

  void ModRM(int32_t mod, int32_t rm, int32_t reg);
  void SibSB(int32_t ss, int32_t rm, int32_t index);

  void SET8R(int32_t cc, int32_t to);

  uint8_t *J8Rel(int32_t cc, int32_t to);
  uint32_t *J32Rel(int32_t cc, int32_t to);

  // IX86 intructions

  /*
   * scale values:
   *  0 - *1
   *  1 - *2
   *  2 - *4
   *  3 - *8
   */

  void CMOV32RtoR(int32_t cc, int32_t to, int32_t from);
  void CMOV32MtoR(int32_t cc, int32_t to, int32_t from);

  // mov instructions

  /* mov r32 to r32 */
  void MOV32RtoR(int32_t to, int32_t from);
  /* mov r32 to m32 */
  void MOV32RtoM(uint32_t to, int32_t from);
  /* mov m32 to r32 */
  void MOV32MtoR(int32_t to, uint32_t from);
  /* mov [r32] to r32 */
  void MOV32RmtoR(int32_t to, int32_t from);
  /* mov [r32][r32*scale] to r32 */
  void MOV32RmStoR(int32_t to, int32_t from, int32_t from2, int32_t scale);
  /* mov r32 to [r32] */
  void MOV32RtoRm(int32_t to, int32_t from);
  /* mov r32 to [r32][r32*scale] */
  void MOV32RtoRmS(int32_t to, int32_t to2, int32_t scale, int32_t from);
  /* mov imm32 to r32 */
  void MOV32ItoR(int32_t to, uint32_t from);
  /* mov imm32 to m32 */
  void MOV32ItoM(uint32_t to, uint32_t from);

  /* mov r16 to m16 */
  void MOV16RtoM(uint32_t to, int32_t from);
  /* mov m16 to r16 */
  void MOV16MtoR(int32_t to, uint32_t from);
  /* mov imm16 to m16 */
  void MOV16ItoM(uint32_t to, uint16_t from);

  /* mov r8 to m8 */
  void MOV8RtoM(uint32_t to, int32_t from);
  /* mov m8 to r8 */
  void MOV8MtoR(int32_t to, uint32_t from);
  /* mov imm8 to m8 */
  void MOV8ItoM(uint32_t to, uint8_t from);

  /* movsx r8 to r32 */
  void MOVSX32R8toR(int32_t to, int32_t from);
  /* movsx m8 to r32 */
  void MOVSX32M8toR(int32_t to, uint32_t from);
  /* movsx r16 to r32 */
  void MOVSX32R16toR(int32_t to, int32_t from);
  /* movsx m16 to r32 */
  void MOVSX32M16toR(int32_t to, uint32_t from);

  /* movzx r8 to r32 */
  void MOVZX32R8toR(int32_t to, int32_t from);
  /* movzx m8 to r32 */
  void MOVZX32M8toR(int32_t to, uint32_t from);
  /* movzx r16 to r32 */
  void MOVZX32R16toR(int32_t to, int32_t from);
  /* movzx m16 to r32 */
  void MOVZX32M16toR(int32_t to, uint32_t from);

  /* cmovne r32 to r32 */
  void CMOVNE32RtoR(int32_t to, int32_t from);
  /* cmovne m32 to r32*/
  void CMOVNE32MtoR(int32_t to, uint32_t from);
  /* cmove r32 to r32*/
  void CMOVE32RtoR(int32_t to, int32_t from);
  /* cmove m32 to r32*/
  void CMOVE32MtoR(int32_t to, uint32_t from);
  /* cmovg r32 to r32*/
  void CMOVG32RtoR(int32_t to, int32_t from);
  /* cmovg m32 to r32*/
  void CMOVG32MtoR(int32_t to, uint32_t from);
  /* cmovge r32 to r32*/
  void CMOVGE32RtoR(int32_t to, int32_t from);
  /* cmovge m32 to r32*/
  void CMOVGE32MtoR(int32_t to, uint32_t from);
  /* cmovl r32 to r32*/
  void CMOVL32RtoR(int32_t to, int32_t from);
  /* cmovl m32 to r32*/
  void CMOVL32MtoR(int32_t to, uint32_t from);
  /* cmovle r32 to r32*/
  void CMOVLE32RtoR(int32_t to, int32_t from);
  /* cmovle m32 to r32*/
  void CMOVLE32MtoR(int32_t to, uint32_t from);

  // arithmetic instructions

  /* add imm32 to r32 */
  void ADD32ItoR(int32_t to, uint32_t from);
  /* add imm32 to m32 */
  void ADD32ItoM(uint32_t to, uint32_t from);
  /* add r32 to r32 */
  void ADD32RtoR(int32_t to, int32_t from);
  /* add r32 to m32 */
  void ADD32RtoM(uint32_t to, int32_t from);
  /* add m32 to r32 */
  void ADD32MtoR(int32_t to, uint32_t from);

  /* adc imm32 to r32 */
  void ADC32ItoR(int32_t to, uint32_t from);
  /* adc r32 to r32 */
  void ADC32RtoR(int32_t to, int32_t from);
  /* adc m32 to r32 */
  void ADC32MtoR(int32_t to, uint32_t from);

  /* inc r32 */
  void INC32R(int32_t to);
  /* inc m32 */
  void INC32M(uint32_t to);

  /* sub imm32 to r32 */
  void SUB32ItoR(int32_t to, uint32_t from);
  /* sub r32 to r32 */
  void SUB32RtoR(int32_t to, int32_t from);
  /* sub m32 to r32 */
  void SUB32MtoR(int32_t to, uint32_t from);

  /* sbb imm32 to r32 */
  void SBB32ItoR(int32_t to, uint32_t from);
  /* sbb r32 to r32 */
  void SBB32RtoR(int32_t to, int32_t from);
  /* sbb m32 to r32 */
  void SBB32MtoR(int32_t to, uint32_t from);

  /* dec r32 */
  void DEC32R(int32_t to);
  /* dec m32 */
  void DEC32M(uint32_t to);

  /* mul eax by r32 to edx:eax */
  void MUL32R(int32_t from);
  /* mul eax by m32 to edx:eax */
  void MUL32M(uint32_t from);

  /* imul eax by r32 to edx:eax */
  void IMUL32R(int32_t from);
  /* imul eax by m32 to edx:eax */
  void IMUL32M(uint32_t from);
  /* imul r32 by r32 to r32 */
  void IMUL32RtoR(int32_t to, int32_t from);

  /* div eax by r32 to edx:eax */
  void DIV32R(int32_t from);
  /* div eax by m32 to edx:eax */
  void DIV32M(uint32_t from);

  /* idiv eax by r32 to edx:eax */
  void IDIV32R(int32_t from);
  /* idiv eax by m32 to edx:eax */
  void IDIV32M(uint32_t from);

  // shifting instructions

  /* shl imm8 to r32 */
  void SHL32ItoR(int32_t to, uint8_t from);
  /* shl cl to r32 */
  void SHL32CLtoR(int32_t to);

  /* shr imm8 to r32 */
  void SHR32ItoR(int32_t to, uint8_t from);
  /* shr cl to r32 */
  void SHR32CLtoR(int32_t to);

  /* sar imm8 to r32 */
  void SAR32ItoR(int32_t to, uint8_t from);
  /* sar cl to r32 */
  void SAR32CLtoR(int32_t to);

/* sal imm8 to r32 */
#define SAL32ItoR SHL32ItoR
/* sal cl to r32 */
#define SAL32CLtoR SHL32CLtoR

  // logical instructions

  /* or imm32 to r32 */
  void OR32ItoR(int32_t to, uint32_t from);
  /* or imm32 to m32 */
  void OR32ItoM(uint32_t to, uint32_t from);
  /* or r32 to r32 */
  void OR32RtoR(int32_t to, int32_t from);
  /* or r32 to m32 */
  void OR32RtoM(uint32_t to, int32_t from);
  /* or m32 to r32 */
  void OR32MtoR(int32_t to, uint32_t from);

  /* xor imm32 to r32 */
  void XOR32ItoR(int32_t to, uint32_t from);
  /* xor imm32 to m32 */
  void XOR32ItoM(uint32_t to, uint32_t from);
  /* xor r32 to r32 */
  void XOR32RtoR(int32_t to, int32_t from);
  /* xor r32 to m32 */
  void XOR32RtoM(uint32_t to, int32_t from);
  /* xor m32 to r32 */
  void XOR32MtoR(int32_t to, uint32_t from);

  /* and imm32 to r32 */
  void AND32ItoR(int32_t to, uint32_t from);
  /* and imm32 to m32 */
  void AND32ItoM(uint32_t to, uint32_t from);
  /* and r32 to r32 */
  void AND32RtoR(int32_t to, int32_t from);
  /* and r32 to m32 */
  void AND32RtoM(uint32_t to, int32_t from);
  /* and m32 to r32 */
  void AND32MtoR(int32_t to, uint32_t from);

  /* not r32 */
  void NOT32R(int32_t from);
  /* neg r32 */
  void NEG32R(int32_t from);

  // jump instructions

  /* jmp rel8 */
  uint8_t *JMP8(uint8_t to);

  /* jmp rel32 */
  uint32_t *JMP32(uint32_t to);
  /* jmp r32 */
  void JMP32R(int32_t to);

  /* je rel8 */
  uint8_t *JE8(uint8_t to);
  /* jz rel8 */
  uint8_t *JZ8(uint8_t to);
  /* jg rel8 */
  uint8_t *JG8(uint8_t to);
  /* jge rel8 */
  uint8_t *JGE8(uint8_t to);
  /* jl rel8 */
  uint8_t *JL8(uint8_t to);
  /* jle rel8 */
  uint8_t *JLE8(uint8_t to);
  /* jne rel8 */
  uint8_t *JNE8(uint8_t to);
  /* jnz rel8 */
  uint8_t *JNZ8(uint8_t to);
  /* jng rel8 */
  uint8_t *JNG8(uint8_t to);
  /* jnge rel8 */
  uint8_t *JNGE8(uint8_t to);
  /* jnl rel8 */
  uint8_t *JNL8(uint8_t to);
  /* jnle rel8 */
  uint8_t *JNLE8(uint8_t to);
  /* jo rel8 */
  uint8_t *JO8(uint8_t to);
  /* jno rel8 */
  uint8_t *JNO8(uint8_t to);

  /* je rel32 */
  uint32_t *JE32(uint32_t to);
  /* jz rel32 */
  uint32_t *JZ32(uint32_t to);
  /* jg rel32 */
  uint32_t *JG32(uint32_t to);
  /* jge rel32 */
  uint32_t *JGE32(uint32_t to);
  /* jl rel32 */
  uint32_t *JL32(uint32_t to);
  /* jle rel32 */
  uint32_t *JLE32(uint32_t to);
  /* jne rel32 */
  uint32_t *JNE32(uint32_t to);
  /* jnz rel32 */
  uint32_t *JNZ32(uint32_t to);
  /* jng rel32 */
  uint32_t *JNG32(uint32_t to);
  /* jnge rel32 */
  uint32_t *JNGE32(uint32_t to);
  /* jnl rel32 */
  uint32_t *JNL32(uint32_t to);
  /* jnle rel32 */
  uint32_t *JNLE32(uint32_t to);
  /* jo rel32 */
  uint32_t *JO32(uint32_t to);
  /* jno rel32 */
  uint32_t *JNO32(uint32_t to);

  /* call func */
  void CALLFunc(uint32_t func); // based on CALL32
  /* call rel32 */
  void CALL32(uint32_t to);
  /* call r32 */
  void CALL32R(int32_t to);
  /* call m32 */
  void CALL32M(uint32_t to);

  // misc instructions

  /* cmp imm32 to r32 */
  void CMP32ItoR(int32_t to, uint32_t from);
  /* cmp imm32 to m32 */
  void CMP32ItoM(uint32_t to, uint32_t from);
  /* cmp r32 to r32 */
  void CMP32RtoR(int32_t to, int32_t from);
  /* cmp m32 to r32 */
  void CMP32MtoR(int32_t to, uint32_t from);

  /* test imm32 to r32 */
  void TEST32ItoR(int32_t to, uint32_t from);
  /* test r32 to r32 */
  void TEST32RtoR(int32_t to, int32_t from);
  /* sets r8 */
  void SETS8R(int32_t to);
  /* setl r8 */
  void SETL8R(int32_t to);
  /* setb r8 */
  void SETB8R(int32_t to);

  /* cbw */
  void CBW();
  /* cwd */
  void CWD();
  /* cdq */
  void CDQ();

  /* push r32 */
  void PUSH32R(int32_t from);
  /* push m32 */
  void PUSH32M(uint32_t from);
  /* push imm32 */
  void PUSH32I(uint32_t from);

  /* pop r32 */
  void POP32R(int32_t from);

  /* pushad */
  void PUSHA32();
  /* popad */
  void POPA32();

  /* ret */
  void RET();

  // FPU instructions

  /* fild m32 to fpu reg stack */
  void FILD32(uint32_t from);
  /* fistp m32 from fpu reg stack */
  void FISTP32(uint32_t from);
  /* fld m32 to fpu reg stack */
  void FLD32(uint32_t from);
  /* fstp m32 from fpu reg stack */
  void FSTP32(uint32_t to);

  /* fldcw fpu control word from m16 */
  void FLDCW(uint32_t from);
  /* fstcw fpu control word to m16 */
  void FNSTCW(uint32_t to);

  /* fadd m32 to fpu reg stack */
  void FADD32(uint32_t from);
  /* fsub m32 to fpu reg stack */
  void FSUB32(uint32_t from);
  /* fmul m32 to fpu reg stack */
  void FMUL32(uint32_t from);
  /* fdiv m32 to fpu reg stack */
  void FDIV32(uint32_t from);
  /* fabs fpu reg stack */
  void FABS();
  /* fsqrt fpu reg stack */
  void FSQRT();
  /* fchs fpu reg stack */
  void FCHS();

  // MMX instructions

  // r64 = mm

  /* movq m64 to r64 */
  void MOVQMtoR(int32_t to, uint32_t from);
  /* movq r64 to m64 */
  void MOVQRtoM(uint32_t to, int32_t from);

  /* pand r64 to r64 */
  void PANDRtoR(int32_t to, int32_t from);
  /* pand m64 to r64 */
  void PANDMtoR(int32_t to, uint32_t from);

  /* pandn r64 to r64 */
  void PANDNRtoR(int32_t to, int32_t from);

  /* pandn r64 to r64 */
  void PANDNMtoR(int32_t to, uint32_t from);

  /* por r64 to r64 */
  void PORRtoR(int32_t to, int32_t from);
  /* por m64 to r64 */
  void PORMtoR(int32_t to, uint32_t from);

  /* pxor r64 to r64 */
  void PXORRtoR(int32_t to, int32_t from);
  /* pxor m64 to r64 */
  void PXORMtoR(int32_t to, uint32_t from);

  /* psllq r64 to r64 */
  void PSLLQRtoR(int32_t to, int32_t from);
  /* psllq m64 to r64 */
  void PSLLQMtoR(int32_t to, uint32_t from);
  /* psllq imm8 to r64 */
  void PSLLQItoR(int32_t to, uint8_t from);

  /* psrlq r64 to r64 */
  void PSRLQRtoR(int32_t to, int32_t from);
  /* psrlq m64 to r64 */
  void PSRLQMtoR(int32_t to, uint32_t from);
  /* psrlq imm8 to r64 */
  void PSRLQItoR(int32_t to, uint8_t from);

  /* paddusb r64 to r64 */
  void PADDUSBRtoR(int32_t to, int32_t from);
  /* paddusb m64 to r64 */
  void PADDUSBMtoR(int32_t to, uint32_t from);
  /* paddusw r64 to r64 */
  void PADDUSWRtoR(int32_t to, int32_t from);
  /* paddusw m64 to r64 */
  void PADDUSWMtoR(int32_t to, uint32_t from);

  /* paddb r64 to r64 */
  void PADDBRtoR(int32_t to, int32_t from);
  /* paddb m64 to r64 */
  void PADDBMtoR(int32_t to, uint32_t from);
  /* paddw r64 to r64 */
  void PADDWRtoR(int32_t to, int32_t from);
  /* paddw m64 to r64 */
  void PADDWMtoR(int32_t to, uint32_t from);
  /* paddd r64 to r64 */
  void PADDDRtoR(int32_t to, int32_t from);
  /* paddd m64 to r64 */
  void PADDDMtoR(int32_t to, uint32_t from);

  /* emms */
  void EMMS();
  void FEMMS();
  void BT32ItoR(int32_t to, int32_t from);
  void RCR32ItoR(int32_t to, int32_t from);

  // Basara:changed
  void PADDSBRtoR(int32_t to, int32_t from);
  void PADDSWRtoR(int32_t to, int32_t from);
  void PADDSDRtoR(int32_t to, int32_t from);
  void PSUBSBRtoR(int32_t to, int32_t from);
  void PSUBSWRtoR(int32_t to, int32_t from);
  void PSUBSDRtoR(int32_t to, int32_t from);

  void PSUBBRtoR(int32_t to, int32_t from);
  void PSUBWRtoR(int32_t to, int32_t from);
  void PSUBDRtoR(int32_t to, int32_t from);

  // Prototype
  // Todo add all consts to end of block.not after jr $+8
  void MOVQ64ItoR(int32_t reg, uint64_t i);

  void PSUBUSBRtoR(int32_t to, int32_t from);
  void PSUBUSWRtoR(int32_t to, int32_t from);

  void PMAXSWRtoR(int32_t to, int32_t from);
  void PMINSWRtoR(int32_t to, int32_t from);

  void PCMPEQBRtoR(int32_t to, int32_t from);
  void PCMPEQWRtoR(int32_t to, int32_t from);
  void PCMPEQDRtoR(int32_t to, int32_t from);

  void PCMPGTBRtoR(int32_t to, int32_t from);
  void PCMPGTWRtoR(int32_t to, int32_t from);
  void PCMPGTDRtoR(int32_t to, int32_t from);

  void PSRLWItoR(int32_t to, int32_t from);
  void PSRLDItoR(int32_t to, int32_t from);
  void PSLLWItoR(int32_t to, int32_t from);
  void PSLLDItoR(int32_t to, int32_t from);
  void PSRAWItoR(int32_t to, int32_t from);
  void PSRADItoR(int32_t to, int32_t from);

  // Added:basara 11.01.2003
  void FCOMP32(uint32_t from);
  void FNSTSWtoAX();
  void SETNZ8R(int32_t to);

  // Added:basara 14.01.2003
  void PFCMPEQMtoR(int32_t to, int32_t from);
  void PFCMPGTMtoR(int32_t to, int32_t from);
  void PFCMPGEMtoR(int32_t to, int32_t from);

  void PFADDMtoR(int32_t to, int32_t from);
  void PFADDRtoR(int32_t to, int32_t from);

  void PFSUBMtoR(int32_t to, int32_t from);
  void PFSUBRtoR(int32_t to, int32_t from);

  void PFMULMtoR(int32_t to, int32_t from);
  void PFMULRtoR(int32_t to, int32_t from);

  void PFRCPMtoR(int32_t to, int32_t from);
  void PFRCPRtoR(int32_t to, int32_t from);
  void PFRCPIT1RtoR(int32_t to, int32_t from);
  void PFRCPIT2RtoR(int32_t to, int32_t from);

  void PFRSQRTRtoR(int32_t to, int32_t from);
  void PFRSQIT1RtoR(int32_t to, int32_t from);

  void PF2IDMtoR(int32_t to, int32_t from);
  void PF2IDRtoR(int32_t to, int32_t from);
  void PI2FDMtoR(int32_t to, int32_t from);
  void PI2FDRtoR(int32_t to, int32_t from);

  void PFMAXMtoR(int32_t to, int32_t from);
  void PFMAXRtoR(int32_t to, int32_t from);
  void PFMINMtoR(int32_t to, int32_t from);
  void PFMINRtoR(int32_t to, int32_t from);

  void MOVDMtoR(int32_t to, uint32_t from);
  void MOVDRtoM(uint32_t to, int32_t from);
  void MOVD32RtoR(int32_t to, int32_t from);
  void MOVD64RtoR(int32_t to, int32_t from);

  void MOVQRtoR(int32_t to, int32_t from);

  // if to==from MMLO=MMHI
  void PUNPCKHDQRtoR(int32_t to, int32_t from);

  // if to==from MMHI=MMLO
  void PUNPCKLDQRtoR(int32_t to, int32_t from);

  // SSE intructions

  void MOVAPSMtoR(int32_t to, int32_t from);
  void MOVAPSRtoM(int32_t to, int32_t from);
  void MOVAPSRtoR(int32_t to, int32_t from);

  void ORPSMtoR(int32_t to, int32_t from);
  void ORPSRtoR(int32_t to, int32_t from);

  void XORPSMtoR(int32_t to, int32_t from);
  void XORPSRtoR(int32_t to, int32_t from);

  void ANDPSMtoR(int32_t to, int32_t from);
  void ANDPSRtoR(int32_t to, int32_t from);

protected:
  int8_t *x86Ptr;
  const int8_t *endPtr;

}; // struct ix86
