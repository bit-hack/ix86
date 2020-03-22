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
 * ix86 core v0.5.1
 *  Authors: linuzappz <linuzappz@pcsx.net>
 *           alexey silinov
 */

#if defined(__i386__) || !defined(_WIN64)

#include <cassert>

#include "ix86.h"

enum {
  SIB = 4,
  DISP32 = 5,
};

void ix86::write8(const uint8_t val) {
  *(uint8_t *)x86Ptr = val;
  x86Ptr += 1;
}

void ix86::write16(const uint16_t val) {
  *(uint16_t *)x86Ptr = val;
  x86Ptr += 2;
}

void ix86::write32(const uint32_t val) {
  *(uint32_t *)x86Ptr = val;
  x86Ptr += 4;
}

void ix86::write32(void *val) {
  *(uint32_t *)x86Ptr = uint32_t(val);
  x86Ptr += 4;
}

void ix86::write64(const uint64_t val) {
  *(uint64_t *)x86Ptr = val;
  x86Ptr += 8;
}

void ix86::setJ8(uint8_t *j8) {
  uint32_t jump = (x86Ptr - (int8_t *)j8) - 1;
  assert(jump <= 0x7f);
  *j8 = (uint8_t)jump;
}

void ix86::setJ32(uint32_t *j32) {
  *j32 = (x86Ptr - (int8_t *)j32) - 4;
}

void ix86::align(int32_t bytes) {
  x86Ptr = (int8_t *)(((uintptr_t)x86Ptr + bytes) & ~(bytes - 1));
}

void ix86::ModRM(int32_t mod, int32_t rm, int32_t reg) {
  write8((mod << 6) | (rm << 3) | (reg));
}

void ix86::SibSB(int32_t ss, int32_t rm, int32_t index) {
  write8((ss << 6) | (rm << 3) | (index));
}

// mov instructions

void ix86::MOV32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x89);
  ModRM(3, from, to);
}

void ix86::MOV32RtoM(void *to, gp_reg_t from) {
  write8(0x89);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV32MtoR(gp_reg_t to, void *from) {
  write8(0x8B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV32RmtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x8B);
  ModRM(0, to, from);
}

void ix86::MOV32RmStoR(gp_reg_t to, gp_reg_t from, gp_reg_t from2,
                       scale_t scale) {
  write8(0x8B);
  ModRM(0, to, 0x4);
  SibSB(scale, from2, from);
}

void ix86::MOV32RtoRm(gp_reg_t to, gp_reg_t from) {
  write8(0x89);
  ModRM(0, from, to);
}

void ix86::MOV32RtoRmS(gp_reg_t to, gp_reg_t to2, scale_t scale,
                       gp_reg_t from) {
  write8(0x89);
  ModRM(0, from, 0x4);
  SibSB(scale, to2, to);
}

void ix86::MOV32ItoR(gp_reg_t to, uint32_t from) {
  write8(0xB8 | to);
  write32(from);
}

void ix86::MOV32ItoM(void *to, uint32_t from) {
  write8(0xC7);
  ModRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void ix86::MOV16RtoM(void *to, gp_reg_t from) {
  write8(0x66);
  write8(0x89);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV16MtoR(gp_reg_t to, void *from) {
  write8(0x66);
  write8(0x8B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV16ItoM(void *to, uint16_t from) {
  write8(0x66);
  write8(0xC7);
  ModRM(0, 0, DISP32);
  write32(to);
  write16(from);
}

void ix86::MOV8RtoM(void *to, gp_reg_t from) {
  write8(0x88);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV8MtoR(gp_reg_t to, void *from) {
  write8(0x8A);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV8ItoM(void *to, uint8_t from) {
  write8(0xC6);
  ModRM(0, 0, DISP32);
  write32(to);
  write8(from);
}

// mov sign extend

void ix86::MOVSX32R8toR(gp_reg_t to, gp_reg_t from) {
  write16(0xBE0F);
  ModRM(3, to, from);
}

void ix86::MOVSX32M8toR(gp_reg_t to, void *from) {
  write16(0xBE0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVSX32R16toR(gp_reg_t to, gp_reg_t from) {
  write16(0xBF0F);
  ModRM(3, to, from);
}

void ix86::MOVSX32M16toR(gp_reg_t to, void *from) {
  write16(0xBF0F);
  ModRM(0, to, DISP32);
  write32(from);
}

// mov zero extend

void ix86::MOVZX32R8toR(gp_reg_t to, gp_reg_t from) {
  write16(0xB60F);
  ModRM(3, to, from);
}

void ix86::MOVZX32M8toR(gp_reg_t to, void *from) {
  write16(0xB60F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVZX32R16toR(gp_reg_t to, gp_reg_t from) {
  write16(0xB70F);
  ModRM(3, to, from);
}

void ix86::MOVZX32M16toR(gp_reg_t to, void *from) {
  write16(0xB70F);
  ModRM(0, to, DISP32);
  write32(from);
}

// CMOV

void ix86::CMOV32RtoR(cmov_cc_t cc, gp_reg_t to, gp_reg_t from) {
  write8(0x0F);
  write8(cc);
  ModRM(3, to, from);
}

void ix86::CMOV32MtoR(cmov_cc_t cc, gp_reg_t to, void *from) {
  write8(0x0F);
  write8(cc);
  ModRM(0, to, DISP32);
  write32(from);
}

// ADD

void ix86::ADD32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x05);
  } else {
    write8(0x81);
    ModRM(3, 0, to);
  }
  write32(from);
}

void ix86::ADD32ItoM(void *to, uint32_t from) {
  write8(0x81);
  ModRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void ix86::ADD32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x01);
  ModRM(3, from, to);
}

void ix86::ADD32RtoM(void *to, gp_reg_t from) {
  write8(0x01);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::ADD32MtoR(gp_reg_t to, void *from) {
  write8(0x03);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::ADC32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x15);
  } else {
    write8(0x81);
    ModRM(3, 2, to);
  }
  write32(from);
}

void ix86::ADC32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x11);
  ModRM(3, from, to);
}

void ix86::ADC32MtoR(gp_reg_t to, void *from) {
  write8(0x13);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::INC32R(gp_reg_t to) {
  write8(0x40 + to);
}

void ix86::INC32M(void *to) {
  write8(0xFF);
  ModRM(0, 0, DISP32);
  write32(to);
}

void ix86::SUB32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x2D);
  } else {
    write8(0x81);
    ModRM(3, 5, to);
  }
  write32(from);
}

void ix86::SUB32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x29);
  ModRM(3, from, to);
}

void ix86::SUB32MtoR(gp_reg_t to, void *from) {
  write8(0x2B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::SBB32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x1D);
  } else {
    write8(0x81);
    ModRM(3, 3, to);
  }
  write32(from);
}

void ix86::SBB32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x19);
  ModRM(3, from, to);
}

void ix86::SBB32MtoR(gp_reg_t to, void *from) {
  write8(0x1B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::DEC32R(gp_reg_t to) {
  write8(0x48 + to);
}

void ix86::DEC32M(void *to) {
  write8(0xFF);
  ModRM(0, 1, DISP32);
  write32(to);
}

void ix86::MUL32R(gp_reg_t from) {
  write8(0xF7);
  ModRM(3, 4, from);
}

void ix86::MUL32M(void *from) {
  write8(0xF7);
  ModRM(0, 4, DISP32);
  write32(from);
}

void ix86::IMUL32R(gp_reg_t from) {
  write8(0xF7);
  ModRM(3, 5, from);
}

void ix86::IMUL32M(void *from) {
  write8(0xF7);
  ModRM(0, 5, DISP32);
  write32(from);
}

void ix86::IMUL32RtoR(gp_reg_t to, gp_reg_t from) {
  write16(0xAF0F);
  ModRM(3, to, from);
}

void ix86::DIV32R(gp_reg_t from) {
  write8(0xF7);
  ModRM(3, 6, from);
}

void ix86::IDIV32R(gp_reg_t from) {
  write8(0xF7);
  ModRM(3, 7, from);
}

void ix86::DIV32M(void *from) {
  write8(0xF7);
  ModRM(0, 6, DISP32);
  write32(from);
}

void ix86::IDIV32M(void *from) {
  write8(0xF7);
  ModRM(0, 7, DISP32);
  write32(from);
}

// shifting

void ix86::RCR32ItoR(int32_t to, int32_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xd8 | to);
  } else {
    write8(0xc1);
    write8(0xd8 | to);
    write8(from);
  }
}

void ix86::SHL32ItoR(gp_reg_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe0 | to);
    return;
  }
  write8(0xC1);
  ModRM(3, 4, to);
  write8(from);
}

void ix86::SHL32CLtoR(gp_reg_t to) {
  write8(0xD3);
  ModRM(3, 4, to);
}

void ix86::SHR32ItoR(gp_reg_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe8 | to);
    return;
  }
  write8(0xC1);
  ModRM(3, 5, to);
  write8(from);
}

void ix86::SHR32CLtoR(gp_reg_t to) {
  write8(0xD3);
  ModRM(3, 5, to);
}

void ix86::SAR32ItoR(gp_reg_t to, uint8_t from) {
  write8(0xC1);
  ModRM(3, 7, to);
  write8(from);
}

void ix86::SAR32CLtoR(gp_reg_t to) {
  write8(0xD3);
  ModRM(3, 7, to);
}

void ix86::OR32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x0D);
  } else {
    write8(0x81);
    ModRM(3, 1, to);
  }
  write32(from);
}

void ix86::OR32ItoM(void *to, uint32_t from) {
  write8(0x81);
  ModRM(0, 1, DISP32);
  write32(to);
  write32(from);
}

void ix86::OR32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x09);
  ModRM(3, from, to);
}

void ix86::OR32RtoM(void *to, gp_reg_t from) {
  write8(0x09);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::OR32MtoR(gp_reg_t to, void *from) {
  write8(0x0B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::XOR32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x35);
  } else {
    write8(0x81);
    ModRM(3, 6, to);
  }
  write32(from);
}

void ix86::XOR32ItoM(void *to, uint32_t from) {
  write8(0x81);
  ModRM(0, 6, DISP32);
  write32(to);
  write32(from);
}

void ix86::XOR32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x31);
  ModRM(3, from, to);
}

void ix86::XOR32RtoM(void *to, gp_reg_t from) {
  write8(0x31);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::XOR32MtoR(gp_reg_t to, void *from) {
  write8(0x33);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::AND32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x25);
  } else {
    write8(0x81);
    ModRM(3, 0x4, to);
  }
  write32(from);
}

void ix86::AND32ItoM(void *to, uint32_t from) {
  write8(0x81);
  ModRM(0, 0x4, DISP32);
  write32(to);
  write32(from);
}

void ix86::AND32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x21);
  ModRM(3, from, to);
}

void ix86::AND32RtoM(void *to, gp_reg_t from) {
  write8(0x21);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::AND32MtoR(gp_reg_t to, void *from) {
  write8(0x23);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::NOT32R(gp_reg_t from) {
  write8(0xF7);
  ModRM(3, 2, from);
}

void ix86::NEG32R(gp_reg_t from) {
  write8(0xF7);
  ModRM(3, 3, from);
}

// jump instructions

uint8_t *ix86::J8Rel(int32_t cc, int32_t to) {
  write8(cc);
  write8(to);
  return (uint8_t *)(x86Ptr - 1);
}

uint32_t *ix86::J32Rel(int32_t cc, int32_t to) {
  write8(0x0F);
  write8(cc);
  write32(to);
  return (uint32_t *)(x86Ptr - 4);
}

uint8_t *ix86::JMP8(uint8_t to) {
  write8(0xEB);
  write8(to);
  return (uint8_t *)(x86Ptr - 1);
}

uint32_t *ix86::JMP32(uint32_t to) {
  write8(0xE9);
  write32(to);
  return (uint32_t *)(x86Ptr - 4);
}

void ix86::JMP32R(int32_t to) {
  write8(0xFF);
  ModRM(3, 4, to);
}

uint8_t *ix86::JE8(uint8_t to) {
  return J8Rel(0x74, to);
}

uint8_t *ix86::JZ8(uint8_t to) {
  return J8Rel(0x74, to);
}

uint8_t *ix86::JG8(uint8_t to) {
  return J8Rel(0x7F, to);
}

uint8_t *ix86::JGE8(uint8_t to) {
  return J8Rel(0x7D, to);
}

uint8_t *ix86::JL8(uint8_t to) {
  return J8Rel(0x7C, to);
}

uint8_t *ix86::JLE8(uint8_t to) {
  return J8Rel(0x7E, to);
}

uint8_t *ix86::JNE8(uint8_t to) {
  return J8Rel(0x75, to);
}

uint8_t *ix86::JNZ8(uint8_t to) {
  return J8Rel(0x75, to);
}

uint8_t *ix86::JNG8(uint8_t to) {
  return J8Rel(0x7E, to);
}

uint8_t *ix86::JNGE8(uint8_t to) {
  return J8Rel(0x7C, to);
}

uint8_t *ix86::JNL8(uint8_t to) {
  return J8Rel(0x7D, to);
}

uint8_t *ix86::JNLE8(uint8_t to) {
  return J8Rel(0x7F, to);
}

uint8_t *ix86::JO8(uint8_t to) {
  return J8Rel(0x70, to);
}

uint8_t *ix86::JNO8(uint8_t to) {
  return J8Rel(0x71, to);
}

uint32_t *ix86::JE32(uint32_t to) {
  return J32Rel(0x84, to);
}

uint32_t *ix86::JZ32(uint32_t to) {
  return J32Rel(0x84, to);
}

uint32_t *ix86::JG32(uint32_t to) {
  return J32Rel(0x8F, to);
}

uint32_t *ix86::JGE32(uint32_t to) {
  return J32Rel(0x8D, to);
}

uint32_t *ix86::JL32(uint32_t to) {
  return J32Rel(0x8C, to);
}

uint32_t *ix86::JLE32(uint32_t to) {
  return J32Rel(0x8E, to);
}

uint32_t *ix86::JNE32(uint32_t to) {
  return J32Rel(0x85, to);
}

uint32_t *ix86::JNZ32(uint32_t to) {
  return J32Rel(0x85, to);
}

uint32_t *ix86::JNG32(uint32_t to) {
  return J32Rel(0x8E, to);
}

uint32_t *ix86::JNGE32(uint32_t to) {
  return J32Rel(0x8C, to);
}

uint32_t *ix86::JNL32(uint32_t to) {
  return J32Rel(0x8D, to);
}

uint32_t *ix86::JNLE32(uint32_t to) {
  return J32Rel(0x8F, to);
}

uint32_t *ix86::JO32(uint32_t to) {
  return J32Rel(0x80, to);
}

uint32_t *ix86::JNO32(uint32_t to) {
  return J32Rel(0x81, to);
}

void ix86::CALLFunc(uint32_t func) {
  uint32_t to = func - ((uint32_t)x86Ptr + 5);
  write8(0xE8);
  write32(to);
}

void ix86::CALL32(uint32_t to) {
  write8(0xE8);
  write32(to);
}

void ix86::CALL32R(gp_reg_t to) {
  write8(0xFF);
  ModRM(3, 2, to);
}

void ix86::CALL32M(void *to) {
  write8(0xFF);
  ModRM(0, 2, DISP32);
  write32(to);
}

void ix86::BT32ItoR(gp_reg_t to, int32_t from) {
  write16(0xba0f);
  write8(0xe0 | to);
  write8(from);
}

void ix86::CMP32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x3D);
  } else {
    write8(0x81);
    ModRM(3, 7, to);
  }
  write32(from);
}

void ix86::CMP32ItoM(void *to, uint32_t from) {
  write8(0x81);
  ModRM(0, 7, DISP32);
  write32(to);
  write32(from);
}

void ix86::CMP32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x39);
  ModRM(3, from, to);
}

void ix86::CMP32MtoR(gp_reg_t to, void *from) {
  write8(0x3B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::TEST32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0xA9);
  } else {
    write8(0xF7);
    ModRM(3, 0, to);
  }
  write32(from);
}

void ix86::TEST32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x85);
  ModRM(3, from, to);
}

void ix86::SET8R(int32_t cc, gp_reg_t to) {
  write8(0x0F);
  write8(cc);
  write8(0xC0 | to);
}

void ix86::SETS8R(gp_reg_t to) {
  SET8R(0x98, to);
}

void ix86::SETL8R(gp_reg_t to) {
  SET8R(0x9C, to);
}

void ix86::SETB8R(gp_reg_t to) {
  SET8R(0x92, to);
}

void ix86::SETNZ8R(gp_reg_t to) {
  SET8R(0x95, to);
}

void ix86::CBW() {
  write16(0x9866);
}

void ix86::CWD() {
  write8(0x98);
}

void ix86::CDQ() {
  write8(0x99);
}

void ix86::PUSH32R(gp_reg_t from) {
  write8(0x50 | from);
}

void ix86::PUSH32M(void *from) {
  write8(0xFF);
  ModRM(0, 6, DISP32);
  write32(from);
}

void ix86::PUSH32I(uint32_t from) {
  write8(0x68);
  write32(from);
}

void ix86::POP32R(gp_reg_t from) {
  write8(0x58 | from);
}

void ix86::PUSHA32() {
  write8(0x60);
}

void ix86::POPA32() {
  write8(0x61);
}

void ix86::RET() {
  write8(0xC3);
}

#endif // #if defined(__i386__) || !defined(_WIN64)
