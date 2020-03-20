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

void ix86::write64(const uint64_t val) {
  *(uint64_t *)x86Ptr = val;
  x86Ptr += 8;
}

void ix86::x86SetJ8(uint8_t *j8) {
  uint32_t jump = (x86Ptr - (int8_t *)j8) - 1;
  assert(jump <= 0x7f);
  *j8 = (uint8_t)jump;
}

void ix86::x86SetJ32(uint32_t *j32) {
  *j32 = (x86Ptr - (int8_t *)j32) - 4;
}

void ix86::x86Align(int32_t bytes) {
  // fordward align
  x86Ptr = (int8_t *)(((uint32_t)x86Ptr + bytes) & ~(bytes - 1));
}

#define SIB 4
#define DISP32 5

void ix86::ModRM(int32_t mod, int32_t rm, int32_t reg) {
  write8((mod << 6) | (rm << 3) | (reg));
}

void ix86::SibSB(int32_t ss, int32_t rm, int32_t index) {
  write8((ss << 6) | (rm << 3) | (index));
}

void ix86::SET8R(int32_t cc, int32_t to) {
  write8(0x0F);
  write8(cc);
  write8((0xC0) | (to));
}

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

void ix86::CMOV32RtoR(int32_t cc, int32_t to, int32_t from) {
  write8(0x0F);
  write8(cc);
  ModRM(3, to, from);
}

void ix86::CMOV32MtoR(int32_t cc, int32_t to, int32_t from) {
  write8(0x0F);
  write8(cc);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV32RtoR(int32_t to, int32_t from) {
  write8(0x89);
  ModRM(3, from, to);
}

void ix86::MOV32RtoM(uint32_t to, int32_t from) {
  write8(0x89);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV32MtoR(int32_t to, uint32_t from) {
  write8(0x8B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV32RmtoR(int32_t to, int32_t from) {
  write8(0x8B);
  ModRM(0, to, from);
}

void ix86::MOV32RmStoR(int32_t to, int32_t from, int32_t from2, int32_t scale) {
  write8(0x8B);
  ModRM(0, to, 0x4);
  SibSB(scale, from2, from);
}

void ix86::MOV32RtoRm(int32_t to, int32_t from) {
  write8(0x89);
  ModRM(0, from, to);
}

void ix86::MOV32RtoRmS(int32_t to, int32_t to2, int32_t scale, int32_t from) {
  write8(0x89);
  ModRM(0, from, 0x4);
  SibSB(scale, to2, to);
}

void ix86::MOV32ItoR(int32_t to, uint32_t from) {
  write8(0xB8 | to);
  write32(from);
}

void ix86::MOV32ItoM(uint32_t to, uint32_t from) {
  write8(0xC7);
  ModRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void ix86::MOV16RtoM(uint32_t to, int32_t from) {
  write8(0x66);
  write8(0x89);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV16MtoR(int32_t to, uint32_t from) {
  write8(0x66);
  write8(0x8B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV16ItoM(uint32_t to, uint16_t from) {
  write8(0x66);
  write8(0xC7);
  ModRM(0, 0, DISP32);
  write32(to);
  write16(from);
}

void ix86::MOV8RtoM(uint32_t to, int32_t from) {
  write8(0x88);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV8MtoR(int32_t to, uint32_t from) {
  write8(0x8A);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV8ItoM(uint32_t to, uint8_t from) {
  write8(0xC6);
  ModRM(0, 0, DISP32);
  write32(to);
  write8(from);
}

void ix86::MOVSX32R8toR(int32_t to, int32_t from) {
  write16(0xBE0F);
  ModRM(3, to, from);
}

void ix86::MOVSX32M8toR(int32_t to, uint32_t from) {
  write16(0xBE0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVSX32R16toR(int32_t to, int32_t from) {
  write16(0xBF0F);
  ModRM(3, to, from);
}

void ix86::MOVSX32M16toR(int32_t to, uint32_t from) {
  write16(0xBF0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVZX32R8toR(int32_t to, int32_t from) {
  write16(0xB60F);
  ModRM(3, to, from);
}

void ix86::MOVZX32M8toR(int32_t to, uint32_t from) {
  write16(0xB60F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVZX32R16toR(int32_t to, int32_t from) {
  write16(0xB70F);
  ModRM(3, to, from);
}

void ix86::MOVZX32M16toR(int32_t to, uint32_t from) {
  write16(0xB70F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::CMOVNE32RtoR(int32_t to, int32_t from) {
  CMOV32RtoR(0x45, to, from);
}

void ix86::CMOVNE32MtoR(int32_t to, uint32_t from) {
  CMOV32MtoR(0x45, to, from);
}

void ix86::CMOVE32RtoR(int32_t to, int32_t from) {
  CMOV32RtoR(0x44, to, from);
}

void ix86::CMOVE32MtoR(int32_t to, uint32_t from) {
  CMOV32MtoR(0x44, to, from);
}

void ix86::CMOVG32RtoR(int32_t to, int32_t from) {
  CMOV32RtoR(0x4F, to, from);
}

void ix86::CMOVG32MtoR(int32_t to, uint32_t from) {
  CMOV32MtoR(0x4F, to, from);
}

void ix86::CMOVGE32RtoR(int32_t to, int32_t from) {
  CMOV32RtoR(0x4D, to, from);
}

void ix86::CMOVGE32MtoR(int32_t to, uint32_t from) {
  CMOV32MtoR(0x4D, to, from);
}

void ix86::CMOVL32RtoR(int32_t to, int32_t from) {
  CMOV32RtoR(0x4C, to, from);
}

void ix86::CMOVL32MtoR(int32_t to, uint32_t from) {
  CMOV32MtoR(0x4C, to, from);
}

void ix86::CMOVLE32RtoR(int32_t to, int32_t from) {
  CMOV32RtoR(0x4E, to, from);
}

void ix86::CMOVLE32MtoR(int32_t to, uint32_t from) {
  CMOV32MtoR(0x4E, to, from);
}

void ix86::ADD32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x05);
  } else {
    write8(0x81);
    ModRM(3, 0, to);
  }
  write32(from);
}

void ix86::ADD32ItoM(uint32_t to, uint32_t from) {
  write8(0x81);
  ModRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void ix86::ADD32RtoR(int32_t to, int32_t from) {
  write8(0x01);
  ModRM(3, from, to);
}

void ix86::ADD32RtoM(uint32_t to, int32_t from) {
  write8(0x01);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::ADD32MtoR(int32_t to, uint32_t from) {
  write8(0x03);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::ADC32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x15);
  } else {
    write8(0x81);
    ModRM(3, 2, to);
  }
  write32(from);
}

void ix86::ADC32RtoR(int32_t to, int32_t from) {
  write8(0x11);
  ModRM(3, from, to);
}

void ix86::ADC32MtoR(int32_t to, uint32_t from) {
  write8(0x13);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::INC32R(int32_t to) {
  write8(0x40 + to);
}

void ix86::INC32M(uint32_t to) {
  write8(0xFF);
  ModRM(0, 0, DISP32);
  write32(to);
}

void ix86::SUB32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x2D);
  } else {
    write8(0x81);
    ModRM(3, 5, to);
  }
  write32(from);
}

void ix86::SUB32RtoR(int32_t to, int32_t from) {
  write8(0x29);
  ModRM(3, from, to);
}

void ix86::SUB32MtoR(int32_t to, uint32_t from) {
  write8(0x2B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::SBB32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x1D);
  } else {
    write8(0x81);
    ModRM(3, 3, to);
  }
  write32(from);
}

void ix86::SBB32RtoR(int32_t to, int32_t from) {
  write8(0x19);
  ModRM(3, from, to);
}

void ix86::SBB32MtoR(int32_t to, uint32_t from) {
  write8(0x1B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::DEC32R(int32_t to) {
  write8(0x48 + to);
}

void ix86::DEC32M(uint32_t to) {
  write8(0xFF);
  ModRM(0, 1, DISP32);
  write32(to);
}

void ix86::MUL32R(int32_t from) {
  write8(0xF7);
  ModRM(3, 4, from);
}

void ix86::IMUL32R(int32_t from) {
  write8(0xF7);
  ModRM(3, 5, from);
}

void ix86::MUL32M(uint32_t from) {
  write8(0xF7);
  ModRM(0, 4, DISP32);
  write32(from);
}

void ix86::IMUL32M(uint32_t from) {
  write8(0xF7);
  ModRM(0, 5, DISP32);
  write32(from);
}

void ix86::IMUL32RtoR(int32_t to, int32_t from) {
  write16(0xAF0F);
  ModRM(3, to, from);
}

void ix86::DIV32R(int32_t from) {
  write8(0xF7);
  ModRM(3, 6, from);
}

void ix86::IDIV32R(int32_t from) {
  write8(0xF7);
  ModRM(3, 7, from);
}

void ix86::DIV32M(uint32_t from) {
  write8(0xF7);
  ModRM(0, 6, DISP32);
  write32(from);
}

void ix86::IDIV32M(uint32_t from) {
  write8(0xF7);
  ModRM(0, 7, DISP32);
  write32(from);
}

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

void ix86::SHL32ItoR(int32_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe0 | to);
    return;
  }
  write8(0xC1);
  ModRM(3, 4, to);
  write8(from);
}

void ix86::SHL32CLtoR(int32_t to) {
  write8(0xD3);
  ModRM(3, 4, to);
}

void ix86::SHR32ItoR(int32_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe8 | to);
    return;
  }
  write8(0xC1);
  ModRM(3, 5, to);
  write8(from);
}

void ix86::SHR32CLtoR(int32_t to) {
  write8(0xD3);
  ModRM(3, 5, to);
}

void ix86::SAR32ItoR(int32_t to, uint8_t from) {
  write8(0xC1);
  ModRM(3, 7, to);
  write8(from);
}

void ix86::SAR32CLtoR(int32_t to) {
  write8(0xD3);
  ModRM(3, 7, to);
}

void ix86::OR32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x0D);
  } else {
    write8(0x81);
    ModRM(3, 1, to);
  }
  write32(from);
}

void ix86::OR32ItoM(uint32_t to, uint32_t from) {
  write8(0x81);
  ModRM(0, 1, DISP32);
  write32(to);
  write32(from);
}

void ix86::OR32RtoR(int32_t to, int32_t from) {
  write8(0x09);
  ModRM(3, from, to);
}

void ix86::OR32RtoM(uint32_t to, int32_t from) {
  write8(0x09);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::OR32MtoR(int32_t to, uint32_t from) {
  write8(0x0B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::XOR32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x35);
  } else {
    write8(0x81);
    ModRM(3, 6, to);
  }
  write32(from);
}

void ix86::XOR32ItoM(uint32_t to, uint32_t from) {
  write8(0x81);
  ModRM(0, 6, DISP32);
  write32(to);
  write32(from);
}

void ix86::XOR32RtoR(int32_t to, int32_t from) {
  write8(0x31);
  ModRM(3, from, to);
}

void ix86::XOR32RtoM(uint32_t to, int32_t from) {
  write8(0x31);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::XOR32MtoR(int32_t to, uint32_t from) {
  write8(0x33);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::AND32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x25);
  } else {
    write8(0x81);
    ModRM(3, 0x4, to);
  }
  write32(from);
}

void ix86::AND32ItoM(uint32_t to, uint32_t from) {
  write8(0x81);
  ModRM(0, 0x4, DISP32);
  write32(to);
  write32(from);
}

void ix86::AND32RtoR(int32_t to, int32_t from) {
  write8(0x21);
  ModRM(3, from, to);
}

void ix86::AND32RtoM(uint32_t to, int32_t from) {
  write8(0x21);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::AND32MtoR(int32_t to, uint32_t from) {
  write8(0x23);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::NOT32R(int32_t from) {
  write8(0xF7);
  ModRM(3, 2, from);
}

void ix86::NEG32R(int32_t from) {
  write8(0xF7);
  ModRM(3, 3, from);
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
  CALL32(func - ((uint32_t)x86Ptr + 5));
}

void ix86::CALL32(uint32_t to) {
  write8(0xE8);
  write32(to);
}

void ix86::CALL32R(int32_t to) {
  write8(0xFF);
  ModRM(3, 2, to);
}

void ix86::CALL32M(uint32_t to) {
  write8(0xFF);
  ModRM(0, 2, DISP32);
  write32(to);
}

void ix86::CMP32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x3D);
  } else {
    write8(0x81);
    ModRM(3, 7, to);
  }
  write32(from);
}

void ix86::CMP32ItoM(uint32_t to, uint32_t from) {
  write8(0x81);
  ModRM(0, 7, DISP32);
  write32(to);
  write32(from);
}

void ix86::CMP32RtoR(int32_t to, int32_t from) {
  write8(0x39);
  ModRM(3, from, to);
}

void ix86::CMP32MtoR(int32_t to, uint32_t from) {
  write8(0x3B);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::TEST32ItoR(int32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0xA9);
  } else {
    write8(0xF7);
    ModRM(3, 0, to);
  }
  write32(from);
}

void ix86::TEST32RtoR(int32_t to, int32_t from) {
  write8(0x85);
  ModRM(3, from, to);
}

void ix86::BT32ItoR(int32_t to, int32_t from) {
  write16(0xba0f);
  write8(0xe0 | to);
  write8(from);
}

void ix86::SETS8R(int32_t to) {
  SET8R(0x98, to);
}

void ix86::SETL8R(int32_t to) {
  SET8R(0x9C, to);
}

void ix86::SETB8R(int32_t to) {
  SET8R(0x92, to);
}

void ix86::SETNZ8R(int32_t to) {
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

void ix86::PUSH32R(int32_t from) {
  write8(0x50 | from);
}

void ix86::PUSH32M(uint32_t from) {
  write8(0xFF);
  ModRM(0, 6, DISP32);
  write32(from);
}

void ix86::PUSH32I(uint32_t from) {
  write8(0x68);
  write32(from);
}

void ix86::POP32R(int32_t from) {
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

void ix86::FCOMP32(uint32_t from) {
  write8(0xD8);
  ModRM(0, 0x3, DISP32);
  write32(from);
}

void ix86::FNSTSWtoAX() {
  write16(0xE0DF);
}

void ix86::FILD32(uint32_t from) {
  write8(0xDB);
  ModRM(0, 0x0, DISP32);
  write32(from);
}

void ix86::FISTP32(uint32_t from) {
  write8(0xDB);
  ModRM(0, 0x3, DISP32);
  write32(from);
}

void ix86::FLD32(uint32_t from) {
  write8(0xD9);
  ModRM(0, 0x0, DISP32);
  write32(from);
}

void ix86::FSTP32(uint32_t to) {
  write8(0xD9);
  ModRM(0, 0x3, DISP32);
  write32(to);
}

//

void ix86::FLDCW(uint32_t from) {
  write8(0xD9);
  ModRM(0, 0x5, DISP32);
  write32(from);
}

void ix86::FNSTCW(uint32_t to) {
  write8(0xD9);
  ModRM(0, 0x7, DISP32);
  write32(to);
}

//

void ix86::FADD32(uint32_t from) {
  write8(0xD8);
  ModRM(0, 0x0, DISP32);
  write32(from);
}

void ix86::FSUB32(uint32_t from) {
  write8(0xD8);
  ModRM(0, 0x4, DISP32);
  write32(from);
}

void ix86::FMUL32(uint32_t from) {
  write8(0xD8);
  ModRM(0, 0x1, DISP32);
  write32(from);
}

void ix86::FDIV32(uint32_t from) {
  write8(0xD8);
  ModRM(0, 0x6, DISP32);
  write32(from);
}

void ix86::FABS() {
  write16(0xE1D9);
}

void ix86::FSQRT() {
  write16(0xFAD9);
}

void ix86::FCHS() {
  write16(0xE0D9);
}

void ix86::MOVQMtoR(int32_t to, uint32_t from) {
  write16(0x6F0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVQRtoM(uint32_t to, int32_t from) {
  write16(0x7F0F);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::PANDRtoR(int32_t to, int32_t from) {
  write16(0xDB0F);
  ModRM(3, to, from);
}

void ix86::PANDNRtoR(int32_t to, int32_t from) {
  write16(0xDF0F);
  ModRM(3, to, from);
}

void ix86::PORRtoR(int32_t to, int32_t from) {
  write16(0xEB0F);
  ModRM(3, to, from);
}

void ix86::PXORRtoR(int32_t to, int32_t from) {
  write16(0xEF0F);
  ModRM(3, to, from);
}

void ix86::PSLLQRtoR(int32_t to, int32_t from) {
  write16(0xF30F);
  ModRM(3, to, from);
}

void ix86::PSLLQMtoR(int32_t to, uint32_t from) {
  write16(0xF30F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PSLLQItoR(int32_t to, uint8_t from) {
  write16(0x730F);
  ModRM(3, 6, to);
  write8(from);
}

void ix86::PSRLQRtoR(int32_t to, int32_t from) {
  write16(0xD30F);
  ModRM(3, to, from);
}

void ix86::PSRLQMtoR(int32_t to, uint32_t from) {
  write16(0xD30F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PSRLQItoR(int32_t to, uint8_t from) {
  write16(0x730F);
  ModRM(3, 2, to);
  write8(from);
}

void ix86::PADDUSBRtoR(int32_t to, int32_t from) {
  write16(0xDC0F);
  ModRM(3, to, from);
}

void ix86::PADDUSBMtoR(int32_t to, uint32_t from) {
  write16(0xDC0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PADDUSWRtoR(int32_t to, int32_t from) {
  write16(0xDD0F);
  ModRM(3, to, from);
}

void ix86::PADDUSWMtoR(int32_t to, uint32_t from) {
  write16(0xDD0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PADDBRtoR(int32_t to, int32_t from) {
  write16(0xFC0F);
  ModRM(3, to, from);
}

void ix86::PADDBMtoR(int32_t to, uint32_t from) {
  write16(0xFC0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PADDWRtoR(int32_t to, int32_t from) {
  write16(0xFD0F);
  ModRM(3, to, from);
}

void ix86::PADDWMtoR(int32_t to, uint32_t from) {
  write16(0xFD0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PADDDRtoR(int32_t to, int32_t from) {
  write16(0xFE0F);
  ModRM(3, to, from);
}

void ix86::PADDDMtoR(int32_t to, uint32_t from) {
  write16(0xFE0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::EMMS() {
  // use femms if we have 3dnow
  write16(0x0e0f);
  return;
}

void ix86::FEMMS() {
  write16(0x770F);
  return;
}

void ix86::PADDSBRtoR(int32_t to, int32_t from) {
  write16(0xEC0F);
  ModRM(3, to, from);
}

void ix86::PADDSWRtoR(int32_t to, int32_t from) {
  write16(0xED0F);
  ModRM(3, to, from);
}

void ix86::PADDSDRtoR(int32_t to, int32_t from) {
  write16(0xEE0F);
  ModRM(3, to, from);
}

void ix86::PSUBSBRtoR(int32_t to, int32_t from) {
  write16(0xE80F);
  ModRM(3, to, from);
}

void ix86::PSUBSWRtoR(int32_t to, int32_t from) {
  write16(0xE90F);
  ModRM(3, to, from);
}

void ix86::PSUBSDRtoR(int32_t to, int32_t from) {
  write16(0xEA0F);
  ModRM(3, to, from);
}

void ix86::PSUBBRtoR(int32_t to, int32_t from) {
  write16(0xF80F);
  ModRM(3, to, from);
}

void ix86::PSUBWRtoR(int32_t to, int32_t from) {
  write16(0xF90F);
  ModRM(3, to, from);
}

void ix86::PSUBDRtoR(int32_t to, int32_t from) {
  write16(0xFA0F);
  ModRM(3, to, from);
}

void ix86::MOVQ64ItoR(int32_t reg, uint64_t i) {
  MOVQMtoR(reg, (uint32_t)(x86Ptr) + 2 + 7);
  JMP8(8);
  write64(i);
}

void ix86::PSUBUSBRtoR(int32_t to, int32_t from) {
  write16(0xD80F);
  ModRM(3, to, from);
}

void ix86::PSUBUSWRtoR(int32_t to, int32_t from) {
  write16(0xD90F);
  ModRM(3, to, from);
}

void ix86::PMAXSWRtoR(int32_t to, int32_t from) {
  write16(0xEE0F);
  ModRM(3, to, from);
}

void ix86::PMINSWRtoR(int32_t to, int32_t from) {
  write16(0xEA0F);
  ModRM(3, to, from);
}

void ix86::PCMPEQBRtoR(int32_t to, int32_t from) {
  write16(0x740F);
  ModRM(3, to, from);
}

void ix86::PCMPEQWRtoR(int32_t to, int32_t from) {
  write16(0x750F);
  ModRM(3, to, from);
}

void ix86::PCMPEQDRtoR(int32_t to, int32_t from) {
  write16(0x760F);
  ModRM(3, to, from);
}

void ix86::PCMPGTBRtoR(int32_t to, int32_t from) {
  write16(0x640F);
  ModRM(3, to, from);
}

void ix86::PCMPGTWRtoR(int32_t to, int32_t from) {
  write16(0x650F);
  ModRM(3, to, from);
}

void ix86::PCMPGTDRtoR(int32_t to, int32_t from) {
  write16(0x660F);
  ModRM(3, to, from);
}

void ix86::PSRLWItoR(int32_t to, int32_t from) {
  write16(0x710f);
  ModRM(2, 2, to);
  write8(from);
}
void ix86::PSRLDItoR(int32_t to, int32_t from) {
  write16(0x720f);
  ModRM(2, 2, to);
  write8(from);
}

void ix86::PSLLWItoR(int32_t to, int32_t from) {
  write16(0x710f);
  ModRM(3, 6, to);
  write8(from);
}

void ix86::PSLLDItoR(int32_t to, int32_t from) {
  write16(0x720f);
  ModRM(3, 6, to);
  write8(from);
}

void ix86::PSRAWItoR(int32_t to, int32_t from) {
  write16(0x710f);
  ModRM(3, 4, to);
  write8(from);
}

void ix86::PSRADItoR(int32_t to, int32_t from) {
  write16(0x720f);
  ModRM(3, 4, to);
  write8(from);
}

void ix86::PORMtoR(int32_t to, uint32_t from) {
  write16(0xEB0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PXORMtoR(int32_t to, uint32_t from) {
  write16(0xEF0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PANDMtoR(int32_t to, uint32_t from) {
  write16(0xDB0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::PANDNMtoR(int32_t to, uint32_t from) {
  write16(0xDF0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVDMtoR(int32_t to, uint32_t from) {
  write16(0x6E0F);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVDRtoM(uint32_t to, int32_t from) {
  write16(0x7E0F);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOVD32RtoR(int32_t to, int32_t from) {
  write16(0x6E0F);
  ModRM(3, to, from);
}

void ix86::MOVD64RtoR(int32_t to, int32_t from) {
  write16(0x7E0F);
  ModRM(3, from, to);
}

void ix86::MOVQRtoR(int32_t to, int32_t from) {
  write16(0x6F0F);
  ModRM(3, to, from);
}

void ix86::PUNPCKHDQRtoR(int32_t to, int32_t from) {
  write16(0x6A0F);
  ModRM(3, to, from);
}

void ix86::PUNPCKLDQRtoR(int32_t to, int32_t from) {
  write16(0x620F);
  ModRM(3, to, from);
}

void ix86::MOVAPSMtoR(int32_t to, int32_t from) {
  write16(0x280f);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVAPSRtoM(int32_t to, int32_t from) {
  write16(0x2b0f);
  ModRM(0, from, DISP32);
  write32(to);
}

void ix86::MOVAPSRtoR(int32_t to, int32_t from) {
  write16(0x290f);
  ModRM(3, to, from);
}

void ix86::ORPSMtoR(int32_t to, int32_t from) {
  write16(0x560f);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::ORPSRtoR(int32_t to, int32_t from) {
  write16(0x560f);
  ModRM(3, to, from);
}

void ix86::XORPSMtoR(int32_t to, int32_t from) {
  write16(0x570f);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::XORPSRtoR(int32_t to, int32_t from) {
  write16(0x570f);
  ModRM(3, to, from);
}

void ix86::ANDPSMtoR(int32_t to, int32_t from) {
  write16(0x540f);
  ModRM(0, to, DISP32);
  write32(from);
}

void ix86::ANDPSRtoR(int32_t to, int32_t from) {
  write16(0x540f);
  ModRM(3, to, from);
}

void ix86::PFCMPEQMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0xb0);
}

void ix86::PFCMPGTMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0xa0);
}

void ix86::PFCMPGEMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x90);
}

void ix86::PFADDMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x9e);
}

void ix86::PFADDRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x9e);
}

void ix86::PFSUBMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x9a);
}

void ix86::PFSUBRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x9a);
}

void ix86::PFMULMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0xb4);
}

void ix86::PFMULRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0xb4);
}

void ix86::PFRCPMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x96);
}

void ix86::PFRCPRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x96);
}

void ix86::PFRCPIT1RtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0xa6);
}

void ix86::PFRCPIT2RtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0xb6);
}

void ix86::PFRSQRTRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x97);
}

void ix86::PFRSQIT1RtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0xa7);
}

void ix86::PF2IDMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x1d);
}

void ix86::PF2IDRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x1d);
}

void ix86::PI2FDMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x0d);
}

void ix86::PI2FDRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x0d);
}

void ix86::PFMAXMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0xa4);
}

void ix86::PFMAXRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0xa4);
}

void ix86::PFMINMtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(0, to, DISP32);
  write32(from);
  write8(0x94);
}

void ix86::PFMINRtoR(int32_t to, int32_t from) {
  write16(0x0f0f);
  ModRM(3, to, from);
  write8(0x94);
}

#endif
