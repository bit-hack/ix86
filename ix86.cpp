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

#include <cassert>

#include "ix86.h"

enum {
  SIB = 4,
  DISP32 = 5,
};

void ix86::write8(const uint8_t val) {
  assert((end - ptr) >= 1);
  *(uint8_t *)ptr = val;
  ptr += 1;
}

void ix86::write16(const uint16_t val) {
  assert((end - ptr) >= 2);
  *(uint16_t *)ptr = val;
  ptr += 2;
}

void ix86::write32(const uint32_t val) {
  assert((end - ptr) >= 4);
  *(uint32_t *)ptr = val;
  ptr += 4;
}

void ix86::write32(void *val) {
  assert((end - ptr) >= 4);
  *(uint32_t *)ptr = uint32_t(val);
  ptr += 4;
}

void ix86::setJ8(uint8_t *j8) {
  uint32_t jump = (ptr - (int8_t *)j8) - 1;
  assert(jump <= 0x7f);
  *j8 = (uint8_t)jump;
}

void ix86::setJ32(uint32_t *j32) {
  *j32 = (ptr - (int8_t *)j32) - 4;
}

void ix86::align(int32_t bytes) {
  ptr = (int8_t *)(((uintptr_t)ptr + bytes) & ~(bytes - 1));
  assert(ptr <= end);
}

void ix86::modRM(int32_t mod, int32_t rm, int32_t reg) {
  write8((mod << 6) | (rm << 3) | (reg));
}

void ix86::sibSB(int32_t ss, int32_t rm, int32_t index) {
  write8((ss << 6) | (rm << 3) | (index));
}

// mov instructions

void ix86::MOV32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x89);
  modRM(3, from, to);
}

void ix86::MOV32RtoM(void *to, gp_reg_t from) {
  write8(0x89);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV32MtoR(gp_reg_t to, void *from) {
  write8(0x8B);
  modRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV32RmtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x8B);
  modRM(0, to, from);
}

void ix86::MOV32RmStoR(gp_reg_t to, gp_reg_t from, gp_reg_t from2,
                       scale_t scale) {
  write8(0x8B);
  modRM(0, to, 0x4);
  sibSB(scale, from2, from);
}

void ix86::MOV32RtoRm(gp_reg_t to, gp_reg_t from) {
  write8(0x89);
  modRM(0, from, to);
}

void ix86::MOV32RtoRmS(gp_reg_t to, gp_reg_t to2, scale_t scale,
                       gp_reg_t from) {
  write8(0x89);
  modRM(0, from, 0x4);
  sibSB(scale, to2, to);
}

void ix86::MOV32ItoR(gp_reg_t to, uint32_t from) {
  write8(0xB8 | to);
  write32(from);
}

void ix86::MOV32ItoM(void *to, uint32_t from) {
  write8(0xC7);
  modRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void ix86::MOV16RtoM(void *to, gp_reg_t from) {
  write8(0x66);
  write8(0x89);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV16MtoR(gp_reg_t to, void *from) {
  write8(0x66);
  write8(0x8B);
  modRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV16ItoM(void *to, uint16_t from) {
  write8(0x66);
  write8(0xC7);
  modRM(0, 0, DISP32);
  write32(to);
  write16(from);
}

void ix86::MOV8RtoM(void *to, gp_reg_t from) {
  write8(0x88);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::MOV8MtoR(gp_reg_t to, void *from) {
  write8(0x8A);
  modRM(0, to, DISP32);
  write32(from);
}

void ix86::MOV8ItoM(void *to, uint8_t from) {
  write8(0xC6);
  modRM(0, 0, DISP32);
  write32(to);
  write8(from);
}

// mov sign extend

void ix86::MOVSX32R8toR(gp_reg_t to, gp_reg_t from) {
  write16(0xBE0F);
  modRM(3, to, from);
}

void ix86::MOVSX32M8toR(gp_reg_t to, void *from) {
  write16(0xBE0F);
  modRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVSX32R16toR(gp_reg_t to, gp_reg_t from) {
  write16(0xBF0F);
  modRM(3, to, from);
}

void ix86::MOVSX32M16toR(gp_reg_t to, void *from) {
  write16(0xBF0F);
  modRM(0, to, DISP32);
  write32(from);
}

// mov zero extend

void ix86::MOVZX32R8toR(gp_reg_t to, gp_reg_t from) {
  write16(0xB60F);
  modRM(3, to, from);
}

void ix86::MOVZX32M8toR(gp_reg_t to, void *from) {
  write16(0xB60F);
  modRM(0, to, DISP32);
  write32(from);
}

void ix86::MOVZX32R16toR(gp_reg_t to, gp_reg_t from) {
  write16(0xB70F);
  modRM(3, to, from);
}

void ix86::MOVZX32M16toR(gp_reg_t to, void *from) {
  write16(0xB70F);
  modRM(0, to, DISP32);
  write32(from);
}

// conditional move instructions

void ix86::CMOV32RtoR(cc_t cc, gp_reg_t to, gp_reg_t from) {
  write8(0x0F);
  write8(0x40 | cc);
  modRM(3, to, from);
}

void ix86::CMOV32MtoR(cc_t cc, gp_reg_t to, void *from) {
  write8(0x0F);
  write8(0x40 | cc);
  modRM(0, to, DISP32);
  write32(from);
}

// add instructions

void ix86::ADD32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x05);
  } else {
    write8(0x81);
    modRM(3, 0, to);
  }
  write32(from);
}

void ix86::ADD32ItoM(void *to, uint32_t from) {
  write8(0x81);
  modRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void ix86::ADD32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x01);
  modRM(3, from, to);
}

void ix86::ADD32RtoM(void *to, gp_reg_t from) {
  write8(0x01);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::ADD32MtoR(gp_reg_t to, void *from) {
  write8(0x03);
  modRM(0, to, DISP32);
  write32(from);
}

// add with carry instructions

void ix86::ADC32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x15);
  } else {
    write8(0x81);
    modRM(3, 2, to);
  }
  write32(from);
}

void ix86::ADC32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x11);
  modRM(3, from, to);
}

void ix86::ADC32MtoR(gp_reg_t to, void *from) {
  write8(0x13);
  modRM(0, to, DISP32);
  write32(from);
}

// increment instructions

void ix86::INC32R(gp_reg_t to) {
  write8(0x40 + to);
}

void ix86::INC32M(void *to) {
  write8(0xFF);
  modRM(0, 0, DISP32);
  write32(to);
}

// subtract instructions

void ix86::SUB32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x2D);
  } else {
    write8(0x81);
    modRM(3, 5, to);
  }
  write32(from);
}

void ix86::SUB32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x29);
  modRM(3, from, to);
}

void ix86::SUB32MtoR(gp_reg_t to, void *from) {
  write8(0x2B);
  modRM(0, to, DISP32);
  write32(from);
}

// subtract with borrow instructions

void ix86::SBB32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x1D);
  } else {
    write8(0x81);
    modRM(3, 3, to);
  }
  write32(from);
}

void ix86::SBB32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x19);
  modRM(3, from, to);
}

void ix86::SBB32MtoR(gp_reg_t to, void *from) {
  write8(0x1B);
  modRM(0, to, DISP32);
  write32(from);
}

// decrement instructions

void ix86::DEC32R(gp_reg_t to) {
  write8(0x48 + to);
}

void ix86::DEC32M(void *to) {
  write8(0xFF);
  modRM(0, 1, DISP32);
  write32(to);
}

// multiply instructions

void ix86::MUL32R(gp_reg_t from) {
  write8(0xF7);
  modRM(3, 4, from);
}

void ix86::MUL32M(void *from) {
  write8(0xF7);
  modRM(0, 4, DISP32);
  write32(from);
}

// integer multiply instructions

void ix86::IMUL32R(gp_reg_t from) {
  write8(0xF7);
  modRM(3, 5, from);
}

void ix86::IMUL32M(void *from) {
  write8(0xF7);
  modRM(0, 5, DISP32);
  write32(from);
}

void ix86::IMUL32RtoR(gp_reg_t to, gp_reg_t from) {
  write16(0xAF0F);
  modRM(3, to, from);
}

// divide instructions

void ix86::DIV32R(gp_reg_t from) {
  write8(0xF7);
  modRM(3, 6, from);
}

void ix86::DIV32M(void *from) {
  write8(0xF7);
  modRM(0, 6, DISP32);
  write32(from);
}

// integer divide instructions

void ix86::IDIV32R(gp_reg_t from) {
  write8(0xF7);
  modRM(3, 7, from);
}

void ix86::IDIV32M(void *from) {
  write8(0xF7);
  modRM(0, 7, DISP32);
  write32(from);
}

// rotate carry right instructions

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

// shift left instructions

void ix86::SHL32ItoR(gp_reg_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe0 | to);
    return;
  }
  write8(0xC1);
  modRM(3, 4, to);
  write8(from);
}

void ix86::SHL32CLtoR(gp_reg_t to) {
  write8(0xD3);
  modRM(3, 4, to);
}

// shift right instructions

void ix86::SHR32ItoR(gp_reg_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe8 | to);
    return;
  }
  write8(0xC1);
  modRM(3, 5, to);
  write8(from);
}

void ix86::SHR32CLtoR(gp_reg_t to) {
  write8(0xD3);
  modRM(3, 5, to);
}

// shift arithmetic right instructions

void ix86::SAR32ItoR(gp_reg_t to, uint8_t from) {
  write8(0xC1);
  modRM(3, 7, to);
  write8(from);
}

void ix86::SAR32CLtoR(gp_reg_t to) {
  write8(0xD3);
  modRM(3, 7, to);
}

// bitwise or instructions

void ix86::OR32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x0D);
  } else {
    write8(0x81);
    modRM(3, 1, to);
  }
  write32(from);
}

void ix86::OR32ItoM(void *to, uint32_t from) {
  write8(0x81);
  modRM(0, 1, DISP32);
  write32(to);
  write32(from);
}

void ix86::OR32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x09);
  modRM(3, from, to);
}

void ix86::OR32RtoM(void *to, gp_reg_t from) {
  write8(0x09);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::OR32MtoR(gp_reg_t to, void *from) {
  write8(0x0B);
  modRM(0, to, DISP32);
  write32(from);
}

// bitwise xor instructions

void ix86::XOR32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x35);
  } else {
    write8(0x81);
    modRM(3, 6, to);
  }
  write32(from);
}

void ix86::XOR32ItoM(void *to, uint32_t from) {
  write8(0x81);
  modRM(0, 6, DISP32);
  write32(to);
  write32(from);
}

void ix86::XOR32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x31);
  modRM(3, from, to);
}

void ix86::XOR32RtoM(void *to, gp_reg_t from) {
  write8(0x31);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::XOR32MtoR(gp_reg_t to, void *from) {
  write8(0x33);
  modRM(0, to, DISP32);
  write32(from);
}

// bitwise and instructions

void ix86::AND32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x25);
  } else {
    write8(0x81);
    modRM(3, 0x4, to);
  }
  write32(from);
}

void ix86::AND32ItoM(void *to, uint32_t from) {
  write8(0x81);
  modRM(0, 0x4, DISP32);
  write32(to);
  write32(from);
}

void ix86::AND32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x21);
  modRM(3, from, to);
}

void ix86::AND32RtoM(void *to, gp_reg_t from) {
  write8(0x21);
  modRM(0, from, DISP32);
  write32(to);
}

void ix86::AND32MtoR(gp_reg_t to, void *from) {
  write8(0x23);
  modRM(0, to, DISP32);
  write32(from);
}

// bitwise not instruction

void ix86::NOT32R(gp_reg_t from) {
  write8(0xF7);
  modRM(3, 2, from);
}

// arithmetic negate instruction

void ix86::NEG32R(gp_reg_t from) {
  write8(0xF7);
  modRM(3, 3, from);
}

// jump instructions

uint8_t *ix86::CJMP8Rel(cc_t cc, int32_t to) {
  write8(0x70 | cc);
  write8(to);
  return (uint8_t *)(ptr - 1);
}

uint32_t *ix86::CJMP32Rel(cc_t cc, int32_t to) {
  write8(0x0F);
  write8(0x80 | cc);
  write32(to);
  return (uint32_t *)(ptr - 4);
}

uint8_t *ix86::JMP8(uint8_t to) {
  write8(0xEB);
  write8(to);
  return (uint8_t *)(ptr - 1);
}

uint32_t *ix86::JMP32(uint32_t to) {
  write8(0xE9);
  write32(to);
  return (uint32_t *)(ptr - 4);
}

void ix86::JMP32R(int32_t to) {
  write8(0xFF);
  modRM(3, 4, to);
}

// call subroutine instructions

void ix86::CALLFunc(uint32_t func) {
  uint32_t to = func - ((uint32_t)ptr + 5);
  write8(0xE8);
  write32(to);
}

void ix86::CALL32(uint32_t to) {
  write8(0xE8);
  write32(to);
}

void ix86::CALL32R(gp_reg_t to) {
  write8(0xFF);
  modRM(3, 2, to);
}

void ix86::CALL32M(void *to) {
  write8(0xFF);
  modRM(0, 2, DISP32);
  write32(to);
}

// bit test instruction

void ix86::BT32ItoR(gp_reg_t to, int32_t from) {
  write16(0xba0f);
  write8(0xe0 | to);
  write8(from);
}

// compare instruction

void ix86::CMP32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x3D);
  } else {
    write8(0x81);
    modRM(3, 7, to);
  }
  write32(from);
}

void ix86::CMP32ItoM(void *to, uint32_t from) {
  write8(0x81);
  modRM(0, 7, DISP32);
  write32(to);
  write32(from);
}

void ix86::CMP32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x39);
  modRM(3, from, to);
}

void ix86::CMP32MtoR(gp_reg_t to, void *from) {
  write8(0x3B);
  modRM(0, to, DISP32);
  write32(from);
}

// test instruction

void ix86::TEST32ItoR(gp_reg_t to, uint32_t from) {
  if (to == EAX) {
    write8(0xA9);
  } else {
    write8(0xF7);
    modRM(3, 0, to);
  }
  write32(from);
}

void ix86::TEST32RtoR(gp_reg_t to, gp_reg_t from) {
  write8(0x85);
  modRM(3, from, to);
}

// conditional byte set instructions

void ix86::SET8R(cc_t cc, gp_reg_t to) {
  write8(0x0F);
  write8(cc);
  write8(0xC0 | to);
}

// convert byte to word instruction

void ix86::CBW() {
  write16(0x9866);
}

// convert word to doubleword instruction

void ix86::CWD() {
  write8(0x98);
}

// convert doubleword to quadword instruction

void ix86::CDQ() {
  write8(0x99);
}

// stack push instructions

void ix86::PUSH32R(gp_reg_t from) {
  write8(0x50 | from);
}

void ix86::PUSH32M(void *from) {
  write8(0xFF);
  modRM(0, 6, DISP32);
  write32(from);
}

void ix86::PUSH32I(uint32_t from) {
  write8(0x68);
  write32(from);
}

// stack pop instruction

void ix86::POP32R(gp_reg_t from) {
  write8(0x58 | from);
}

// push general purpose registers

void ix86::PUSHA32() {
  write8(0x60);
}

// pop general purpose registers

void ix86::POPA32() {
  write8(0x61);
}

// return from subroutine instruction

void ix86::RET() {
  write8(0xC3);
}
