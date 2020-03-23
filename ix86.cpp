#include <cassert>

#include "ix86.h"

namespace runasm {

enum {
  SIB = 4,
  DISP32 = 5,
};

label_t runasm_t::label() const {
  return ptr;
}

void runasm_t::write8(const uint8_t val) {
  assert((end - ptr) >= 1);
  *(uint8_t *)ptr = val;
  ptr += 1;
}

void runasm_t::write16(const uint16_t val) {
  assert((end - ptr) >= 2);
  *(uint16_t *)ptr = val;
  ptr += 2;
}

void runasm_t::write32(const uint32_t val) {
  assert((end - ptr) >= 4);
  *(uint32_t *)ptr = val;
  ptr += 4;
}

void runasm_t::write32(void *val) {
  assert((end - ptr) >= 4);
  *(uint32_t *)ptr = uint32_t(val);
  ptr += 4;
}

void runasm_t::setTarget(rel8_t j8, label_t target) {
  uint32_t jump = ((uint32_t)target - (uint32_t)j8) - 1;
  assert(jump <= 0x7f);
  *j8 = (uint8_t)jump;
}

void runasm_t::setTarget(rel32_t j32, label_t target) {
  *j32 = ((uint32_t)target - (uint32_t)j32) - 4;
}

void runasm_t::setTarget(rel8_t j8) {
  uint32_t jump = (ptr - (int8_t *)j8) - 1;
  assert(jump <= 0x7f);
  *j8 = (uint8_t)jump;
}

void runasm_t::setTarget(rel32_t j32) {
  *j32 = (ptr - (int8_t *)j32) - 4;
}

void runasm_t::align(int32_t bytes) {
  ptr = (int8_t *)(((uintptr_t)ptr + bytes) & ~(bytes - 1));
  assert(ptr <= end);
}

void runasm_t::modRM(int32_t mod, int32_t rm, int32_t reg) {
  write8((mod << 6) | (rm << 3) | (reg));
}

void runasm_t::sibSB(int32_t ss, int32_t rm, int32_t index) {
  write8((ss << 6) | (rm << 3) | (index));
}

// mov instructions

void runasm_t::MOV_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x89);
  modRM(3, from, to);
}

void runasm_t::MOV_32RtoM(mem32_t to, gp_reg32_t from) {
  assert(to);
  write8(0x89);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::MOV_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x8B);
  modRM(0, to, DISP32);
  write32(from);
}

void runasm_t::MOV_32RmtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x8B);
  modRM(0, to, from);
}

void runasm_t::MOV_32RmStoR(gp_reg32_t to, gp_reg32_t from, gp_reg32_t from2,
                        scale_t scale) {
  write8(0x8B);
  modRM(0, to, 0x4);
  sibSB(scale, from2, from);
}

void runasm_t::MOV_32RtoRm(gp_reg32_t to, gp_reg32_t from) {
  write8(0x89);
  modRM(0, from, to);
}

void runasm_t::MOV_32RtoRmS(gp_reg32_t to, gp_reg32_t to2, scale_t scale,
                        gp_reg32_t from) {
  write8(0x89);
  modRM(0, from, 0x4);
  sibSB(scale, to2, to);
}

void runasm_t::MOV_32ItoR(gp_reg32_t to, uint32_t from) {
  write8(0xB8 | to);
  write32(from);
}

void runasm_t::MOV_32ItoM(mem32_t to, uint32_t from) {
  assert(to);
  write8(0xC7);
  modRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void runasm_t::MOV_16RtoM(mem16_t to, gp_reg16_t from) {
  assert(to);
  write8(0x66);
  write8(0x89);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::MOV_16MtoR(gp_reg16_t to, mem16_t from) {
  assert(from);
  write8(0x66);
  write8(0x8B);
  modRM(0, to, DISP32);
  write32(from);
}

void runasm_t::MOV_16ItoM(mem16_t to, uint16_t from) {
  assert(to);
  write8(0x66);
  write8(0xC7);
  modRM(0, 0, DISP32);
  write32(to);
  write16(from);
}

void runasm_t::MOV_8RtoM(mem8_t to, gp_reg32_t from) {
  assert(to);
  write8(0x88);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::MOV_8MtoR(gp_reg32_t to, mem8_t from) {
  assert(from);
  write8(0x8A);
  modRM(0, to, DISP32);
  write32(from);
}

void runasm_t::MOV_8ItoM(mem8_t to, uint8_t from) {
  assert(to);
  assert(from);
  write8(0xC6);
  modRM(0, 0, DISP32);
  write32(to);
  write8(from);
}

// mov sign extend

void runasm_t::MOVSX_32R8toR(gp_reg32_t to, gp_reg8_t from) {
  write16(0xBE0F);
  modRM(3, to, from);
}

void runasm_t::MOVSX_32M8toR(gp_reg32_t to, mem8_t from) {
  assert(from);
  write16(0xBE0F);
  modRM(0, to, DISP32);
  write32(from);
}

void runasm_t::MOVSX_32R16toR(gp_reg32_t to, gp_reg16_t from) {
  write16(0xBF0F);
  modRM(3, to, from);
}

void runasm_t::MOVSX_32M16toR(gp_reg32_t to, mem16_t from) {
  assert(from);
  write16(0xBF0F);
  modRM(0, to, DISP32);
  write32(from);
}

// mov zero extend

void runasm_t::MOVZX_32R8toR(gp_reg32_t to, gp_reg8_t from) {
  write16(0xB60F);
  modRM(3, to, from);
}

void runasm_t::MOVZX_32M8toR(gp_reg32_t to, mem8_t from) {
  assert(from);
  write16(0xB60F);
  modRM(0, to, DISP32);
  write32(from);
}

void runasm_t::MOVZX_32R16toR(gp_reg32_t to, gp_reg16_t from) {
  write16(0xB70F);
  modRM(3, to, from);
}

void runasm_t::MOVZX_32M16toR(gp_reg32_t to, mem16_t from) {
  assert(from);
  write16(0xB70F);
  modRM(0, to, DISP32);
  write32(from);
}

// conditional move instructions

void runasm_t::CMOV_32RtoR(cc_t cc, gp_reg32_t to, gp_reg32_t from) {
  write8(0x0F);
  write8(0x40 | cc);
  modRM(3, to, from);
}

void runasm_t::CMOV_32MtoR(cc_t cc, gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x0F);
  write8(0x40 | cc);
  modRM(0, to, DISP32);
  write32(from);
}

// add instructions

void runasm_t::ADD_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x05);
  } else {
    write8(0x81);
    modRM(3, 0, to);
  }
  write32(from);
}

void runasm_t::ADD_32ItoM(mem32_t to, uint32_t from) {
  assert(to);
  write8(0x81);
  modRM(0, 0, DISP32);
  write32(to);
  write32(from);
}

void runasm_t::ADD_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x01);
  modRM(3, from, to);
}

void runasm_t::ADD_32RtoM(mem32_t to, gp_reg32_t from) {
  assert(to);
  write8(0x01);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::ADD_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x03);
  modRM(0, to, DISP32);
  write32(from);
}

// add with carry instructions

void runasm_t::ADC_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x15);
  } else {
    write8(0x81);
    modRM(3, 2, to);
  }
  write32(from);
}

void runasm_t::ADC_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x11);
  modRM(3, from, to);
}

void runasm_t::ADC_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x13);
  modRM(0, to, DISP32);
  write32(from);
}

// increment instructions

void runasm_t::INC_32R(gp_reg32_t to) {
  write8(0x40 + to);
}

void runasm_t::INC_32M(mem32_t to) {
  assert(to);
  write8(0xFF);
  modRM(0, 0, DISP32);
  write32(to);
}

// subtract instructions

void runasm_t::SUB_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x2D);
  } else {
    write8(0x81);
    modRM(3, 5, to);
  }
  write32(from);
}

void runasm_t::SUB_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x29);
  modRM(3, from, to);
}

void runasm_t::SUB_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x2B);
  modRM(0, to, DISP32);
  write32(from);
}

// subtract with borrow instructions

void runasm_t::SBB_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x1D);
  } else {
    write8(0x81);
    modRM(3, 3, to);
  }
  write32(from);
}

void runasm_t::SBB_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x19);
  modRM(3, from, to);
}

void runasm_t::SBB_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x1B);
  modRM(0, to, DISP32);
  write32(from);
}

// decrement instructions

void runasm_t::DEC_32R(gp_reg32_t to) {
  write8(0x48 + to);
}

void runasm_t::DEC_32M(mem32_t to) {
  assert(to);
  write8(0xFF);
  modRM(0, 1, DISP32);
  write32(to);
}

// multiply instructions

void runasm_t::MUL_32R(gp_reg32_t from) {
  write8(0xF7);
  modRM(3, 4, from);
}

void runasm_t::MUL_32M(mem32_t from) {
  assert(from);
  write8(0xF7);
  modRM(0, 4, DISP32);
  write32(from);
}

// integer multiply instructions

void runasm_t::IMUL_32R(gp_reg32_t from) {
  write8(0xF7);
  modRM(3, 5, from);
}

void runasm_t::IMUL_32M(mem32_t from) {
  assert(from);
  write8(0xF7);
  modRM(0, 5, DISP32);
  write32(from);
}

void runasm_t::IMUL_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write16(0xAF0F);
  modRM(3, to, from);
}

// divide instructions

void runasm_t::DIV_32R(gp_reg32_t from) {
  write8(0xF7);
  modRM(3, 6, from);
}

void runasm_t::DIV_32M(mem32_t from) {
  assert(from);
  write8(0xF7);
  modRM(0, 6, DISP32);
  write32(from);
}

// integer divide instructions

void runasm_t::IDIV_32R(gp_reg32_t from) {
  write8(0xF7);
  modRM(3, 7, from);
}

void runasm_t::IDIV_32M(mem32_t from) {
  assert(from);
  write8(0xF7);
  modRM(0, 7, DISP32);
  write32(from);
}

// rotate carry right instructions

void runasm_t::RCR_32ItoR(int32_t to, int32_t from) {
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

void runasm_t::SHL_32ItoR(gp_reg32_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe0 | to);
    return;
  }
  write8(0xC1);
  modRM(3, 4, to);
  write8(from);
}

void runasm_t::SHL_32CLtoR(gp_reg32_t to) {
  write8(0xD3);
  modRM(3, 4, to);
}

// shift right instructions

void runasm_t::SHR_32ItoR(gp_reg32_t to, uint8_t from) {
  if (from == 1) {
    write8(0xd1);
    write8(0xe8 | to);
    return;
  }
  write8(0xC1);
  modRM(3, 5, to);
  write8(from);
}

void runasm_t::SHR_32CLtoR(gp_reg32_t to) {
  write8(0xD3);
  modRM(3, 5, to);
}

// shift arithmetic right instructions

void runasm_t::SAR_32ItoR(gp_reg32_t to, uint8_t from) {
  write8(0xC1);
  modRM(3, 7, to);
  write8(from);
}

void runasm_t::SAR_32CLtoR(gp_reg32_t to) {
  write8(0xD3);
  modRM(3, 7, to);
}

// bitwise or instructions

void runasm_t::OR_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x0D);
  } else {
    write8(0x81);
    modRM(3, 1, to);
  }
  write32(from);
}

void runasm_t::OR_32ItoM(mem32_t to, uint32_t from) {
  assert(to);
  write8(0x81);
  modRM(0, 1, DISP32);
  write32(to);
  write32(from);
}

void runasm_t::OR_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x09);
  modRM(3, from, to);
}

void runasm_t::OR_32RtoM(mem32_t to, gp_reg32_t from) {
  assert(to);
  write8(0x09);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::OR_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x0B);
  modRM(0, to, DISP32);
  write32(from);
}

// bitwise xor instructions

void runasm_t::XOR_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x35);
  } else {
    write8(0x81);
    modRM(3, 6, to);
  }
  write32(from);
}

void runasm_t::XOR_32ItoM(mem32_t to, uint32_t from) {
  assert(to);
  write8(0x81);
  modRM(0, 6, DISP32);
  write32(to);
  write32(from);
}

void runasm_t::XOR_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x31);
  modRM(3, from, to);
}

void runasm_t::XOR_32RtoM(mem32_t to, gp_reg32_t from) {
  assert(to);
  write8(0x31);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::XOR_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x33);
  modRM(0, to, DISP32);
  write32(from);
}

// bitwise and instructions

void runasm_t::AND_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x25);
  } else {
    write8(0x81);
    modRM(3, 0x4, to);
  }
  write32(from);
}

void runasm_t::AND_32ItoM(mem32_t to, uint32_t from) {
  assert(to);
  write8(0x81);
  modRM(0, 0x4, DISP32);
  write32(to);
  write32(from);
}

void runasm_t::AND_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x21);
  modRM(3, from, to);
}

void runasm_t::AND_32RtoM(mem32_t to, gp_reg32_t from) {
  assert(to);
  write8(0x21);
  modRM(0, from, DISP32);
  write32(to);
}

void runasm_t::AND_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x23);
  modRM(0, to, DISP32);
  write32(from);
}

// bitwise not instruction

void runasm_t::NOT_32R(gp_reg32_t from) {
  write8(0xF7);
  modRM(3, 2, from);
}

// arithmetic negate instruction

void runasm_t::NEG_32R(gp_reg32_t from) {
  write8(0xF7);
  modRM(3, 3, from);
}

// jump instructions

rel8_t runasm_t::CJMP_8Rel(cc_t cc, label_t to) {
  write8(0x70 | cc);
  write8(0);
  rel8_t rel = (rel8_t)(ptr - 1);
  if (rel) {
    setTarget(rel, to);
  }
  return rel;
}

rel32_t runasm_t::CJMP_32Rel(cc_t cc, label_t to) {
  write8(0x0F);
  write8(0x80 | cc);
  write32(0u);
  rel32_t rel = (rel32_t)(ptr - 4);
  if (to) {
    setTarget(rel, to);
  }
  return rel;
}

rel8_t runasm_t::JMP_8(label_t to) {
  write8(0xEB);
  write8(0);
  rel8_t rel = (rel8_t)(ptr - 1);
  if (to) {
    setTarget(rel, to);
  }
  return rel;
}

rel32_t runasm_t::JMP_32(label_t to) {
  write8(0xE9);
  write32(0u);
  rel32_t rel = (rel32_t)(ptr - 4);
  if (to) {
    setTarget(rel, to);
  }
  return rel;
}

void runasm_t::JMP_32R(gp_reg32_t to) {
  write8(0xFF);
  modRM(3, 4, to);
}

// call subroutine instructions

void runasm_t::CALLFunc(void *func) {
  assert(func);
  write8(0xE8);
  write32(0u);
  rel32_t rel = (rel32_t)(ptr - 4);
  setTarget(rel, func);
}

rel32_t runasm_t::CALL_32(label_t to) {
  write8(0xE8);
  write32(0u);
  rel32_t rel = (rel32_t)(ptr - 4);
  if (to) {
    setTarget(rel, to);
  }
  return rel;
}

void runasm_t::CALL_32R(gp_reg32_t to) {
  write8(0xFF);
  modRM(3, 2, to);
}

void runasm_t::CALL_32M(mem32_t to) {
  assert(to);
  write8(0xFF);
  modRM(0, 2, DISP32);
  write32(to);
}

// bit test instruction

void runasm_t::BT_32ItoR(gp_reg32_t to, int32_t from) {
  write16(0xba0f);
  write8(0xe0 | to);
  write8(from);
}

// compare instruction

void runasm_t::CMP_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0x3D);
  } else {
    write8(0x81);
    modRM(3, 7, to);
  }
  write32(from);
}

void runasm_t::CMP_32ItoM(mem32_t to, uint32_t from) {
  assert(to);
  write8(0x81);
  modRM(0, 7, DISP32);
  write32(to);
  write32(from);
}

void runasm_t::CMP_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x39);
  modRM(3, from, to);
}

void runasm_t::CMP_32MtoR(gp_reg32_t to, mem32_t from) {
  assert(from);
  write8(0x3B);
  modRM(0, to, DISP32);
  write32(from);
}

// test instruction

void runasm_t::TEST_32ItoR(gp_reg32_t to, uint32_t from) {
  if (to == EAX) {
    write8(0xA9);
  } else {
    write8(0xF7);
    modRM(3, 0, to);
  }
  write32(from);
}

void runasm_t::TEST_32RtoR(gp_reg32_t to, gp_reg32_t from) {
  write8(0x85);
  modRM(3, from, to);
}

// conditional byte set instructions

void runasm_t::SET_8R(cc_t cc, gp_reg32_t to) {
  write8(0x0F);
  write8(cc);
  write8(0xC0 | to);
}

// convert byte to word instruction

void runasm_t::CBW() {
  write16(0x9866);
}

// convert word to doubleword instruction

void runasm_t::CWD() {
  write8(0x98);
}

// convert doubleword to quadword instruction

void runasm_t::CDQ() {
  write8(0x99);
}

// stack push instructions

void runasm_t::PUSH_32R(gp_reg32_t from) {
  write8(0x50 | from);
}

void runasm_t::PUSH_32M(mem32_t from) {
  assert(from);
  write8(0xFF);
  modRM(0, 6, DISP32);
  write32(from);
}

void runasm_t::PUSH_32I(uint32_t from) {
  write8(0x68);
  write32(from);
}

// stack pop instruction

void runasm_t::POP_32R(gp_reg32_t from) {
  write8(0x58 | from);
}

// push general purpose registers

void runasm_t::PUSHA_32() {
  write8(0x60);
}

// pop general purpose registers

void runasm_t::POPA_32() {
  write8(0x61);
}

// return from subroutine instruction

void runasm_t::RET() {
  write8(0xC3);
}

} // namespace runasm
