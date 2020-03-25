#pragma once

//
// Small X86 dynarec engine
//   github.com/bit-hack
//
// Based on runasm_t definitions v0.5.1 by:
//   linuzappz <linuzappz@pcsx.net>
//   alexey silinov
//
// Nice x86 reference at:
//   https://www.felixcloutier.com/x86/index.html
//

#include <cassert>
#include <cstdint>

namespace runasm {

// 8bit regs
enum gp_reg8_t {
  AL = 0,
  CL = 1,
  DL = 2,
  BL = 3,
  AH = 4,
  CH = 5,
  DH = 6,
  BH = 7,
};

// 16 bit regs
enum gp_reg16_t {
  AX = 0,
  CX = 1,
  DX = 2,
  BX = 3,
  SP = 4,
  BP = 5,
  SI = 6,
  DI = 7,
};

// 32 bit regs
enum gp_reg32_t {
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

enum cc_t {
  CC_O  = 0x0, // overflow         JO    (OF=1)
  CC_NO = 0x1, // not overflow     JNO   (OF=0)
  CC_C  = 0x2, // carry            JC    (CF=1)
  CC_AE = 0x3, // above or equal   JAE   (CF=0)
  CC_EQ = 0x4, // equal            JE    (ZF=1)
  CC_NE = 0x5, // not equal        JNE   (ZF=0)
  CC_BE = 0x6, // below or equal   JBE   (CF=1 or ZF=1)
  CC_AB = 0x7, // above            JA    (CF=0 and ZF=0)
  CC_S  = 0x8, // sign             JS    (SF=1)
  CC_NS = 0x9, // not sign         JNS   (SF=0)
  CC_P  = 0xa, // parity           JP    (PF=1)
  CC_NP = 0xb, // parity odd       JNP   (PF=0)
  CC_LT = 0xc, // less             JL    (SF!=OF)
  CC_GE = 0xd, // greater or equal JGE   (SF=OF)
  CC_LE = 0xe, // less or equal    JLE   (ZF=1 or SF!=OF)
  CC_GT = 0xf, // greater          JG    (ZF=0 and SF=OF)
};

// helper types
typedef uint8_t *rel8_t;
typedef uint32_t *rel32_t;
typedef void *mem8_t;
typedef void *mem16_t;
typedef void *mem32_t;

struct label_t {

  label_t() : ptr(nullptr) {
  }

  label_t(void *p) : ptr(p) {
  }

  operator bool() const {
    return ptr != nullptr;
  }

  void * const ptr;
};

struct deref_t {

  deref_t(gp_reg32_t r) : reg(r), type(type_reg) {
  }

  deref_t(mem32_t m) : mem(m), type(type_mem) {
  }

  union {
    gp_reg32_t reg;
    mem32_t mem;
  };

  bool is_reg() const {
    return type == type_reg;
  }

  bool is_mem() const {
    return type == type_mem;
  }

protected:
  enum type_t {
    type_reg,
    type_mem,
  };
  const type_t type;
};

struct runasm_t {

  // construct with target code buffer
  runasm_t(void *dst, size_t size)
      : start((int8_t *)dst)
      , ptr((int8_t *)dst)
      , end((int8_t *)dst + size) {
    assert(dst);
  }

  // return pointer to the code buffer
  uint8_t *code() const {
    return (uint8_t *)start;
  }

  // reset the instruction stream
  void reset() {
    ptr = start;
  }

  // write value to code buffer
  void write8(const uint8_t val);
  void write16(const uint16_t val);
  void write32(const uint32_t val);
  void write32(void *val);

  // set jump target
  void setTarget(rel8_t op);
  void setTarget(rel32_t op);
  void setTarget(rel8_t op, label_t target);
  void setTarget(rel32_t op, label_t target);

  // align instruction stream
  void align(int32_t bytes);

  // return a label at this point in the code stream
  label_t label() const;

  // mov r32 to r32
  void MOV_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // mov r32 to m32
  void MOV_32RtoM(mem32_t to, gp_reg32_t from);
  // mov m32 to r32
  void MOV_32MtoR(gp_reg32_t to, mem32_t from);
  // mov [r32] to r32
  void MOV_32RmtoR(gp_reg32_t to, deref_t from);
  // mov [r32][r32*scale] to r32
  void MOV_32RmStoR(gp_reg32_t to,
                    gp_reg32_t from,
                    gp_reg32_t from2,
                    scale_t scale);
  // mov r32 to [r32]
  void MOV_32RtoRm(deref_t to, gp_reg32_t from);
  // mov r32 to [r32][r32*scale]
  void MOV_32RtoRmS(gp_reg32_t to,
                    gp_reg32_t to2,
                    scale_t scale,
                    gp_reg32_t from);
  // mov imm32 to r32
  void MOV_32ItoR(gp_reg32_t to, uint32_t from);
  // mov imm32 to m32
  void MOV_32ItoM(mem32_t to, uint32_t from);

  // mov r16 to m16
  void MOV_16RtoM(mem16_t to, gp_reg16_t from);
  // mov m16 to r16
  void MOV_16MtoR(gp_reg16_t to, mem16_t from);
  // mov imm16 to m16
  void MOV_16ItoM(mem16_t to, uint16_t from);

  // mov r8 to m8
  void MOV_8RtoM(mem8_t to, gp_reg8_t from);
  // mov m8 to r8
  void MOV_8MtoR(gp_reg8_t to, mem8_t from);
  // mov imm8 to m8
  void MOV_8ItoM(mem8_t to, uint8_t from);

  // mov sign extend r8 to r32
  void MOVSX_32R8toR(gp_reg32_t to, gp_reg8_t from);
  // mov sign extend m8 to r32
  void MOVSX_32M8toR(gp_reg32_t to, mem8_t from);
  // mov sign extend r16 to r32
  void MOVSX_32R16toR(gp_reg32_t to, gp_reg16_t from);
  // mov sign extend m16 to r32
  void MOVSX_32M16toR(gp_reg32_t to, mem16_t from);

  // mov zero extend r8 to r32
  void MOVZX_32R8toR(gp_reg32_t to, gp_reg8_t from);
  // mov zero extend m8 to r32
  void MOVZX_32M8toR(gp_reg32_t to, mem8_t from);
  // mov zero extend r16 to r32
  void MOVZX_32R16toR(gp_reg32_t to, gp_reg16_t from);
  // mov zero extend m16 to r32
  void MOVZX_32M16toR(gp_reg32_t to, mem16_t from);

  // conditional move
  void CMOV_32RtoR(cc_t cc, gp_reg32_t to, gp_reg32_t from);
  // conditional move
  void CMOV_32MtoR(cc_t cc, gp_reg32_t to, mem32_t from);

  // add imm32 to r32
  void ADD_32ItoR(gp_reg32_t to, uint32_t from);
  // add imm32 to m32
  void ADD_32ItoM(mem32_t to, uint32_t from);
  // add r32 to r32
  void ADD_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // add r32 to m32
  void ADD_32RtoM(mem32_t to, gp_reg32_t from);
  // add m32 to r32
  void ADD_32MtoR(gp_reg32_t to, mem32_t from);

  // adc imm32 to r32
  void ADC_32ItoR(gp_reg32_t to, uint32_t from);
  // adc r32 to r32
  void ADC_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // adc m32 to r32
  void ADC_32MtoR(gp_reg32_t to, mem32_t from);

  // inc r32
  void INC_32R(gp_reg32_t to);
  // inc m32
  void INC_32M(mem32_t to);

  // sub imm32 to r32
  void SUB_32ItoR(gp_reg32_t to, uint32_t from);
  // sub r32 to r32
  void SUB_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // sub m32 to r32
  void SUB_32MtoR(gp_reg32_t to, mem32_t from);

  // sbb imm32 to r32
  void SBB_32ItoR(gp_reg32_t to, uint32_t from);
  // sbb r32 to r32
  void SBB_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // sbb m32 to r32
  void SBB_32MtoR(gp_reg32_t to, mem32_t from);

  // dec r32
  void DEC_32R(gp_reg32_t to);
  // dec m32
  void DEC_32M(mem32_t to);

  // mul eax by r32 to edx:eax
  void MUL_32R(gp_reg32_t from);
  // mul eax by m32 to edx:eax
  void MUL_32M(mem32_t from);

  // imul eax by r32 to edx:eax
  void IMUL_32R(gp_reg32_t from);
  // imul eax by m32 to edx:eax
  void IMUL_32M(mem32_t from);
  // imul r32 by r32 to r32
  void IMUL_32RtoR(gp_reg32_t to, gp_reg32_t from);

  // div eax by r32 to edx:eax
  void DIV_32R(gp_reg32_t from);
  // div eax by m32 to edx:eax
  void DIV_32M(mem32_t from);

  // idiv eax by r32 to edx:eax
  void IDIV_32R(gp_reg32_t from);
  // idiv eax by m32 to edx:eax
  void IDIV_32M(mem32_t from);

  // rotate carry right
  void RCR_32ItoR(int32_t to, int32_t from);

  // shl imm8 to r32
  void SHL_32ItoR(gp_reg32_t to, uint8_t from);
  // shl cl to r32
  void SHL_32CLtoR(gp_reg32_t to);

  // shr imm8 to r32
  void SHR_32ItoR(gp_reg32_t to, uint8_t from);
  // shr cl to r32
  void SHR_32CLtoR(gp_reg32_t to);

  // sar imm8 to r32
  void SAR_32ItoR(gp_reg32_t to, uint8_t from);
  // sar cl to r32
  void SAR_32CLtoR(gp_reg32_t to);

  // or imm32 to r32
  void OR_32ItoR(gp_reg32_t to, uint32_t from);
  // or imm32 to m32
  void OR_32ItoM(mem32_t to, uint32_t from);
  // or r32 to r32
  void OR_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // or r32 to m32
  void OR_32RtoM(mem32_t to, gp_reg32_t from);
  // or m32 to r32
  void OR_32MtoR(gp_reg32_t to, mem32_t from);

  // xor imm32 to r32
  void XOR_32ItoR(gp_reg32_t to, uint32_t from);
  // xor imm32 to m32
  void XOR_32ItoM(mem32_t to, uint32_t from);
  // xor r32 to r32
  void XOR_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // xor r32 to m32
  void XOR_32RtoM(mem32_t to, gp_reg32_t from);
  // xor m32 to r32
  void XOR_32MtoR(gp_reg32_t to, mem32_t from);

  // and imm32 to r32
  void AND_32ItoR(gp_reg32_t to, uint32_t from);
  // and imm32 to m32
  void AND_32ItoM(mem32_t to, uint32_t from);
  // and r32 to r32
  void AND_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // and r32 to m32
  void AND_32RtoM(mem32_t to, gp_reg32_t from);
  // and m32 to r32
  void AND_32MtoR(gp_reg32_t to, mem32_t from);

  // not r32
  void NOT_32R(gp_reg32_t from);
  // neg r32
  void NEG_32R(gp_reg32_t from);

  // conditional jump
  rel8_t CJMP_8Rel(cc_t cc, label_t to = label_t());
  rel32_t CJMP_32Rel(cc_t cc, label_t to = label_t());

  // jmp rel8
  rel8_t JMP_8(label_t to = label_t());
  // jmp rel32
  rel32_t JMP_32(label_t to = label_t());
  // jmp r32
  void JMP_32R(gp_reg32_t to);

  // call near, relative, displacement relative to next instruction
  void CALL_32I(void *func);
  // call near, relative, displacement relative to next instruction
  rel32_t CALL_32(label_t to = label_t());
  // call near, relative, displacement relative to next instruction
  void CALL_32R(gp_reg32_t to);
  // call far, absolute indirect, address given in m16:32
  // XXX: (mem32_t *to)
  void CALL_32M(mem32_t to);

  // bit test
  void BT_32ItoR(gp_reg32_t to, int32_t from);

  // cmp imm32 to r32
  void CMP_32ItoR(gp_reg32_t to, uint32_t from);
  // cmp imm32 to m32
  void CMP_32ItoM(mem32_t to, uint32_t from);
  // cmp r32 to r32
  void CMP_32RtoR(gp_reg32_t to, gp_reg32_t from);
  // cmp m32 to r32
  void CMP_32MtoR(gp_reg32_t to, mem32_t from);

  // test imm32 to r32
  void TEST_32ItoR(gp_reg32_t to, uint32_t from);
  // test r32 to r32
  void TEST_32RtoR(gp_reg32_t to, gp_reg32_t from);

  // set byte on condition
  void SET_8R(cc_t cc, gp_reg32_t to);

  // convert byte to word
  void CBW();
  // convert word to doubleword
  void CWD();
  // convert doubleword to quadword
  void CDQ();

  // push r32 to stack
  void PUSH_32R(gp_reg32_t from);
  // push m32 to stack
  void PUSH_32M(mem32_t from);
  // push imm32 to stack
  void PUSH_32I(uint32_t from);

  // pop r32 from stack
  void POP_32R(gp_reg32_t from);

  // push All General-Purpose Registers
  void PUSHA_32();
  // pop All General-Purpose Registers
  void POPA_32();

  // return
  void RET();

protected:

  void modRM(int32_t mod, int32_t rm, int32_t reg);
  void sibSB(int32_t ss, int32_t rm, int32_t index);

  int8_t *start;
  int8_t *ptr;
  const int8_t *end;

}; // struct runasm_t

} // namespace runasm
