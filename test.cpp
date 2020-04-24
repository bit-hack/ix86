#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <Windows.h>

#include <cstdio>
#include <vector>

#include "runasm.h"

#define TEST_ASSERT(X)                                                         \
  if (!(X)) {                                                                  \
    return false;                                                              \
  }

using namespace runasm;

struct test_t {
  test_t(const char *name) : name(name) {}
  virtual bool run(runasm_t &x86) = 0;
  const char *name;
};

namespace {

int32_t call_code(runasm_t &x86) {
  typedef int (*func_t)(void);
  func_t func = (func_t)x86.code();
  return func();
}

int32_t call_code(runasm_t &x86, int arg) {
  typedef int (*func_t)(int);
  func_t func = (func_t)x86.code();
  return func(arg);
}

std::vector<test_t *> tests;

} // namespace {}

struct test_RET_t : public test_t {
  test_RET_t() : test_t("RET") {}
  bool run(runasm_t &x86) {
    x86.MOV(EAX, 1234);
    x86.RET(); // <--
    int val = call_code(x86);
    TEST_ASSERT(val == 1234);
    return true;
  }
};

struct test_MOV_32RtoR_t : public test_t {
  test_MOV_32RtoR_t() : test_t("MOV_32RtoR") {}
  bool run(runasm_t &x86) {
    int value = 3625;
    x86.MOV(ECX, value);
    x86.MOV(EAX, ECX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV_32RtoM_t : public test_t {
  test_MOV_32RtoM_t() : test_t("MOV_32RtoM") {}
  bool run(runasm_t &x86) {
    uint32_t value = -1;
    x86.MOV(EAX, 54321);
    x86.MOV(&value, EAX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(54321 == value);
    return true;
  }
};

struct test_MOV_32MtoR_t : public test_t {
  test_MOV_32MtoR_t() : test_t("MOV_32MtoR") {}
  bool run(runasm_t &x86) {
    uint32_t value = 3625;
    x86.MOV(EAX, &value); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV_32ItoR_t : public test_t {
  test_MOV_32ItoR_t() : test_t("MOV_32ItoR") {}
  bool run(runasm_t &x86) {
    int value = 472581;
    x86.MOV(EAX, value); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV_32RmtoR_t : public test_t {
  test_MOV_32RmtoR_t() : test_t("MOV_32RmtoR") {}
  bool run(runasm_t &x86) {
    int value = 472581;
    x86.MOV(ECX, (uint32_t)&value);
    x86.MOV(EAX, deref_t(ECX)); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV_32ItoM_t : public test_t {
  test_MOV_32ItoM_t() : test_t("MOV_32ItoM") {}
  bool run(runasm_t &x86) {
    uint32_t value = -1;
    x86.MOV(&value, 12345); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(12345 == value);
    return true;
  }
};

struct test_MOV_16RtoM_t : public test_t {
  test_MOV_16RtoM_t() : test_t("MOV_16RtoM") {}
  bool run(runasm_t &x86) {
    uint16_t value = -1;
    x86.MOV(EAX, 0x7FFF1234);
    x86.MOV(&value, AX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0x1234 == value);
    return true;
  }
};

struct test_MOV_16MtoR_t : public test_t {
  test_MOV_16MtoR_t() : test_t("MOV_16MtoR") {}
  bool run(runasm_t &x86) {
    uint16_t value = 0x1337;
    x86.MOV(EAX, 0u);
    x86.MOV(AX, &value); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(1234 == value);
    return true;
  }
};

struct test_MOV_16ItoR_t : public test_t {
  test_MOV_16ItoR_t() : test_t("MOV_16ItoR") {}
  bool run(runasm_t &x86) {
    uint16_t value = -1;
    x86.MOV(&value, 0x3456); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0x3456 == value);
    return true;
  }
};

struct test_MOV_reg_sib_4_t : public test_t {
  test_MOV_reg_sib_4_t() : test_t("MOV_reg_sib_4") {}
  bool run(runasm_t &x86) {
    uint32_t value[8] = {7, 6, 5, 4, 3, 2, 1, 0};
    int index = 2;
    x86.MOV(ECX, (uint32_t)value);
    x86.MOV(EDX, index);
    // should be 8b 04 91             mov eax,dword ptr [ecx+edx*4]
    x86.MOV(EAX, sib_t(4, EDX, ECX)); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value[index]);
    return true;
  }
};

struct test_MOV_reg_sib_2_t : public test_t {
  test_MOV_reg_sib_2_t() : test_t("MOV_reg_sib_2") {}
  bool run(runasm_t &x86) {
    uint16_t value[8] = {7, 6, 5, 4, 3, 2, 1, 0};
    x86.MOV(ECX, (uint32_t)value);
    x86.MOV(EDX, 1);
    x86.MOV(EAX, sib_t(2, EDX, ECX)); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == 0x00050006);
    return true;
  }
};

struct test_MOV_reg_sib_1_t : public test_t {
  test_MOV_reg_sib_1_t() : test_t("MOV_reg_sib_1") {}
  bool run(runasm_t &x86) {
    uint8_t value[8] = {7, 6, 5, 4, 3, 2, 1, 0};
    x86.MOV(ECX, (uint32_t)value);
    x86.MOV(EDX, 1);
    x86.MOV(EAX, sib_t(1, EDX, ECX)); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == 0x03040506);
    return true;
  }
};

struct test_MOV_sib_4_reg_t : public test_t {
  test_MOV_sib_4_reg_t() : test_t("MOV_sib_4_reg") {}
  bool run(runasm_t &x86) {
    uint32_t value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int index = 2;
    x86.MOV(EAX, 0xaabbccdd);
    x86.MOV(ECX, (uint32_t)value);
    x86.MOV(EDX, index);
    x86.MOV(sib_t(4, EDX, ECX), EAX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0xaabbccdd == value[index]);
    return true;
  }
};

struct test_MOV_sib_2_reg_t : public test_t {
  test_MOV_sib_2_reg_t() : test_t("MOV_sib_2_reg") {}
  bool run(runasm_t &x86) {
    uint16_t value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    x86.MOV(EAX, 0xaabbccdd);
    x86.MOV(ECX, (uint32_t)value);
    x86.MOV(EDX, 1);
    x86.MOV(sib_t(2, EDX, ECX), EAX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0xccdd == value[1]);
    TEST_ASSERT(0xaabb == value[2]);
    return true;
  }
};

struct test_MOV_sib_1_reg_t : public test_t {
  test_MOV_sib_1_reg_t() : test_t("MOV_sib_1_reg") {}
  bool run(runasm_t &x86) {
    uint8_t value[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    x86.MOV(EAX, 0xaabbccdd);
    x86.MOV(ECX, (uint32_t)value);
    x86.MOV(EDX, 1);
    x86.MOV(sib_t(1, EDX, ECX), EAX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0xcc == value[2]);
    return true;
  }
};

struct test_sib_esp_1_t : public test_t {
  test_sib_esp_1_t() : test_t("test_sib_esp_1_t") {
  }
  bool run(runasm_t &x86) {

    // 8b 44 24 08
    //
    // modrm    01 000 100
    // sib      00 100 100

    x86.MOV(EAX, deref_t(ESP, 4));
    x86.RET();

    int ret = call_code(x86, 1234);
    return ret == 1234;
  }
};

struct test_sib_esp_2_t : public test_t {
  test_sib_esp_2_t() : test_t("test_sib_esp_2_t") {}
  bool run(runasm_t &x86) {
    uint32_t x[16] = {0};
    x86.MOV(ECX, ESP);
    x86.MOV(ESP, uint32_t(x));
    x86.MOV(EAX, 1234);
    x86.MOV(deref_t(ESP, 8), EAX);
    x86.MOV(EAX, deref_t(ESP, 8));
    x86.MOV(ESP, ECX);
    x86.RET();
    int ret = call_code(x86, 1234);
    return ret == 1234 && x[2] == 1234;
  }
};

struct test_JMP_8_t : public test_t {
  test_JMP_8_t() : test_t("JMP_8") {}
  bool run(runasm_t &x86) {
    rel8_t J1 = x86.JMP8(); // <--

    label_t L1 = x86.label();
    x86.MOV(EAX, 0u);
    x86.RET();

    label_t L2 = x86.label();
    x86.MOV(EAX, 1);
    x86.RET();

    x86.setTarget(J1, L2);

    int val = call_code(x86);
    TEST_ASSERT(val == 1);
    return true;
  }
};

struct test_JMP_32_t : public test_t {
  test_JMP_32_t() : test_t("JMP_32") {}
  bool run(runasm_t &x86) {
    rel32_t J1 = x86.JMP32(); // <--

    label_t L1 = x86.label();
    x86.MOV(EAX, 0u);
    x86.RET();

    label_t L2 = x86.label();
    x86.MOV(EAX, 1);
    x86.RET();

    x86.setTarget(J1, L2);

    int val = call_code(x86);
    TEST_ASSERT(val == 1);
    return true;
  }
};

struct test_CALL_32I_t : public test_t {
  test_CALL_32I_t() : test_t("CALL_32I") {}
  static int foo;
  static void f() {
    foo = 1;
  }
  bool run(runasm_t &x86) {
    x86.CALL(f); // <--
    x86.RET();
    call_code(x86);
    TEST_ASSERT(foo == 1);
    return true;
  }
};
int test_CALL_32I_t::foo = 0;

struct test_CALL_32_t : public test_t {
  test_CALL_32_t() : test_t("CALL_32") {}
  bool run(runasm_t &x86) {
    x86.XOR(EAX, EAX);
    rel32_t rel = x86.CALL(); // <--
    x86.RET();
    label_t L1 = x86.label();
    x86.XOR(EAX, EAX);
    x86.INC(EAX);
    x86.RET();
    x86.setTarget(rel, L1);

    int val = call_code(x86);
    TEST_ASSERT(val == 1);
    return true;
  }
};

struct test_CALL_32M_t : public test_t {
  test_CALL_32M_t() : test_t("CALL_32M") {}
  static int foo;
  static void f() {
    foo = 1;
  }
  bool run(runasm_t &x86) {
    void *ptr = f;
    x86.CALL_32M(&ptr); // <--
    x86.RET();
    call_code(x86);
    TEST_ASSERT(foo == 1);
    return true;
  }
};
int test_CALL_32M_t::foo = 0;

struct test_argument_1_t : public test_t {

  test_argument_1_t() : test_t("Argument_1") {
  }

  bool run(runasm_t &x86) {
    // prologue
    x86.PUSH(EBP);
    x86.MOV(EBP, ESP);
    // function
    x86.MOV(EAX, deref_t(EBP, 8));
    // epilogue
    x86.MOV(ESP, EBP);
    x86.POP(EBP);
    x86.RET();
    // test
    for (int i = 0; i < 10; ++i) {
      if (call_code(x86, i) != i) {
        return false;
      }
    }
    return true;
  }
};

struct test_fibonacci_t: public test_t {

  test_fibonacci_t() : test_t("Fibonacci") {
  }

  // reference function
  int fib(int n) {
    if (n <= 1)
      return n;
    return fib(n - 1) + fib(n - 2);
  }

  bool run(runasm_t &x86) {

    // top of the function to be callable
    auto FUNC = x86.label();

    // prologue
    x86.PUSH(EBP);
    x86.MOV(EBP, ESP);

    // function
    x86.MOV(EAX, deref_t(EBP, 8));      // get argument
    x86.MOV(ECX, 1);
    x86.CMP(EAX, ECX);
    auto L1_ = x86.Jcc32(cc_t::CC_GT);  // n > 1
    auto L2_ = x86.Jcc32(cc_t::CC_GT);  // n > 1

    // epilogue
    x86.MOV(ESP, EBP);
    x86.POP(EBP);
    x86.RET();                          // return n

    auto L1 = x86.label();
    x86.setTarget(L1_, L1);

    // fib(n - 1)
    x86.MOV(EAX, deref_t(EBP, 8));      // get argument
    x86.SUB(EAX, 1);                    // n - 1
    x86.PUSH(EAX);
    x86.CALL(FUNC);                     // fib(n - 1)
    x86.ADD(ESP, 4);                    // cleanup
    x86.PUSH(EAX);

    // fib(n - 2)
    x86.MOV(EAX, deref_t(EBP, 8));      // get argument
    x86.SUB(EAX, 2);                    // n - 2
    x86.PUSH(EAX);
    x86.CALL(FUNC);                     // fib(n - 2)
    x86.ADD(ESP, 4);                    // cleanup

    // fib(n - 2) + fib(n - 1)
    x86.POP(ECX);
    x86.ADD(EAX, ECX);

    // epilogue
    x86.MOV(ESP, EBP);
    x86.POP(EBP);
    x86.RET();

    // test it
    for (int i = 0; i < 32; ++i) {
      if (call_code(x86, i) != fib(i)) {
        return false;
      }
    }

    // success
    return true;
  }
};

struct test_dummy_t : public test_t {
  test_dummy_t() : test_t("dummy") {}
  bool run(runasm_t &x86) {
    return true;
  }
};

void collect_tests() {
  // test ret
  tests.push_back(new test_RET_t);
  // test mov
  tests.push_back(new test_MOV_32RtoR_t);
  tests.push_back(new test_MOV_32RtoM_t);
  tests.push_back(new test_MOV_32MtoR_t);

  tests.push_back(new test_MOV_reg_sib_4_t);
  tests.push_back(new test_MOV_reg_sib_2_t);
  tests.push_back(new test_MOV_reg_sib_1_t);
  tests.push_back(new test_MOV_sib_4_reg_t);
  tests.push_back(new test_MOV_sib_2_reg_t);
  tests.push_back(new test_MOV_sib_1_reg_t);

  tests.push_back(new test_sib_esp_1_t);
  tests.push_back(new test_sib_esp_2_t);

  tests.push_back(new test_MOV_32RmtoR_t);

  tests.push_back(new test_MOV_32ItoR_t);
  tests.push_back(new test_MOV_32ItoM_t);
  tests.push_back(new test_MOV_16RtoM_t);
  tests.push_back(new test_MOV_16ItoR_t);

  tests.push_back(new test_JMP_8_t);
  tests.push_back(new test_JMP_32_t);

  tests.push_back(new test_CALL_32I_t);
  tests.push_back(new test_CALL_32_t);
  tests.push_back(new test_CALL_32M_t);

  tests.push_back(new test_argument_1_t);

  tests.push_back(new test_fibonacci_t);

  tests.push_back(new test_dummy_t);
}

int main(int argc, const char **args) {

  collect_tests();

  // allocate some executable memory
  const size_t size = 1024;
  void *mem = VirtualAlloc(nullptr, size, MEM_COMMIT, PAGE_EXECUTE_READWRITE);
  if (!mem) {
    return 1;
  }
  runasm_t x86{mem, size};

  printf("--------------------------------\n");

  std::vector<test_t *> fails;
  std::vector<test_t *> passes;

  for (auto &test : tests) {
    x86.reset();
    bool res = test->run(x86);
    printf("  %s | %s\n", (res ? " " : "F"), test->name);
    if (res) {
      passes.push_back(test);
    } else {
      fails.push_back(test);
    }
  }

  printf("--------------------------------\n");

  // print test summary
  printf("%d of %d tests pass\n", int(passes.size()), int(tests.size()));
  printf("%d failed\n", int(fails.size()));
  for (auto &test : fails) {
    printf("  F | %s\n", test->name);
  }

  getchar();
  return 0;
}
