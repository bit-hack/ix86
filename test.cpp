#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <Windows.h>

#include <cstdio>
#include <vector>

#include "ix86.h"

#define TEST_ASSERT(X)                                                         \
  if (!(X)) {                                                                  \
    return false;                                                              \
  }

struct test_t {
  test_t(const char *name) : name(name) {}
  virtual bool run(ix86 &x86) = 0;
  const char *name;
};

namespace {

int32_t call_code(ix86 &x86) {
  typedef int (*func_t)(void);
  int (*func)(void) = (func_t)x86.code();
  return func();
}

std::vector<test_t *> tests;

} // namespace {}

struct test_RET_t : public test_t {
  test_RET_t() : test_t("RET") {}
  bool run(ix86 &x86) {
    x86.MOV32ItoR(ix86::EAX, 1234);
    x86.RET(); // <--
    int val = call_code(x86);
    TEST_ASSERT(val == 1234);
    return true;
  }
};

struct test_MOV32RtoR_t : public test_t {
  test_MOV32RtoR_t() : test_t("MOV32RtoR") {}
  bool run(ix86 &x86) {
    int value = 3625;
    x86.MOV32ItoR(ix86::ECX, value);
    x86.MOV32RtoR(ix86::EAX, ix86::ECX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV32RtoM_t : public test_t {
  test_MOV32RtoM_t() : test_t("MOV32RtoM") {}
  bool run(ix86 &x86) {
    int value = -1;
    x86.MOV32ItoR(ix86::EAX, 54321);
    x86.MOV32RtoM(&value, ix86::EAX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(54321 == value);
    return true;
  }
};

struct test_MOV32MtoR_t : public test_t {
  test_MOV32MtoR_t() : test_t("MOV32MtoR") {}
  bool run(ix86 &x86) {
    int value = 3625;
    x86.MOV32MtoR(ix86::EAX, &value); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV32ItoR_t : public test_t {
  test_MOV32ItoR_t() : test_t("MOV32ItoR") {}
  bool run(ix86 &x86) {
    int value = 472581;
    x86.MOV32ItoR(ix86::EAX, value); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_MOV32ItoM_t : public test_t {
  test_MOV32ItoM_t() : test_t("MOV32ItoM") {}
  bool run(ix86 &x86) {
    int value = -1;
    x86.MOV32ItoM(&value, 12345); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(12345 == value);
    return true;
  }
};

struct test_MOV16RtoM_t : public test_t {
  test_MOV16RtoM_t() : test_t("MOV16RtoM") {}
  bool run(ix86 &x86) {
    uint16_t value = -1;
    x86.MOV32ItoR(ix86::EAX, 0x7FFF1234);
    x86.MOV16RtoM(&value, ix86::AX); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0x1234 == value);
    return true;
  }
};

struct test_MOV16MtoR_t : public test_t {
  test_MOV16MtoR_t() : test_t("MOV16MtoR") {}
  bool run(ix86 &x86) {
    uint16_t value = 0x1337;
    x86.MOV32ItoR(ix86::EAX, 0);
    x86.MOV16MtoR(ix86::AX, &value); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(1234 == value);
    return true;
  }
};

struct test_MOV16ItoR_t : public test_t {
  test_MOV16ItoR_t() : test_t("MOV16ItoR") {}
  bool run(ix86 &x86) {
    uint16_t value = -1;
    x86.MOV16ItoM(&value, 0x3456); // <--
    x86.RET();
    int val = call_code(x86);
    TEST_ASSERT(0x3456 == value);
    return true;
  }
};

struct test_dummy_t : public test_t {
  test_dummy_t() : test_t("dummy") {}
  bool run(ix86 &x86) {
    return true;
  }
};

void collect_tests() {
  // test ret
  tests.push_back(new test_RET_t);
  // test mov
  tests.push_back(new test_MOV32RtoR_t);
  tests.push_back(new test_MOV32RtoM_t);
  tests.push_back(new test_MOV32MtoR_t);
  // TODO: MOV32RmtoR
  // TODO: MOV32RmStoR
  // TODO: MOV32RtoRm
  // TODO: MOV32RtoRmS
  tests.push_back(new test_MOV32ItoR_t);
  tests.push_back(new test_MOV32ItoM_t);
  tests.push_back(new test_MOV16RtoM_t);
  tests.push_back(new test_MOV16ItoR_t);

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
  ix86 x86{mem, size};

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
