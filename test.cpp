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
  test_t(const char *name) : name(name) {
  }
  virtual bool run(ix86 &x86) = 0;
  const char *name;
};

std::vector<test_t *> tests;

struct test_1_t : public test_t {
  test_1_t() : test_t("return value") {
  }
  bool run(ix86 &x86) {
    x86.MOV32ItoR(ix86::EAX, 1234);
    x86.RET();
    int (*foo)(void) = (int (*)(void))x86.code();
    int val = foo();
    TEST_ASSERT(val == 1234);
    return true;
  }
};

struct test_2_t : public test_t {
  test_2_t() : test_t("read mem") {
  }
  bool run(ix86 &x86) {
    int value = 3625;
    x86.MOV32MtoR(ix86::EAX, &value);
    x86.RET();
    int (*foo)(void) = (int (*)(void))x86.code();
    int val = foo();
    TEST_ASSERT(val == value);
    return true;
  }
};

struct test_dummy_t : public test_t {
  test_dummy_t() : test_t("dummy") {
  }
  bool run(ix86 &x86) {
    return true;
  }
};

void collect_tests() {
  tests.push_back(new test_1_t);
  tests.push_back(new test_2_t);
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
