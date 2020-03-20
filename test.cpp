#define WIN32_LEAN_AND_MEAN
#define VC_EXTRALEAN
#include <Windows.h>

#include "ix86.h"

int main(int argc, const char **args) {

  const size_t size = 1024;
  void *mem = VirtualAlloc(nullptr, size, MEM_COMMIT, PAGE_EXECUTE_READWRITE);
  if (!mem) {
    return 1;
  }

  ix86 x86{mem, size};
  x86.MOV32ItoR(ix86::EBX, 4);
  x86.MOV32ItoR(ix86::EAX, 1234);
  x86.ADD32RtoR(ix86::EAX, ix86::EBX);
  x86.RET();

  int (*foo)(void) = (int (*)(void))mem;
  int val = foo();

  return 0;
}
