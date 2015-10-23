// Author: Peihong Guo
/*
 * To use this program:
 *   1. Compile it directly: the compilation will fail because is_zero is
 * not defined.
 *   2. Uncomment IS_ZERO_VERSION_1 and compile: the compilation will fail
 * because DummyType(int v) and operator==(const DummyType& v) are not defined.
 *   3. Uncomment IS_ZERO_VERSION_1 and change ENABLE_IS_ZERO from 0 to 1: the
 * compilation succeeds.
 *   4. Uncomment IS_ZERO_VERSION_2 and compile: the compilation will fail
 * because operator==(int v) is not defined in DummyType.
 *   5. Uncomment IS_ZERO_VERSION_1 and change ENABLE_IS_ZERO from 0 to 1: the
 * compilation succeeds.
 */
//#define IS_ZERO_VERSION_1
//#define IS_ZERO_VERSION_2
#define ENABLE_IS_ZERO 0
#include "p2_triple.hpp"

#include <cassert>

struct DummyType {
  DummyType() {}
#ifdef IS_ZERO_VERSION_1
#if ENABLE_IS_ZERO
  explicit DummyType(int v) {}
  bool operator==(const DummyType& v) const { return true; }
#endif
#endif

#ifdef IS_ZERO_VERSION_2
#if ENABLE_IS_ZERO
  bool operator==(int v) const { return true; }
#endif
#endif
};

int main(int argc, char** argv) {
  triple<int, float, DummyType> t(0, 0.0, DummyType());
  assert(t.is_zero());
  return 0;
}
