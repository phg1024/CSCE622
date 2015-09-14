#include "p1_triple.hpp"
#include <cassert>

int main(int argc, char **argv) {
  triple<int, int, char> a;
  triple<int, int, char> b(1, 2, 'a');
  triple<int, int, char> c = b;
  a = b;
  assert(a.first == 1);
  assert(b.first == 1);
  assert(c.first == 1);
  typedef triple<int, int, char>::first_type t; // t is int
  assert(b == make_triple(1, 2, 'a'));
  ++b.first;
  assert(a != b);

  return 0;
}
