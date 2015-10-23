// Author: Peihong Guo
#include <string>
#include <vector>
#include <iostream>
#include <cassert>
using namespace std;

#include "p1_triple.hpp"

// Helpers for writing vector and triple to ostream
template <typename T>
ostream& operator<<(ostream& os, const vector<T>& v) {
  os << '[';
  for(auto &x : v) {
    os << x << ' ';
  }
  os << ']';
  return os;
}

template <typename T1, typename T2, typename T3>
ostream& operator<<(ostream& os, const triple<T1, T2, T3>& t) {
  os << '(' << t.first << ", " << t.second << ", " << t.third << ')';
  return os;
}

// Default exchange function utilizing a temporary variable for swapping.
template <typename T>
void exchange(T& x, T& y) {
  cout << "using default exchange: " << x << " <-> " << y << endl;
  T tmp = x;
  x = y;
  y = tmp;
}

// Specialized exchange function for integers using bit operations to perform
// exchange.
template <>
void exchange(int& x, int& y) {
  cout << "using integer exchange: " << x << " <-> " << y << endl;
  x^=y^=x^=y;
}

// Overloaded exchange function for vectors using the swap function in vector
// class.
template <typename T, typename Allocator = std::allocator<T>>
void exchange(vector<T, Allocator>& x, vector<T, Allocator>& y) {
  cout << "using vector exchange: " << x << " <-> " << y << endl;
  x.swap(y);
}

// Overloaded exchange function for triple to take advantage of other exchange
// functions.
template <typename T1, typename T2, typename T3>
void exchange(triple<T1, T2, T3>& x, triple<T1, T2, T3>& y) {
  cout << "using triple exchange: " << x << " <-> " << y << endl;
  exchange(x.first, y.first);
  exchange(x.second, y.second);
  exchange(x.third, y.third);
}

// Helper to test exchange function.
template <typename T>
void test_exchange(T x, T y) {
  cout << "before:" << endl;
  cout << "x = " << x << endl
       << "y = " << y << endl;
  exchange(x, y);
  cout << "after:" << endl;
  cout << "x = " << x << endl
       << "y = " << y << endl;
}

int main(int argc, char **argv) {
  int x = 1, y = 2;
  test_exchange(x, y);

  vector<char> xv{'r', 'g', 'b', 'a'}, yv{'x', 'y', 'z', 'w'};
  test_exchange(xv, yv);

  triple<int, vector<char>, string> xt(1, xv, "pixel"), yt(2, yv, "position");
  test_exchange(xt, yt);

  return 0;
}
