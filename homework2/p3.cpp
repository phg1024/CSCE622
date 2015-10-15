#include <algorithm>
#include <vector>
#include <iostream>
#include <initializer_list>

// Negates a unary predicate. This class works for both STL predicates and lambdas.
template <typename UnaryPredicate>
struct Negate {
  Negate(UnaryPredicate s) : s(s) {}

  template <typename T>
  bool operator()(const T& t) {
    return !s(t);
  }
  UnaryPredicate s;
};

// Comparer for `sorting` elements based on provided predicate.
// When used together with std::stable_sort, the elements are sorted such that
// elements that satisfy the predicat become the prefix of a sequence, and others
// become postfix.
template <typename UnaryPredicate>
struct Comparer {
  Comparer(UnaryPredicate s) : s(s) {}

  template <typename T>
  bool operator()(const T& a, const T&b) {
    return !s(a) && s(b);
  }
  UnaryPredicate s;
};

template <typename BidirectionalIterator, typename UnaryPredicate>
void dragdrop(BidirectionalIterator f, BidirectionalIterator l,
              BidirectionalIterator p, UnaryPredicate s) {
  std::stable_sort(f, p, Comparer<UnaryPredicate>(s));
  std::stable_sort(p, l, Comparer<Negate<UnaryPredicate>>(s));
}

int main() {
  std::vector<int> v = { 1, 20, 3, 40, 5, 60, 70, 80, 9 };

  dragdrop(v.begin(), v.end(), v.begin()+6, [](int i) { return i >= 10; });

  std::for_each(v.begin(), v.end(), [](int i) { std::cout << i << " "; });
}
