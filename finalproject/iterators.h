#ifndef ITERATORS_H
#define ITERATORS_H

template <typename GraphType, class Visitor>
class Iterator {
public:
  typedef GraphType graph_type;
  typedef typename Visitor::iterator_type iterator_type;

  Iterator() : g(nullptr) {}
  Iterator(iterator_type cur, iterator_type last, const graph_type* g)
    : cur(cur), last(last), g(g) {}

  bool operator==(const Iterator& it) const {
    return cur == it.cur;
  }

  bool operator!=(const Iterator& it) const {
    return !(*this == it);
  }

  operator bool() {
    return cur != last;
  }

  Iterator& operator++() {
    ++cur;
    while(cur != last && !visitor.valid(cur, g)) ++cur;
    return (*this);
  }

  Iterator operator++(int) {
    Iterator cpy = *this;
    ++cur;
    while(cur != last && !!visitor.valid(cur, g)) ++cur;
    return cpy;
  }

  typename Visitor::value_type operator*() {
    return visitor(cur, g);
  }

private:
  iterator_type cur, last;
  const graph_type* g;
  Visitor visitor;
};

#endif  // ITERATORS_H
