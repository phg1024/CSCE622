#ifndef CIRCULATORS_H
#define CIRCULATORS_H

#include <functional>

template <class MeshType, typename Visitor>
class EdgeCirculator {
public:
  typedef MeshType mesh_type;
  typedef typename MeshType::graph_type graph_type;
  typedef typename MeshType::edge_descriptor edge_descriptor;
  typedef std::function<edge_descriptor(edge_descriptor, const graph_type&)> CirculationStrategy;

  EdgeCirculator(edge_descriptor he, const graph_type& g, CirculationStrategy strategy)
    : cycled(false), he(he), cur(he), g(g), strategy(strategy) {}

  EdgeCirculator& operator=(EdgeCirculator other) {
    std::swap(*this, other);
    return *this;
  }

  operator bool() const {
    return !cycled;
  }

  EdgeCirculator& operator++() {
    cur = strategy(cur, g);
    cycled = (cur == he);
    return (*this);
  }

  EdgeCirculator operator++(int) {
    EdgeCirculator cpy = *this;
    cur = strategy(cur, g);
    cycled = (cur == he);
    return cpy;
  }

  typename Visitor::value_type operator*() { return visitor(cur, g); }

private:
  bool cycled;
  edge_descriptor he;
  edge_descriptor cur;
  const graph_type& g;
  Visitor visitor;
  CirculationStrategy strategy;
};

#endif  // CIRCULATORS_H
