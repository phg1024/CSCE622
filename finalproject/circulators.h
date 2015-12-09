#ifndef CIRCULATORS_H
#define CIRCULATORS_H

template <class MeshType, typename Visitor,
          typename CirculationPolicy=std::function<typename MeshType::edge_descriptor(typename MeshType::edge_descriptor, const typename MeshType::graph_type&)>>
class EdgeCirculator {
public:
  typedef MeshType mesh_type;
  typedef typename MeshType::graph_type graph_type;
  typedef typename MeshType::edge_descriptor edge_descriptor;
  EdgeCirculator(edge_descriptor he, const graph_type& g, CirculationPolicy policy)
    : cycled(false), he(he), cur(he), g(g), policy(policy) {}

  EdgeCirculator& operator=(EdgeCirculator other) {
    std::swap(*this, other);
    return *this;
  }

  operator bool() const {
    return !cycled;
  }

  EdgeCirculator& operator++() {
    cur = policy(cur, g);
    cycled = (cur == he);
    return (*this);
  }

  EdgeCirculator operator++(int) {
    EdgeCirculator cpy = *this;
    cur = policy(cur, g);
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
  CirculationPolicy policy;
};

#endif  // CIRCULATORS_H
