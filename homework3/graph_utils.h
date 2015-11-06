#ifndef GRAPH_UTILS_H
#define GRAPH_UTILS_H

#include <iostream>
#include <map>
#include <string>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
using namespace boost;

template <typename Graph>
void PrintGraph(const Graph& g) {
  {
    auto index = get(vertex_index, g);
    std::cout << "vertices(g) = ";
    typedef typename graph_traits<Graph>::vertex_iterator vertex_iter;
    std::pair<vertex_iter, vertex_iter> vp;
    for (vp = vertices(g); vp.first != vp.second; ++vp.first)
      std::cout << index[*vp.first] << ":" << g[*vp.first].name <<  " ";
    std::cout << std::endl;
  }

  // print out the edges
  {
    std::cout << "edges(g) = ";
    typedef typename graph_traits<Graph>::edge_iterator edge_iter;
    std::pair<edge_iter, edge_iter> ep;
    for(ep = edges(g); ep.first != ep.second; ++ep.first) {
      auto s = source(*ep.first, g);
      auto t = target(*ep.first, g);
      std::cout << "(" << g[s].name << " -> " << g[t].name << ")" << " ";
    }
    std::cout << std::endl;
  }
}
#endif  // GRAPH_UTILS_H
