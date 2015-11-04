#ifndef PATH_COUNT_H
#define PATH_COUNT_H

#include <iostream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
using namespace boost;

template <typename VertexListGraph>
class PathCounter : public default_dfs_visitor {
public:
  using VertexDescriptor = typename graph_traits<VertexListGraph>::vertex_descriptor;
  using EdgeSizeType = typename graph_traits<VertexListGraph>::edges_size_type;

  PathCounter(const VertexDescriptor& s, const VertexDescriptor& t,
              EdgeSizeType& path_count, std::ostream& os = std::cout)
    : s(s), t(t), path_count(path_count), os(os) {}

  template <class Vertex, class Graph>
  void start_vertex(Vertex u, const Graph& g) {
    os << "start DFS from " << get(vertex_name, g, u) << std::endl;
    // Initialize the path counts
    auto vp = vertices(g);
    std::for_each(vp.first, vp.second, [&](Vertex v) {
      path_counter[v] = 0;
    });
  }

  template <class Vertex, class Graph>
  void discover_vertex(Vertex u, const Graph& g) {
    os << "discover " << get(vertex_name, g, u) << std::endl;
    if(u == t) path_counter[u] = 1;
    else path_counter[u] = 0;
  }

  template <class Vertex, class Graph>
  void finish_vertex(Vertex u, const Graph& g) {
    os << "finish " << get(vertex_name, g, u) << std::endl;

    // Store intermediate results
    if( u != t ) {
      auto vp = adjacent_vertices(u, g);
      path_counter[u] = std::accumulate(vp.first, vp.second, 0,
        [&](size_t acc, Vertex v) {
          return acc + path_counter[v];
        });
    }

    // Store final result
    if( u == s ) path_count = path_counter[u];

    os << get(vertex_name, g, u) << ": " << path_counter[u] << std::endl;
  }

private:
  VertexDescriptor s, t;
  std::map<VertexDescriptor, EdgeSizeType> path_counter;
  EdgeSizeType& path_count;
  std::ostream& os;
};

template <typename VertexListGraph>
typename graph_traits<VertexListGraph>::edges_size_type
path_count(VertexListGraph& G,
           typename graph_traits<VertexListGraph>::vertex_descriptor source,
           typename graph_traits<VertexListGraph>::vertex_descriptor target)
{
  typename graph_traits<VertexListGraph>::edges_size_type path_count;

  // Initialize the graph, set all nodes to white
  // This is necessary because this algorithm calls depth_first_visit directly instead of depth_first_search
  for(auto vp=vertices(G); vp.first!=vp.second; ++vp.first) {
    put(vertex_color, G, *vp.first, white_color);
  }
  
  depth_first_visit(G, source,
                    PathCounter<VertexListGraph>(source, target, path_count, std::clog),
                    get(vertex_color, G),
                    [&](typename graph_traits<VertexListGraph>::vertex_descriptor u,
                        const VertexListGraph&) { return u == target; });
  std::cout << "Path counting finished." << std::endl;
  return path_count;
}

#endif	// PATH_COUNT_H
