#ifndef PATH_COUNT_H
#define PATH_COUNT_H

#include <iostream>
#include <vector>
#include <list>
#include <string>
#include <map>

#include <boost/algorithm/string/join.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/properties.hpp>
#include <boost/property_map/property_map.hpp>
using namespace boost;

template <typename VertexListGraph>
class PathCounter : public default_dfs_visitor {
public:
  using VertexDescriptor = typename graph_traits<VertexListGraph>::vertex_descriptor;
  using EdgeSizeType = typename graph_traits<VertexListGraph>::edges_size_type;
  using path_t = std::list<std::string>;

  PathCounter(const VertexDescriptor& s, const VertexDescriptor& t,
              EdgeSizeType& path_count, std::ostream& os = std::cout)
    : s(s), t(t), path_count(path_count), os(os) {}

  ~PathCounter() {
    auto& paths_s = paths[s];
    std::cout << "Found paths:" << std::endl;
    std::for_each(paths_s.begin(), paths_s.end(), [&](const path_t& p) {
		    std::cout << boost::algorithm::join(p, " -> ") << std::endl;
		  });
  }

  template <class Vertex, class Graph>
  void start_vertex(Vertex u, const Graph& g) {
    os << "start DFS from " << g[u].name << std::endl;
  }

  template <class Vertex, class Graph>
  void discover_vertex(Vertex u, const Graph& g) {
    os << "discover " << g[u].name << std::endl;
    if(u==t) {
      path_counter[u] = 1;
      paths[u] = std::vector<path_t>(1, path_t(1, g[u].name));
    } else {
      path_counter[u] = 0;
      paths[u] = std::vector<path_t>();
    }
  }

  template <class Vertex, class Graph>
  void finish_vertex(Vertex u, const Graph& g) {
    os << "finish " << g[u].name << ": ";

    // Update the path count
    auto vp = adjacent_vertices(u, g);
    path_counter[u] += std::accumulate(vp.first, vp.second, 0,
                                       [&](size_t acc, Vertex v) {
                                         return acc + path_counter[v];
                                       });

    // Store the path as well
    auto& paths_u = paths[u];
    for(auto vp = adjacent_vertices(u, g); vp.first != vp.second; ++vp.first) {
      const auto& paths_v = paths[*vp.first];
      for(auto p : paths_v) {
        path_t this_path(1, g[u].name);
        this_path.insert(this_path.end(), p.begin(), p.end());
        paths_u.push_back(this_path);
      }
    }

    // Store final result
    if( u == s ) path_count = path_counter[u];

    os << "path counts[" << g[u].name << "] = " << path_counter[u] << std::endl;
  }

private:
  VertexDescriptor s, t;
  std::map<VertexDescriptor, EdgeSizeType> path_counter;
  std::map<VertexDescriptor, std::vector<path_t>> paths;
  EdgeSizeType& path_count;
  std::ostream& os;
};

template <typename VertexListGraph>
typename graph_traits<VertexListGraph>::edges_size_type
path_count(VertexListGraph& G,
           typename graph_traits<VertexListGraph>::vertex_descriptor source,
           typename graph_traits<VertexListGraph>::vertex_descriptor target)
{
  typedef typename graph_traits<VertexListGraph>::vertex_descriptor vertex_desc;

  std::map<vertex_desc, default_color_type> vertex2color;
  boost::associative_property_map<std::map<vertex_desc, default_color_type>> color_map(vertex2color);

  // Initialize the graph, set all nodes to white
  // This is necessary because this algorithm calls depth_first_visit directly instead of depth_first_search
  for(auto vp=vertices(G); vp.first!=vp.second; ++vp.first) {
    vertex2color.insert(std::make_pair(*vp.first, white_color));
  }

  typename graph_traits<VertexListGraph>::edges_size_type path_count;
  depth_first_visit(G, source,
                    PathCounter<VertexListGraph>(source, target, path_count, std::clog),
                    color_map,
                    [&](vertex_desc u, const VertexListGraph&) { return u == target; });
  std::cout << "Path counting finished." << std::endl;
  return path_count;
}

#endif	// PATH_COUNT_H
