#include <iostream>
#include <fstream>
using namespace std;

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
              EdgeSizeType& path_count, ostream& os = cout)
    : s(s), t(t), path_count(path_count), os(os) {}

  template <class Vertex, class Graph>
  void start_vertex(Vertex u, const Graph& g) {
    os << "start DFS from " << get(vertex_name, g, u) << endl;
    // Initialize the graph
    auto vp = vertices(g);
    std::for_each(vp.first, vp.second, [&](Vertex v) {
      path_counter[v] = 0;
    });
  }

  template <class Vertex, class Graph>
  void discover_vertex(Vertex u, const Graph& g) {
    os << "discover " << get(vertex_name, g, u) << endl;
    if(u == t) path_counter[u] = 1;
    else path_counter[u] = 0;
  }

  template <class Vertex, class Graph>
  void finish_vertex(Vertex u, const Graph& g) {
    os << "finish " << get(vertex_name, g, u) << endl;

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

    os << get(vertex_name, g, u) << ": " << path_counter[u] << endl;
  }

private:
  VertexDescriptor s, t;
  std::map<VertexDescriptor, EdgeSizeType> path_counter;
  EdgeSizeType& path_count;
  ostream& os;
};

template <typename VertexListGraph>
typename graph_traits<VertexListGraph>::edges_size_type
path_count(VertexListGraph& G,
           typename graph_traits<VertexListGraph>::vertex_descriptor source,
           typename graph_traits<VertexListGraph>::vertex_descriptor target)
{
  typename graph_traits<VertexListGraph>::edges_size_type path_count;
  
  depth_first_visit(G, source,
                    PathCounter<VertexListGraph>(source, target, path_count),
                    get(vertex_color, G),
                    [&](typename graph_traits<VertexListGraph>::vertex_descriptor u,
                        const VertexListGraph&) { return u == target; });
  cout << "Path counting finished." << endl;
  return path_count;
}

int main(int argc, char** argv) {
  using Graph = adjacency_list<listS, vecS, directedS,
                               property<vertex_name_t, string,
                                        property<vertex_color_t, default_color_type>>>;
  using VertexDescriptor = graph_traits<Graph>::vertex_descriptor;
  using IndexMap = property_map<Graph, vertex_index_t>::type;
  using NameMap = property_map<Graph, vertex_name_t>::type;

  if(argc<2) {
    cout << "Usage: path_count graph_file" << endl;
    return -1;
  } else {
    ifstream fin(argv[1]);
    if(!fin) {
      cout << "Failed to read file " << string(argv[1]) << endl;
      return -1;
    }
    int num_vertices, num_edges;
    fin >> num_vertices >> num_edges;

    Graph g;

    map<string, VertexDescriptor> vertex_map;
    NameMap name_map = get(vertex_name, g);

    for(int i=0;i<num_vertices;++i) {
      string name;
      fin >> name;
      auto vi = add_vertex(g);
      vertex_map[name] = vi;
      name_map[vi] = name;
    }

    int edge_desc_type;
    fin >> edge_desc_type;
    switch(edge_desc_type) {
      case 0: {
        for(int i=0;i<num_edges;++i) {
          string from_vertex, to_vertex;
          fin >> from_vertex >> to_vertex;
          add_edge(vertex_map[from_vertex], vertex_map[to_vertex], g);
        }
        break;
      }
      case 1: {
        for(int i=0;i<num_edges;++i) {
          int from_idx, to_idx;
          fin >> from_idx >> to_idx;
          add_edge(from_idx, to_idx, g);
        }
        break;
      }
    }

    // print out the vertices
    {
      IndexMap index = get(vertex_index, g);

      std::cout << "vertices(g) = ";
      typedef graph_traits<Graph>::vertex_iterator vertex_iter;
      std::pair<vertex_iter, vertex_iter> vp;
      for (vp = vertices(g); vp.first != vp.second; ++vp.first)
        std::cout << index[*vp.first] << ":" << name_map[*vp.first] <<  " ";
      std::cout << std::endl;
    }

    // print out the edges
    {
      std::cout << "edges(g) = ";
      typedef graph_traits<Graph>::edge_iterator edge_iter;
      std::pair<edge_iter, edge_iter> ep;
      for(ep = edges(g); ep.first != ep.second; ++ep.first) {
        auto s = source(*ep.first, g);
        auto t = target(*ep.first, g);
        cout << "(" << name_map[s] << " -> " << name_map[t] << ")" << " ";
      }
      cout << endl;
    }

    // Read the source and target Vertex
    VertexDescriptor s, t;
    switch(edge_desc_type) {
      case 0: {
        string from_vertex, to_vertex;
        fin >> from_vertex >> to_vertex;
        s = vertex_map[from_vertex];
        t = vertex_map[to_vertex];
        break;
      }
      case 1: {
        int from_idx, to_idx;
        fin >> from_idx >> to_idx;
        s = from_idx;
        t = to_idx;
        break;
      }
    }

    cout << "Finding paths from " << name_map[s] << " to " << name_map[t] << endl;
    cout << "Path count = " << path_count(g, s, t) << endl;
    return 0;
  }
}
