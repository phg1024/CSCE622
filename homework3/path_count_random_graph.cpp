#include "graph_utils.h"
#include "path_count.h"

#include <map>
using namespace std;

template <typename Graph>
Graph GenerateRandomGraph(int num_vertices, int num_edges) {
  Graph g;
  typedef typename graph_traits<Graph>::vertex_descriptor VertexDescriptor;
  map<string, VertexDescriptor> vertex_map;
  vector<string> names(num_vertices);
  auto name_map = get(vertex_name, g);

  // Add vertices
  for(int i=0;i<num_vertices;++i) {
    auto vi = add_vertex(g);
    string name_i = "vertex " + std::to_string(i);
    name_map[vi] = name_i;
    vertex_map[name_i] = vi;
    names[i] = name_i;
  }

  // Add edges
  set<pair<int, int>> edge_set;
  while(edge_set.size() < num_edges) {
    int u = rand() % num_vertices;
    int v = rand() % num_vertices;

    // This prevents cycles in the graph
    if( v >= u ) continue;

    // Add this edge only if they are not connected
    if(edge_set.find(make_pair(u, v)) == edge_set.end()) {
      add_edge(vertex_map[names[u]], vertex_map[names[v]], g);
      edge_set.insert(make_pair(u, v));
    }
  }
  PrintGraph(g);
  return g;
}

template <typename Graph>
vector<pair<typename graph_traits<Graph>::vertex_descriptor,
            typename graph_traits<Graph>::vertex_descriptor>>
GenerateQueries(const Graph& g, int num_queries) {
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  vector<Vertex> verts;
  for(auto vp = vertices(g); vp.first != vp.second; ++vp.first) {
    verts.push_back(*vp.first);
  }

  vector<pair<Vertex, Vertex>> queries;
  int num_vertices = verts.size();
  for(int i=0;i<num_queries;++i) {
    int u = rand() % num_vertices;
    int v = rand() % num_vertices;
    queries.push_back(make_pair(verts[u], verts[v]));
  }
  return queries;
}

template <typename Vertex, typename Graph>
void ProcessQueries(Graph& g, const vector<pair<Vertex, Vertex>>& queries) {
  for(auto& query : queries) {
    cout << "Finding paths from " << get(vertex_name, g, query.first)
         << " to " << get(vertex_name, g, query.second) << endl;
    cout << "Path count = " << path_count(g, query.first, query.second) << endl;
  }
}

int main(int argc, char **argv) {
  using Graph = adjacency_list<listS, vecS, directedS,
                               property<vertex_name_t, string,
                                        property<vertex_color_t, default_color_type>>>;
  using VertexDescriptor = graph_traits<Graph>::vertex_descriptor;
  if(argc < 4) {
    cout << "Usage: path_count_random_graph num_vertices num_edges num_queries" << endl;
    return -1;
  }

  int num_vertices = std::stoi(argv[1]);
  int num_edges = std::stoi(argv[2]);
  int num_queries = std::stoi(argv[3]);

  const int max_edges_number = (num_vertices * (num_vertices-1)) / 2;
  if( num_edges > max_edges_number ) {
    cout << "Number of edges is larger than maximum possible value: " << max_edges_number << endl;
    return -1;
  }
  if( num_queries <= 0 ) {
    cout << "Number of queries must be larger than 0." << endl;
    return -1;
  }

  srand(clock());
  Graph g = GenerateRandomGraph<Graph>(num_vertices, num_edges);
  cout << "graph generated." << endl;
  auto queries = GenerateQueries(g, num_queries);
  ProcessQueries(g, queries);
  return 0;
}
