#include "graph_utils.h"
#include "path_count.h"

#include <fstream>
#include <iostream>
#include <iterator>
using namespace std;

struct NamedVertex {
  NamedVertex() {}
  NamedVertex(const string& name) : name(name) {}

  string name;
};

template <typename Vertex, typename Graph>
bool ReadGraphFromFile(const string& filename, Graph& g, map<string, Vertex>& vertex_map) {
  ifstream fin(filename);
  if(!fin) {
    cout << "Failed to read file " << filename << endl;
    return false;
  }
  int num_vertices, num_edges;
  fin >> num_vertices >> num_edges;

  for(int i=0;i<num_vertices;++i) {
    string name;
    fin >> name;
    auto vi = add_vertex(NamedVertex(name), g);
    vertex_map[name] = vi;
  }

  for(int i=0;i<num_edges;++i) {
    string from_vertex, to_vertex;
    fin >> from_vertex >> to_vertex;
    add_edge(vertex_map[from_vertex], vertex_map[to_vertex], g);
  }

  // print out the vertices
  PrintGraph(g);

  return true;
}

template <typename Vertex, typename Graph>
void ProcessQueries(Graph& g, const map<string, Vertex>& vertex_map, const vector<pair<string, string>>& queries) {
  for(auto& query : queries) {
    Vertex s = vertex_map.at(query.first);
    Vertex t = vertex_map.at(query.second);
    cout << "Finding paths from " << g[s].name << " to " << g[t].name << endl;
    cout << "Path count = " << path_count(g, s, t) << endl;
  }
}

namespace std {
istream& operator>>(istream& is, pair<string, string>& p) {
  is >> p.first >> p.second;
  return is;
}
}

int main(int argc, char** argv) {
  using Graph = adjacency_list<listS, vecS, directedS, NamedVertex>;
  using VertexDescriptor = graph_traits<Graph>::vertex_descriptor;

  if(argc<3) {
    cout << "Usage: path_count_with_file graph_file query_file" << endl;
    return -1;
  } else {
    Graph g;
    map<string, VertexDescriptor> vertex_map;	// inverse lookup map: vertex name -> vertex descriptor
    if(!ReadGraphFromFile(argv[1], g, vertex_map)) {
      return -1;
    }

    // Read the source and target Vertex
    using query_t = pair<string, string>;
    vector<query_t> queries;
    ifstream query_file(argv[2]);
    std::copy(istream_iterator<query_t>(query_file), istream_iterator<query_t>(), back_inserter(queries));
    ProcessQueries(g, vertex_map, queries);

    return 0;
  }
}
