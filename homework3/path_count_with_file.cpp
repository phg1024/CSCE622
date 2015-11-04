#include "graph_utils.h"
#include "path_count.h"

#include <fstream>
#include <iostream>
#include <iterator>
using namespace std;

template <typename Vertex, typename Graph>
bool ReadGraphFromFile(const string& filename, Graph& g, map<string, Vertex>& vertex_map) {
  ifstream fin(filename);
  if(!fin) {
    cout << "Failed to read file " << filename << endl;
    return false;
  }
  int num_vertices, num_edges;
  fin >> num_vertices >> num_edges;

  auto name_map = get(vertex_name, g);

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
  PrintGraph(g);

  return true;
}

template <typename Vertex, typename Graph>
void ProcessQueries(Graph& g, const map<string, Vertex>& vertex_map, const vector<pair<string, string>>& queries) {
  for(auto& query : queries) {
    Vertex s = vertex_map.at(query.first);
    Vertex t = vertex_map.at(query.second);
    cout << "Finding paths from " << get(vertex_name, g, s) << " to " << get(vertex_name, g, t) << endl;
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
  using Graph = adjacency_list<listS, vecS, directedS,
                               property<vertex_name_t, string,
                                        property<vertex_color_t, default_color_type>>>;
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
    cout << queries.size() << endl;
    ProcessQueries(g, vertex_map, queries);

    return 0;
  }
}
