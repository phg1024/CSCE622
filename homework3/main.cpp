#include <iostream>
using namespace std;

#include <boost/graph/adjacency_list.hpp>
using namespace boost;

template <typename VertexDesc, typename EdgeDesc>
using Graph = adjacency_list<listS, vecS, directedS, VertexDesc, EdgeDesc>;

template <typename VertexListGraph>
typename graph_traits<VertexListGraph>::edges_size_type
path_count(VertexListGraph& G,
           typename graph_traits<VertexListGraph>::vertex_descriptor source,
           typename graph_traits<VertexListGraph>::vertex_descriptor target)
{
  return 0;
}

struct Vertex {
  Vertex(){}
  Vertex(int id):id(id){}
  int id;
};

struct Edge {
  double weight;
};

int main(int argc, char** argv) {
  using graph = Graph<Vertex, Edge>;
  using vertex_desc = typename graph_traits<graph>::vertex_descriptor;

  graph G;
  // Read the input file

  // Read the source and target Vertex
  vertex_desc s, t;
  cout << "Path count = " << path_count(G, s, t) << endl;
  return 0;
}
