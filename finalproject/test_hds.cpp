#include "meshloader.h"
#include "hdsmesh.h"

#include <iostream>
#include <string>
using namespace std;

using point_t = glm::vec3;
struct color_t {
  float r, g, b, a;
};
using weight_t = double;

OBJLoader loader;
HalfEdgeDataStructure<point_t, weight_t, color_t> hds;

ostream& operator<<(ostream& os, glm::vec3 v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}

void print_hds(HalfEdgeDataStructure<point_t, weight_t, color_t>& hds) {
  cout << "vertices: " << endl;
  auto verts = hds.vertices();
  for(auto it = verts.first; it != verts.second; ++it) {
    cout << hds[*it] << "; ";
  }
  cout << endl;
  cout << "edges: " << endl;
  auto edges = hds.edges();
  for(auto it = edges.first; it != edges.second; ++it) {
    cout << *it << "; ";
  }
  cout << endl;

  cout << "faces: " << endl;
  typedef HalfEdgeDataStructure<point_t, weight_t, color_t>::face_iterator face_iterator;
  typedef HalfEdgeDataStructure<point_t, weight_t, color_t>::edge_descriptor edge_descriptor;
  typedef HalfEdgeDataStructure<point_t, weight_t, color_t>::edge_circulator_type edge_circulator;
  face_iterator fit, fend;
  for(tie(fit, fend) = hds.faces(); fit != fend; ++fit) {
    edge_circulator circ = hds.incident_edges(*fit);
    cout << "face " << *circ << ": ";
    while(circ) {
      cout << *circ << "; ";
      circ++;
    }
    cout << endl;
  }

  cout << "vertex circulation: " << endl;
  for(auto vit = verts.first; vit != verts.second; ++vit) {
    edge_circulator circ = hds.incident_edges(*vit);
    cout << "vertex " << *vit << ": ";
    while(circ) {
      cout << *circ << "; ";
      ++circ;
    }
    cout << endl;
  }
}

int main(int argc, char** argv) {
  // Load the mesh
  if(argc <= 1) {
    cout << "Usage: ./mesh_simplification mesh_file" << endl;
  }
  string filename(argv[1]);
  loader.load(filename);

  // Build the HDS mesh
  hds = build_half_edge_mesh<point_t, weight_t, color_t>(loader.getFaces(), loader.getVerts());
  print_hds(hds);

  return 0;
}
