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

namespace std {
ostream& operator<<(ostream& os, glm::vec3 v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
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
  cout << hds << endl;

  return 0;
}
