#include "hdsmesh.h"

#include <iostream>
#include <fstream>
#include <string>
using namespace std;

using point_t = glm::vec3;
struct color_t {
  float r, g, b, a;
};
using weight_t = double;

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
    return -1;
  }
  string filename(argv[1]);
  OBJLoader loader(filename);

  {
    HalfEdgeDataStructure<point_t, NullType, NullType> hds;
    HalfEdgeDataStructure<point_t, NullType, NullType>::build(loader.getFaces(), loader.getVerts(), hds);

    std::cout << "# faces = " << hds.size_of_facets() << "\n";
    std::cout << "# vertices = " << hds.size_of_vertices() << "\n";

    if(argc > 2) {
      cout << hds << endl;
    }
  }
  return 0;
}
