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
    return -1;
  }
  string filename(argv[1]);
  ifstream fin(filename);
  // Load and build the HDS mesh
  fin >> hds;

  cout << hds << endl;
  return 0;
}
