#include "meshloader.h"
#include "hdsmesh.h"
#include "priority_queue.h"

#include <GL/glut.h>

#include <iostream>
#include <string>
using namespace std;

using point_t = glm::vec3;
struct color_t {
  float r, g, b, a;
};
using weight_t = double;

int main(int argc, char** argv) {
  // Load the mesh
  if(argc <= 2) {
    cout << "Usage: ./mesh_simplification input_mesh_file output_mesh_file" << endl;
  }
  string filename(argv[1]);
  OBJLoader loader;
  loader.load(filename);

  // Build the HDS mesh
  typedef HalfEdgeDataStructure<point_t, weight_t, color_t> HDSMesh;
  HDSMesh hds = build_half_edge_mesh<point_t, weight_t, color_t>(loader.getFaces(), loader.getVerts());

  std::cout << "Initial number of points: " << hds.size_of_vertices() << "\n";
  std::cout << "Initial number of edges: " << hds.size_of_halfedges() / 2 << "\n";
  std::cout << "Initial number of faces: " << hds.size_of_facets() << "\n";

  int r = 0;

  std::cout << "\nFinished...\n" << r << " edges removed.\n"
            << (hds.size_of_halfedges()/2) << " final edges.\n" ;

  return 0;
}
