#include <fstream>
#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
using namespace std;

#include "meshloader.h"

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/IO/Polyhedron_iostream.h>

#include <boost/timer/timer.hpp>

typedef CGAL::Simple_cartesian<float>     Kernel;
typedef CGAL::Polyhedron_3<Kernel>         Polyhedron;
typedef Polyhedron::HalfedgeDS             HalfedgeDS;

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS> {
public:
  const std::vector<MeshLoader::vert_t> &coords;
  const std::vector<MeshLoader::face_t>    &tris;
  polyhedron_builder( const std::vector<MeshLoader::vert_t> &_coords, const std::vector<MeshLoader::face_t> &_tris ) : coords(_coords), tris(_tris) {}
  void operator()( HDS& hds) {
    typedef typename HDS::Vertex   Vertex;
    typedef typename Vertex::Point Point;

    boost::timer::auto_cpu_timer t("cgal hds built in %w seconds.\n");

    // create a cgal incremental builder
    CGAL::Polyhedron_incremental_builder_3<HDS> B( hds, true);
    B.begin_surface( coords.size(), tris.size() );

    // add the polyhedron vertices
    for(int i=0; i<(int)coords.size(); ++i){
      B.add_vertex( Point( coords[i].x, coords[i].y, coords[i].z ) );
    }

    // add the polyhedron triangles
    for( int i=0; i<(int)tris.size(); ++i ){
      B.begin_facet();
      B.add_vertex_to_facet( tris[i].v[0] );
      B.add_vertex_to_facet( tris[i].v[1] );
      B.add_vertex_to_facet( tris[i].v[2] );
      B.end_facet();
    }

    // finish up the surface
    B.end_surface();
  }
};

int main(int argc, char **argv) {
  if(argc <= 1) {
    cout << "Usage: ./mesh_simplification mesh_file" << endl;
    return -1;
  }
  string filename(argv[1]);
  OBJLoader loader(filename);

  // build a polyhedron from the loaded arrays
  {
    Polyhedron P;
    polyhedron_builder<HalfedgeDS> builder( loader.getVerts(), loader.getFaces() );
    P.delegate( builder );

    std::cout << "# faces = " << P.size_of_facets() << "\n";
    std::cout << "# vertices = " << P.size_of_vertices() << "\n";
  }

  return 0;
}
