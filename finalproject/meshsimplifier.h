#ifndef MESH_SIMPLIFIER_H
#define MESH_SIMPLIFIER_H

#include "priority_queue.h"

#include <eigen3/Eigen/Dense>
using namespace Eigen;

namespace std {
ostream& operator<<(ostream& os, glm::vec3 v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}
}

template <class HDSMesh>
class MeshSimplifier {
public:
  typedef typename HDSMesh::vertex_descriptor vertex_descriptor;
  typedef typename HDSMesh::vertex_iterator vertex_iterator;

  typedef typename HDSMesh::face_descriptor face_descriptor;
  typedef typename HDSMesh::face_descriptor_hasher face_descriptor_hasher;
  typedef typename HDSMesh::face_iterator face_iterator;
  typedef std::unordered_map<face_descriptor, glm::vec3, face_descriptor_hasher>  face_normal_map;

  static int simplify(HDSMesh& mesh) {
    int num_removed_edges = 0;

    // compute face normal for each face
    face_normal_map face_normals;
    face_iterator fbegin, fend;
    tie(fbegin, fend) = mesh.faces();
    for(auto fit = fbegin; fit != fend; ++fit) {
      face_normals[*fit] = face_normal(*fit, mesh);
    }

    // compute QEF for each vertex
    std::unordered_map<vertex_descriptor, double> QEFs;
    vertex_iterator vbegin, vend;
    tie(vbegin, vend) = mesh.vertices();
    for(auto vit = vbegin; vit != vend; ++vit) {
      QEFs[*vit] = quadratic_error_function(*vit, face_normals, mesh);
    }

    return num_removed_edges;
  }

  static double quadratic_error_function(typename HDSMesh::vertex_descriptor v,
                                         face_normal_map& face_normals, const HDSMesh& mesh) {
    typedef typename HDSMesh::face_circulator face_circulator;
    double qef = 0;
    face_circulator circ = mesh.incident_faces(v);
    cout << "vertex " << v << ": ";
    while(circ) {
      cout << face_normals[*circ] << "\t";
      ++circ;
    }
    cout << endl;
    return qef;
  }

  static glm::vec3 face_normal(typename HDSMesh::face_descriptor f, const HDSMesh& mesh) {
    typedef typename HDSMesh::edge_circulator edge_circulator;
    edge_circulator circ = mesh.incident_edges(f);
    vector<glm::vec3> verts;

    while(circ) {
      verts.push_back(mesh.vertex(*circ));
      ++circ;
    }

    auto v0 = verts[0], v1 = verts[1], v2 = verts[2];
    return glm::normalize(glm::cross(v1 - v0, v2 - v0));
  }
};

#endif  // MESH_SIMPLIFIER_H
