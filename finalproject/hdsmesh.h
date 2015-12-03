#ifndef HDSMESH_H
#define HDSMESH_H

#include "meshloader.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>
using namespace std;

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
using namespace boost;

// * Each vertex has an out-going halfedge
//  Implement this using bundled vertex property
// * Each edge corresponds to 2 twin halfedges
//  Implement this using bundled edge property
// * Each face has an incident halfedge
//  Implement this using an external map
// * Each halfedge has the following members
//  1. an incident vertex handle
//  2. an incident face handle (face on the left)
//  3. an incident edge handle
//  4. an opposite halfedge handle
//  5. a prev halfedge handle

template <typename HalfedgeDescriptor, typename VertexProperty>
struct HDSVertex {
  HDSVertex() {}
  HDSVertex(const VertexProperty& vp) : prop(vp) {}
  HalfedgeDescriptor he;
  VertexProperty prop;
};

template <typename HalfedgeDescriptor, typename VertexDescriptor,
          typename FaceDescriptor, typename EdgeProperty>
struct HDSEdge {
  HDSEdge() {}
  HDSEdge(const EdgeProperty& ep) : prop(ep) {}

  VertexDescriptor v;
  FaceDescriptor f;
  HalfedgeDescriptor flip, prev, next;
  EdgeProperty prop;
};

template <typename HalfedgeDescriptor, typename FaceProperty>
struct HDSFace {
  HDSFace() {}
  HDSFace(const FaceProperty& fp) : prop(fp) {}
  HDSFace(HalfedgeDescriptor he, const FaceProperty& fp) : he(he), prop(fp) {}
  HalfedgeDescriptor he;
  FaceProperty prop;
};

struct NullType {};

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty>
struct DefaultHalfedgeDataStructureTraits {
  template <typename VP, typename EP, typename GP>
  using g = adjacency_list<listS, vecS, bidirectionalS, VP, EP, GP>;

  typedef graph_traits<g<NullType, NullType, NullType>>::vertex_descriptor vertex_descriptor;
  typedef graph_traits<g<NullType, NullType, NullType>>::edge_descriptor edge_descriptor;
  typedef unsigned int face_descriptor;

  typedef HDSVertex<edge_descriptor, VertexProperty> vertex_type;
  typedef HDSEdge<edge_descriptor, vertex_descriptor, face_descriptor, EdgeProperty> edge_type;
  typedef HDSFace<edge_descriptor, FaceProperty> face_type;

  typedef std::vector<face_type> face_set;

  typedef VertexProperty vertex_property;
  typedef EdgeProperty edge_property;
  typedef FaceProperty face_property;

  typedef g<vertex_type, edge_type, face_set> graph_type;

  typedef typename graph_traits<graph_type>::vertex_iterator vertex_iterator;
  typedef typename graph_traits<graph_type>::edge_iterator edge_iterator;
  typedef typename face_set::iterator face_iterator;
};

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty,
          typename HalfEdgeDataStructureTraits=DefaultHalfedgeDataStructureTraits<VertexProperty, EdgeProperty, FaceProperty>>
class HalfEdgeDataStructure {
public:
  HalfEdgeDataStructure() {}

  typedef typename HalfEdgeDataStructureTraits::graph_type graph_type;

  typedef typename HalfEdgeDataStructureTraits::vertex_descriptor vertex_descriptor;
  typedef typename HalfEdgeDataStructureTraits::edge_descriptor edge_descriptor;
  typedef typename HalfEdgeDataStructureTraits::face_descriptor face_descriptor;

  typedef typename HalfEdgeDataStructureTraits::vertex_type vertex_type;
  typedef typename HalfEdgeDataStructureTraits::edge_type edge_type;
  typedef typename HalfEdgeDataStructureTraits::face_type face_type;

  typedef typename HalfEdgeDataStructureTraits::vertex_property vertex_property;
  typedef typename HalfEdgeDataStructureTraits::edge_property edge_property;
  typedef typename HalfEdgeDataStructureTraits::face_property face_property;

  typedef typename HalfEdgeDataStructureTraits::vertex_iterator vertex_iterator;
  typedef typename HalfEdgeDataStructureTraits::edge_iterator edge_iterator;
  typedef typename HalfEdgeDataStructureTraits::face_iterator face_iterator;

  vertex_descriptor add_vertex(const VertexProperty& vp) {
    return boost::add_vertex(vertex_type(vp), g);
  }

  pair<edge_descriptor, bool> add_edge(vertex_descriptor u, vertex_descriptor v,
                                       const EdgeProperty& ep) {
    return boost::add_edge(u, v, edge_type(ep), g);
  }

  face_descriptor add_face(edge_descriptor e, const FaceProperty& fp) {
    g[graph_bundle].push_back(face_type(e, fp));
    return g[graph_bundle].size() - 1;
  }

  vertex_property& operator[](vertex_descriptor v) {
    return g[v].prop;
  }
  const vertex_property& operator[](vertex_descriptor v) const {
    return g[v].prop;
  }

  edge_property& operator[](edge_descriptor e) {
    return g[e].prop;
  }
  const edge_property& operator[](edge_descriptor e) const {
    return g[e].prop;
  }

  pair<vertex_iterator, vertex_iterator> vertices() {
    return boost::vertices(g);
  }

  pair<edge_iterator, edge_iterator> edges() {
    return boost::edges(g);
  }

  pair<face_iterator, face_iterator> faces() {
    return make_pair(g[graph_bundle].begin(), g[graph_bundle].end());
  }

  edge_descriptor next(edge_descriptor he) {
    return g[he].next;
  }

  edge_descriptor prev(edge_descriptor he) {
    return g[he].prev;
  }

  class edge_circulator {
  public:
    edge_circulator(edge_descriptor he, const graph_type& g, std::function<edge_descriptor(edge_descriptor, const graph_type&)> vis)
      : cycled(false), he(he), cur(he), g(g), vis(vis) {}

    edge_circulator& operator=(edge_circulator other) {
      std::swap(*this, other);
      return *this;
    }

    operator bool() const {
      return !cycled;
    }

    edge_circulator& operator++() {
      cur = vis(cur, g);
      cycled = (cur == he);
      return (*this);
    }

    edge_circulator operator++(int) {
      cur = vis(cur, g);
      cycled = (cur == he);
      return (*this);
    }

    edge_descriptor operator*() { return cur; }

  private:
    bool cycled;
    edge_descriptor he;
    edge_descriptor cur;
    const graph_type& g;
    std::function<edge_descriptor(edge_descriptor, const graph_type&)> vis;
  };

  typedef edge_circulator edge_circulator_type;

  edge_circulator incident_edges(face_descriptor f) {
    return edge_circulator(halfedge(f), g,
                           [](edge_descriptor e, const graph_type& g) {
                             return g[e].next;
                           });
  }

  edge_circulator incident_edges(face_type& f) {
    return edge_circulator(f.he, g,
                           [](edge_descriptor e, const graph_type& g) {
                             return g[e].next;
                           });
  }

  edge_circulator incident_edges(vertex_descriptor v) {
    return edge_circulator(halfedge(v), g,
                           [](edge_descriptor e, const graph_type& g) {
                             return g[g[e].flip].next;
                           });
  }

  vertex_property& vertex(edge_descriptor he) {
    return g[g[he].v].prop;
  }

  pair<edge_descriptor, bool> edge(vertex_descriptor u, vertex_descriptor v) const {
    return boost::edge(u, v, g);
  }

  edge_descriptor halfedge(face_descriptor f) const {
    return g[graph_bundle][f].he;
  }

  edge_descriptor halfedge(vertex_descriptor v) const {
    return g[v].he;
  }

  static HalfEdgeDataStructure build(const vector<MeshLoader::face_t> &inFaces,
             const vector<MeshLoader::vert_t> &inVerts) {
    HalfEdgeDataStructure mesh;

    // add all vertices
    vector<vertex_descriptor> verts(inVerts.size());
    for(int i=0;i<inVerts.size();++i) {
      verts[i] = mesh.add_vertex(inVerts[i]);
    }
    cerr << "vertices added." << endl;

    // add all edges
    for(auto f : inFaces) {
      for(int j=0;j<f.v.size();++j) {
        int jn = (j+1)%f.v.size();
        auto u = verts[f.v[j]], v = verts[f.v[jn]];
        double w = glm::length(mesh[u] - mesh[v]);
        mesh.add_edge(u, v, w);
      }
    }
    cerr << "edges added." << endl;

    // add all faces
    vector<face_descriptor> faces(inFaces.size());
    for(int i=0;i<inFaces.size();++i) {
      auto& f = inFaces[i];
      auto u = verts[f.v[0]], v = verts[f.v[1]];
      auto edge_pair = mesh.edge(u, v);
      assert(edge_pair.second);
      faces[i] = mesh.add_face(edge_pair.first, FaceProperty());
      cerr << edge_pair.first << endl;
    }
    cerr << "faces added." << endl;

    // fix the half edge binding for each vertex
    vertex_iterator vit, vend;
    for(tie(vit, vend) = boost::vertices(mesh.g); vit != vend; ++vit) {
      mesh.g[*vit].he =  *(out_edges(*vit, mesh.g).first);
    }
    cerr << "half edge binding for vertices fixed." << endl;

    // fix the half edges
    // link the prev and next half edges, set the incident vertex and face
    for(int i=0;i<inFaces.size();++i) {
      auto& f = inFaces[i];
      for(int j=0;j<f.v.size();++j) {
        int jn = (j+1)>=f.v.size()?0:j+1;
        int jp = (j-1)<0?(j-1+f.v.size()):j-1;

        auto u = verts[f.v[j]], v = verts[f.v[jn]];
        auto edge_pair = mesh.edge(u, v);
        assert(edge_pair.second);
        mesh.g[edge_pair.first].v = u;
        mesh.g[edge_pair.first].f = faces[i];

        auto next_edge_pair = mesh.edge(verts[f.v[jn]], verts[f.v[jp]]);
        assert(next_edge_pair.second);
        mesh.g[edge_pair.first].next = next_edge_pair.first;

        auto prev_edge_pair = mesh.edge(verts[f.v[jp]], verts[f.v[j]]);
        assert(prev_edge_pair.second);
        mesh.g[edge_pair.first].prev = prev_edge_pair.first;

        auto flip_edge_pair = mesh.edge(v, u);
        assert(flip_edge_pair.second);
        mesh.g[edge_pair.first].flip = flip_edge_pair.first;

        cerr << &mesh.g[edge_pair.first] << ":: " << u << ", " << faces[i] << ", " << edge_pair.first << ": "
             << mesh.g[edge_pair.first].prev << "\t"
             << mesh.g[edge_pair.first].next << "\t"
             << mesh.g[edge_pair.first].flip << endl;
      }
    }

    return mesh;
  }

private:
  graph_type g; // the underlying graph
};

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty>
HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty>
build_half_edge_mesh(const vector<MeshLoader::face_t> &inFaces,
                     const vector<MeshLoader::vert_t> &inVerts) {
  typedef HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty> mesh_type;
  return mesh_type::build(inFaces, inVerts);
}

#endif // HDSMESH_H
