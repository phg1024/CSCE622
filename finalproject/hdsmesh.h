#ifndef HDSMESH_H
#define HDSMESH_H

#include "meshloader.h"
#include "taggedvalue.h"
#include "circulators.h"
#include "iterators.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>
using namespace std;

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/property_map/property_map.hpp>
#include <boost/timer/timer.hpp>
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
  HDSVertex() : valid(true) {}
  HDSVertex(const VertexProperty& vp) : valid(true), prop(vp) {}

  bool valid;
  HalfedgeDescriptor he;
  VertexProperty prop;
};

template <typename HalfedgeDescriptor, typename VertexDescriptor,
          typename FaceDescriptor, typename EdgeProperty>
struct HDSEdge {
  HDSEdge() : valid(true) {}
  HDSEdge(const EdgeProperty& ep) : valid(true), prop(ep) {}

  bool valid;
  VertexDescriptor v;
  FaceDescriptor f;
  HalfedgeDescriptor flip, prev, next;
  EdgeProperty prop;
};

template <typename HalfedgeDescriptor, typename FaceProperty>
struct HDSFace {
  HDSFace() : valid(true) {}
  HDSFace(const FaceProperty& fp) : valid(true), prop(fp) {}
  HDSFace(HalfedgeDescriptor he, const FaceProperty& fp) : valid(true), he(he), prop(fp) {}

  bool valid;
  HalfedgeDescriptor he;
  FaceProperty prop;
};

struct NullType {};

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty>
struct DefaultHalfedgeDataStructureTraits {

  template <typename VP, typename EP, typename GP>
  using g = adjacency_list<listS, listS, bidirectionalS, VP, EP, GP>;

  typedef graph_traits<g<NullType, NullType, NullType>>::vertex_descriptor vertex_descriptor;
  typedef graph_traits<g<NullType, NullType, NullType>>::edge_descriptor edge_descriptor;
  typedef graph_traits<g<NullType, NullType, NullType>>::edge_descriptor halfedge_descriptor;

  struct face_descriptor_tag {};
  typedef TaggedValue<unsigned int, face_descriptor_tag>  face_descriptor;
  typedef TaggedValueHasher<face_descriptor> face_descriptor_hasher;

  typedef HDSVertex<edge_descriptor, VertexProperty> vertex_type;
  typedef HDSEdge<edge_descriptor, vertex_descriptor, face_descriptor, EdgeProperty> edge_type;
  typedef HDSFace<edge_descriptor, FaceProperty> face_type;

  typedef std::unordered_map<face_descriptor, unsigned int, face_descriptor_hasher> face_index_map_t;
  typedef std::vector<face_descriptor> face_descriptor_set_t;
  typedef std::vector<face_type> face_set_t;
  struct face_properties {
    face_properties() : face_index(0) {}
    unsigned int face_index;
    face_descriptor_set_t face_descriptors;
    face_index_map_t face_index_map;
    face_set_t face_set;
  };

  typedef VertexProperty vertex_property_type;
  typedef EdgeProperty edge_property_type;
  typedef FaceProperty face_property_type;

  typedef g<vertex_type, edge_type, face_properties> graph_type;

  typedef typename graph_traits<graph_type>::vertex_iterator vertex_iterator;
  typedef typename graph_traits<graph_type>::edge_iterator edge_iterator;
  typedef typename graph_traits<graph_type>::edge_iterator halfedge_iterator;
  typedef typename face_descriptor_set_t::iterator face_iterator;
  typedef typename face_descriptor_set_t::const_iterator const_face_iterator;
};

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty,
          typename HalfEdgeDataStructureTraits=DefaultHalfedgeDataStructureTraits<VertexProperty, EdgeProperty, FaceProperty>>
class HalfEdgeDataStructure {
public:
  HalfEdgeDataStructure() {}

  typedef HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty, HalfEdgeDataStructureTraits> self;

  typedef typename HalfEdgeDataStructureTraits::graph_type graph_type;
  typedef HalfEdgeDataStructureTraits traits;

  typedef typename graph_type::directed_category directed_category;
  typedef typename graph_type::edge_parallel_category edge_parallel_category;
  typedef typename graph_type::traversal_category traversal_category;

  typedef typename HalfEdgeDataStructureTraits::vertex_descriptor vertex_descriptor;
  typedef typename HalfEdgeDataStructureTraits::edge_descriptor edge_descriptor;
  typedef typename HalfEdgeDataStructureTraits::halfedge_descriptor halfedge_descriptor;
  typedef typename HalfEdgeDataStructureTraits::face_descriptor face_descriptor;

  typedef typename HalfEdgeDataStructureTraits::face_descriptor_hasher face_descriptor_hasher;

  typedef typename HalfEdgeDataStructureTraits::vertex_type vertex_type;
  typedef typename HalfEdgeDataStructureTraits::edge_type edge_type;
  typedef typename HalfEdgeDataStructureTraits::face_type face_type;

  typedef size_t vertices_size_type;
  typedef size_t edges_size_type;
  typedef size_t degree_size_type;

  typedef typename HalfEdgeDataStructureTraits::vertex_property_type vertex_property_type;
  typedef typename HalfEdgeDataStructureTraits::edge_property_type edge_property_type;
  typedef typename HalfEdgeDataStructureTraits::face_property_type face_property_type;

  template <typename IteratorType, typename ValueType>
  struct default_visitor {
    typedef IteratorType iterator_type;
    typedef ValueType value_type;
    value_type operator()(iterator_type e, const graph_type*) { return *e; }
    bool valid(iterator_type e, const graph_type* g) const {
      return (*g)[*e].valid;
    }
  };
  typedef default_visitor<typename HalfEdgeDataStructureTraits::vertex_iterator, vertex_descriptor> default_vertex_visitor;
  typedef Iterator<graph_type, default_vertex_visitor> vertex_iterator;
  typedef const vertex_iterator const_vertex_iterator;

  typedef default_visitor<typename HalfEdgeDataStructureTraits::edge_iterator, edge_descriptor> default_edge_visitor;
  typedef Iterator<graph_type, default_visitor<typename HalfEdgeDataStructureTraits::edge_iterator, edge_descriptor>> edge_iterator;
  typedef const edge_iterator const_edge_iterator;

  template <typename IteratorType>
  struct default_face_visitor {
    typedef IteratorType iterator_type;
    typedef face_descriptor value_type;
    face_descriptor operator()(iterator_type it, const graph_type* g) {
      return *it;
    }
    bool valid(iterator_type it, const graph_type* g) {
      unsigned int idx = (*g)[graph_bundle].face_index_map.at(*it);
      return (*g)[graph_bundle].face_set.at(idx).valid;
    }
  };
  typedef Iterator<graph_type, default_face_visitor<typename HalfEdgeDataStructureTraits::face_iterator>> face_iterator;
  typedef Iterator<graph_type, default_face_visitor<typename HalfEdgeDataStructureTraits::const_face_iterator>> const_face_iterator;

  size_t size_of_vertices() const { return boost::num_vertices(g); }
  size_t size_of_halfedges() const { return boost::num_edges(g); }
  size_t size_of_facets() const { return g[graph_bundle].face_index_map.size(); }

  vertex_descriptor add_vertex(const VertexProperty& vp) {
    return boost::add_vertex(vertex_type(vp), g);
  }

  void remove_vertex(vertex_descriptor v) {
    g[v].valid = false;
  }

  pair<edge_descriptor, bool> add_edge(vertex_descriptor u, vertex_descriptor v,
                                       const EdgeProperty& ep) {
    return boost::add_edge(u, v, edge_type(ep), g);
  }

  void remove_edge(edge_descriptor e) {
    g[e].valid = false;
  }

  face_descriptor add_face(edge_descriptor e, const FaceProperty& fp) {
    face_descriptor descriptor{g[graph_bundle].face_index};
    ++g[graph_bundle].face_index;
    g[graph_bundle].face_set.push_back(face_type(e, fp));
    g[graph_bundle].face_descriptors.push_back(descriptor);
    g[graph_bundle].face_index_map[descriptor] = g[graph_bundle].face_set.size() - 1;
    return descriptor;
  }

  void remove_face(face_descriptor f) {
    unsigned int idx = g[graph_bundle].face_index_map.at(f);
    g[graph_bundle].face_set[idx].valid = false;
  }

  vertex_property_type& operator[](vertex_descriptor v) {
    return g[v].prop;
  }
  const vertex_property_type& operator[](vertex_descriptor v) const {
    return g[v].prop;
  }

  edge_property_type& operator[](edge_descriptor e) {
    return g[e].prop;
  }
  const edge_property_type& operator[](edge_descriptor e) const {
    return g[e].prop;
  }

  face_property_type& operator[](face_descriptor f) {
    unsigned int idx = g[graph_bundle].face_index_map.at(f);
    return g[graph_bundle].face_set.at(idx).prop;
  }

  const face_property_type& operator[](face_descriptor f) const {
    unsigned int idx = g[graph_bundle].face_index_map.at(f);
    return g[graph_bundle].face_set.at(idx).prop;
  }

  vertex_iterator vertices() {
    auto p = boost::vertices(g);
    while(p.first != p.second && !g[*p.first].valid) ++p.first;
    return vertex_iterator(p.first, p.second, &g);
  }

  const_vertex_iterator vertices() const {
    auto p = boost::vertices(g);
    while(p.first != p.second && !g[*p.first].valid) ++p.first;
    return vertex_iterator(p.first, p.second, &g);
  }

  edge_iterator edges() {
    auto p = boost::edges(g);
    while(p.first != p.second && !g[*p.first].valid) ++p.first;
    return edge_iterator(p.first, p.second, &g);
  }

  const_edge_iterator edges() const {
    auto p = boost::edges(g);
    while(p.first != p.second && !g[*p.first].valid) ++p.first;
    return edge_iterator(p.first, p.second, &g);
  }

  face_iterator faces() {
    auto fit = g[graph_bundle].face_descriptors.begin();
    auto fend = g[graph_bundle].face_descriptors.end();
    while(fit != fend && !g[graph_bundle].face_set[g[graph_bundle].face_index_map.at(*fit)].valid) ++fit;
    return face_iterator(fit, fend, &g);
  }

  const_face_iterator faces() const {
    auto fit = g[graph_bundle].face_descriptors.cbegin();
    auto fend = g[graph_bundle].face_descriptors.cend();
    while(fit != fend && !g[graph_bundle].face_set[g[graph_bundle].face_index_map.at(*fit)].valid) ++fit;
    return const_face_iterator(fit, fend, &g);
  }

  edge_descriptor next(edge_descriptor he) {
    return g[he].next;
  }

  edge_descriptor prev(edge_descriptor he) {
    return g[he].prev;
  }

  struct edge_visitor {
    typedef edge_descriptor value_type;
    edge_descriptor operator()(edge_descriptor e, const graph_type&) { return e; }
  };
  typedef EdgeCirculator<self, edge_visitor> edge_circulator;

  edge_circulator incident_edges(face_descriptor f) const {
    return edge_circulator(halfedge(f), g,
                           [](edge_descriptor e, const graph_type& g) {
                             return g[e].next;
                           });
  }

  edge_circulator incident_edges(vertex_descriptor v) const {
    return edge_circulator(halfedge(v), g,
                           [](edge_descriptor e, const graph_type& g) {
                             return g[g[e].flip].next;
                           });
  }

  struct face_visitor {
    typedef face_descriptor value_type;
    face_descriptor operator()(edge_descriptor e, const graph_type& g) {
      return g[e].f;
    }
  };
  typedef EdgeCirculator<self, face_visitor> face_circulator;

  face_circulator incident_faces(vertex_descriptor v) const {
    return face_circulator(halfedge(v), g,
                           [](edge_descriptor e, const graph_type& g) {
                             return g[g[e].flip].next;
                           });
  }

  struct vertex_visitor {
    typedef vertex_descriptor value_type;
    vertex_descriptor operator()(edge_descriptor e, const graph_type& g) {
      return g[e].v;
    }
  };
  typedef EdgeCirculator<self, vertex_visitor> vertex_circulator;

  vertex_circulator neighbor_vertices(vertex_descriptor v) const {
    return vertex_circulator(flip(halfedge(v)), g,
                             [](edge_descriptor e, const graph_type& g) {
                               return g[g[e].next].flip;
                             });
  }

  vertex_circulator incident_vertices(face_descriptor f) const {
    return vertex_circulator(halfedge(f), g,
                             [](edge_descriptor e, const graph_type& g) {
                               return g[e].next;
                             });
  }

  vertex_property_type& vertex(edge_descriptor he) {
    return g[g[he].v].prop;
  }

  const vertex_property_type& vertex(edge_descriptor he) const {
    return g[g[he].v].prop;
  }

  pair<edge_descriptor, bool> edge(vertex_descriptor u, vertex_descriptor v) const {
    return boost::edge(u, v, g);
  }

  bool collapse(vertex_descriptor u, vertex_descriptor v) {
    edge_descriptor e;
    bool exists;
    tie(e, exists) = edge(u, v);
    if(exists) {
      edge_descriptor ef = flip(e);
    } else return false;
  }

  edge_descriptor halfedge(face_descriptor f) {
    unsigned int idx = g[graph_bundle].face_index_map[f];
    return g[graph_bundle].face_set[idx].he;
  }

  edge_descriptor halfedge(face_descriptor f) const {
    unsigned int idx = g[graph_bundle].face_index_map.at(f);
    return g[graph_bundle].face_set[idx].he;
  }

  edge_descriptor halfedge(vertex_descriptor v) {
    return g[v].he;
  }

  edge_descriptor halfedge(vertex_descriptor v) const {
    return g[v].he;
  }

  edge_descriptor flip(edge_descriptor e) {
    return g[e].flip;
  }

  edge_descriptor flip(edge_descriptor e) const {
    return g[e].flip;
  }

  static void build(const vector<MeshLoader::face_t> &inFaces,
             const vector<MeshLoader::vert_t> &inVerts,
             HalfEdgeDataStructure& mesh) {
    boost::timer::auto_cpu_timer t("hds built in %w seconds.\n");

    // add all vertices
    vector<vertex_descriptor> verts(inVerts.size());
    for(size_t i=0;i<inVerts.size();++i) {
      verts[i] = mesh.add_vertex(inVerts[i]);
    }
    //clog << "vertices added." << endl;

    // add all edges
    for(auto f : inFaces) {
      for(size_t j=0;j<f.v.size();++j) {
        int jn = (j+1)%f.v.size();
        auto u = verts[f.v[j]], v = verts[f.v[jn]];
        mesh.add_edge(u, v, edge_property_type());
      }
    }
    //clog << "edges added." << endl;

    // add all faces
    vector<face_descriptor> faces(inFaces.size());
    for(size_t i=0;i<inFaces.size();++i) {
      auto& f = inFaces[i];
      auto u = verts[f.v[0]], v = verts[f.v[1]];
      auto edge_pair = mesh.edge(u, v);
      assert(edge_pair.second);
      faces[i] = mesh.add_face(edge_pair.first, FaceProperty());
      //clog << edge_pair.first << endl;
    }
    //clog << "faces added." << endl;

    // fix the half edge binding for each vertex
    for(vertex_iterator vit = mesh.vertices(); vit; ++vit) {
      //clog << *vit << endl;
      mesh.g[*vit].he =  *(out_edges(*vit, mesh.g).first);
    }
    //clog << "half edge binding for vertices fixed." << endl;

    // fix the half edges
    // link the prev and next half edges, set the incident vertex and face
    for(size_t i=0;i<inFaces.size();++i) {
      auto& f = inFaces[i];
      for(int j=0;j<static_cast<int>(f.v.size());++j) {
        int jn = (j+1)>=static_cast<int>(f.v.size())?0:j+1;
        int jp = (j-1)<0?(j-1+f.v.size()):j-1;

        auto vj = verts[f.v[j]], vjn = verts[f.v[jn]], vjp = verts[f.v[jp]];

        auto edge_pair = mesh.edge(vj, vjn);
        assert(edge_pair.second);
        mesh.g[edge_pair.first].v = vj;
        mesh.g[edge_pair.first].f = faces[i];

        auto next_edge_pair = mesh.edge(vjn, vjp);
        assert(next_edge_pair.second);
        mesh.g[edge_pair.first].next = next_edge_pair.first;

        auto prev_edge_pair = mesh.edge(vjp, vj);
        assert(prev_edge_pair.second);
        mesh.g[edge_pair.first].prev = prev_edge_pair.first;

        auto flip_edge_pair = mesh.edge(vjn, vj);
        assert(flip_edge_pair.second);
        mesh.g[edge_pair.first].flip = flip_edge_pair.first;

        //clog << &mesh.g[edge_pair.first] << ":: " << u << ", " << *faces[i] << ", " << edge_pair.first << ": "
        //     << mesh.g[edge_pair.first].prev << "\t"
        //     << mesh.g[edge_pair.first].next << "\t"
        //     << mesh.g[edge_pair.first].flip << endl;
      }
    }
  }

  template <typename VP, typename EP, typename FP>
  friend istream& operator>>(istream& is, HalfEdgeDataStructure<VP, EP, FP>& hds);

  template <typename VP, typename EP, typename FP>
  friend ostream& operator<<(ostream& os, const HalfEdgeDataStructure<VP, EP, FP>& hds);

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

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty>
istream& operator>>(istream& is, HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty>& hds) {
  typedef HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty> mesh_type;
  OBJLoader loader(is);
  mesh_type::build(loader.getFaces(), loader.getVerts(), hds);
  return is;
}

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty>
ostream& operator<<(ostream& os, const HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty>& hds) {
  typedef HalfEdgeDataStructure<VertexProperty, EdgeProperty, FaceProperty> mesh_t;
  typedef typename mesh_t::edge_circulator edge_circulator;
  typedef typename mesh_t::face_circulator face_circulator;
  typedef typename mesh_t::vertex_circulator vertex_circulator;

  os << hds.size_of_vertices() << " vertices: " << endl;
  for(auto it = hds.vertices(); it; ++it) {
    os << hds[*it] << "; ";
  }
  os << endl;
  os << hds.size_of_halfedges() << " edges: " << endl;
  for(auto it = hds.edges(); it; ++it) {
    os << *it << "; ";
  }
  os << endl;

  os << "faces circulation, incident edges: " << endl;
  for(auto fit = hds.faces(); fit; ++fit) {
    edge_circulator circ = hds.incident_edges(*fit);
    os << "face " << *fit << ": ";
    while(circ) {
      os << *circ << "; ";
      circ++;
    }
    os << endl;
  }

  os << "faces circulation, incident vertices: " << endl;
  for(auto fit = hds.faces(); fit; ++fit) {
    vertex_circulator circ = hds.incident_vertices(*fit);
    os << "face " << *fit << ": ";
    while(circ) {
      os << *circ << "; ";
      circ++;
    }
    os << endl;
  }

  os << "vertex circulation, incident edges: " << endl;
  for(auto vit = hds.vertices(); vit; ++vit) {
    edge_circulator circ = hds.incident_edges(*vit);
    os << "vertex " << *vit << ": ";
    while(circ) {
      os << *circ << "; ";
      ++circ;
    }
    os << endl;
  }

  os << "vertex circulation, incident faces: " << endl;
  for(auto vit = hds.vertices(); vit; ++vit) {
    face_circulator circ = hds.incident_faces(*vit);
    os << "vertex " << *vit << ": ";
    while(circ) {
      os << (*circ) << "; ";
      ++circ;
    }
    os << endl;
  }

  os << "vertex circulation, neighboring vertices: " << endl;
  for(auto vit = hds.vertices(); vit; ++vit) {
    vertex_circulator circ = hds.neighbor_vertices(*vit);
    os << "vertex " << *vit << ": ";
    while(circ) {
      os << (*circ) << "; ";
      ++circ;
    }
    os << endl;
  }
  return os;
}

#endif // HDSMESH_H
