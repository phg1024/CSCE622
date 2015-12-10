<div align="center">
<h3>CSCE 622: Generic Programming -- Final Project Report</h3>
<h4>Peihong Guo UIN: 421003404</h4>
</div>

### Project title: Half-edge Data Structure using BGL

#### User's Guide
Half-edge data structure is an important data structure for representing and manipulating 3D mesh data in computational geometry. A generic implementation of half-edge data structure could be used in various computational geometry algorithms, including mesh simplification, repair and constructive solid geometry, that rely on this data structure.

Excellent implementations of HDS are available in the public domain, among which are the [CGAL implementation](http://doc.cgal.org/latest/HalfedgeDS/index.html) and [OpenMesh library](http://www.openmesh.org/). However, one of the common issues with these specialized libraries is the relatively steep learning curve. The need to providing generic data structure and the tight coupling with many related components make the implementations in these libraries over-complicated such that users have to invest a significant amount of time studying the library before actually utilizing it to solve problems at hand. A light-weight, easy to use library is therefore needed in cases where efficiency is more important.

The main goal of this project is to implement a light-weight generic half-edge data structure using BGL. This an experimental effort towards implementing half-edge data structure using [boost graph library](http://www.boost.org/doc/libs/1_59_0/libs/graph/doc/table_of_contents.html) (BGL), where the library is designed as a wrapper for basic boost graphs to mimic half edge data structure. Theoretically, any graph offered in BGL could serve as the underlying graph for representing HDS. The performance of the resulting HDS is expected to vary significantly given different choice of basic graph.

The user of this library is assumed to have basic knowledge of BGL, and a basic background in computer graphics. Familiarity with the concept of half edge data structure would be a big help in understanding how this implementation works.

##### List of Sources
1. [Half edge data structure in CGAL](http://doc.cgal.org/latest/HalfedgeDS/index.html)
2. [Boost graph library](http://www.boost.org/doc/libs/1_59_0/libs/graph/doc/table_of_contents.html)
3. [OpenMesh library](http://www.openmesh.org/)
4. [Mesh simplification in CGAL]( http://doc.cgal.org/latest/Surface_mesh_simplification/)
5. [Mesh deformation]( http://www.cse.wustl.edu/~taoju/cse554/lectures/lect08_Deformation.pdf)

#### Reference Manual
See how to use the STL documentation for description of how, e.g., concepts are documented in the SGI STL documentation. Follow these guidelines as is appropriate for your project.

#### Design Document
Describe the most important design decisions. Why the interface is the way it is? Why were particular algorithms or data structures chosen?

One purpose of this section is to avoid extra work in the future. Unless the thinking behind various design decisions is documented, the same thinking process may have to be repeated later if the design is re-evaluated. In particular, documenting the reasons why other (inferior) choices were not selected is helpful.

#### Description and results of testing
Test for small cases, including all boundary cases you can think of, such as empty sequences, one-element sequences, graphs with no edges, graphs with more than one parallel edges, and so forth. Test for large cases as well; if possible, define an acceptance routine that checks correctness of result by some means.

If applicable, you should include timings (you want to make sure that the complexity guarantees that you documented in your reference manual are realistic). Test for large cases, measuring performance for large randomly generated inputs of different sizes. Document all necessary detail (hardware platform, compiler, optimization flags, …) to allow someone other than you to repeat the same measurements.

#### Source code

`hdsmesh.h`
```cpp
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
  using g = adjacency_list<listS, vecS, bidirectionalS, VP, EP, GP>;

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

  static HalfEdgeDataStructure build(const vector<MeshLoader::face_t> &inFaces,
             const vector<MeshLoader::vert_t> &inVerts) {
    HalfEdgeDataStructure mesh;

    // add all vertices
    vector<vertex_descriptor> verts(inVerts.size());
    for(size_t i=0;i<inVerts.size();++i) {
      verts[i] = mesh.add_vertex(inVerts[i]);
    }
    clog << "vertices added." << endl;

    // add all edges
    for(auto f : inFaces) {
      for(size_t j=0;j<f.v.size();++j) {
        int jn = (j+1)%f.v.size();
        auto u = verts[f.v[j]], v = verts[f.v[jn]];
        double w = glm::length(mesh[u] - mesh[v]);
        mesh.add_edge(u, v, w);
      }
    }
    clog << "edges added." << endl;

    // add all faces
    vector<face_descriptor> faces(inFaces.size());
    for(size_t i=0;i<inFaces.size();++i) {
      auto& f = inFaces[i];
      auto u = verts[f.v[0]], v = verts[f.v[1]];
      auto edge_pair = mesh.edge(u, v);
      assert(edge_pair.second);
      faces[i] = mesh.add_face(edge_pair.first, FaceProperty());
      clog << edge_pair.first << endl;
    }
    clog << "faces added." << endl;

    // fix the half edge binding for each vertex
    for(vertex_iterator vit = mesh.vertices(); vit; ++vit) {
      clog << *vit << endl;
      mesh.g[*vit].he =  *(out_edges(*vit, mesh.g).first);
    }
    clog << "half edge binding for vertices fixed." << endl;

    // fix the half edges
    // link the prev and next half edges, set the incident vertex and face
    for(size_t i=0;i<inFaces.size();++i) {
      auto& f = inFaces[i];
      for(int j=0;j<static_cast<int>(f.v.size());++j) {
        int jn = (j+1)>=static_cast<int>(f.v.size())?0:j+1;
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

        clog << &mesh.g[edge_pair.first] << ":: " << u << ", " << *faces[i] << ", " << edge_pair.first << ": "
             << mesh.g[edge_pair.first].prev << "\t"
             << mesh.g[edge_pair.first].next << "\t"
             << mesh.g[edge_pair.first].flip << endl;
      }
    }

    return mesh;
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
  hds = mesh_type::build(loader.getFaces(), loader.getVerts());
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
```

`iterators.h`
```cpp
#ifndef ITERATORS_H
#define ITERATORS_H

template <typename GraphType, class Visitor>
class Iterator {
public:
  typedef GraphType graph_type;
  typedef typename Visitor::iterator_type iterator_type;

  Iterator() : g(nullptr) {}
  Iterator(iterator_type cur, iterator_type last, const graph_type* g)
    : cur(cur), last(last), g(g) {}

  bool operator==(const Iterator& it) const {
    return cur == it.cur;
  }

  bool operator!=(const Iterator& it) const {
    return !(*this == it);
  }

  operator bool() {
    return cur != last;
  }

  Iterator& operator++() {
    ++cur;
    while(cur != last && !visitor.valid(cur, g)) ++cur;
    return (*this);
  }

  Iterator operator++(int) {
    Iterator cpy = *this;
    ++cur;
    while(cur != last && !!visitor.valid(cur, g)) ++cur;
    return cpy;
  }

  typename Visitor::value_type operator*() {
    return visitor(cur, g);
  }

private:
  iterator_type cur, last;
  const graph_type* g;
  Visitor visitor;
};

#endif  // ITERATORS_H
```

`circulators.h`
```cpp
#ifndef CIRCULATORS_H
#define CIRCULATORS_H

template <class MeshType, typename Visitor,
          typename CirculationPolicy=std::function<typename MeshType::edge_descriptor(typename MeshType::edge_descriptor, const typename MeshType::graph_type&)>>
class EdgeCirculator {
public:
  typedef MeshType mesh_type;
  typedef typename MeshType::graph_type graph_type;
  typedef typename MeshType::edge_descriptor edge_descriptor;
  EdgeCirculator(edge_descriptor he, const graph_type& g, CirculationPolicy policy)
    : cycled(false), he(he), cur(he), g(g), policy(policy) {}

  EdgeCirculator& operator=(EdgeCirculator other) {
    std::swap(*this, other);
    return *this;
  }

  operator bool() const {
    return !cycled;
  }

  EdgeCirculator& operator++() {
    cur = policy(cur, g);
    cycled = (cur == he);
    return (*this);
  }

  EdgeCirculator operator++(int) {
    EdgeCirculator cpy = *this;
    cur = policy(cur, g);
    cycled = (cur == he);
    return cpy;
  }

  typename Visitor::value_type operator*() { return visitor(cur, g); }

private:
  bool cycled;
  edge_descriptor he;
  edge_descriptor cur;
  const graph_type& g;
  Visitor visitor;
  CirculationPolicy policy;
};

#endif  // CIRCULATORS_H
```

`taggedvalue.h`
```
#ifndef TAGGED_VALUE_H
#define TAGGED_VALUE_H

#include <functional>

template <typename ValueType, typename TagType>
struct TaggedValue {
  typedef ValueType value_t;
  typedef TagType tag_t;

  TaggedValue() = default;
  TaggedValue(const ValueType& value_in) : value(value_in) {}

  ValueType& operator*() { return value; }
  const ValueType& operator*() const { return value; }

  bool operator==(const TaggedValue& other) const {
    return value == other.value;
  }

  template <typename VT, typename TT>
  friend ostream& operator<<(ostream& os, const TaggedValue<VT, TT>& val);

  ValueType value;
};

template <typename VT, typename TT>
ostream& operator<<(ostream& os, const TaggedValue<VT, TT>& val) {
  os << *val;
  return os;
}

template <typename TaggedValue>
struct TaggedValueHasher {
  std::size_t operator()(const TaggedValue& value) const {
    return std::hash<typename TaggedValue::value_t>()(*value);
  }
};

#endif  // TAGGED_VALUE_H
```

```

```
