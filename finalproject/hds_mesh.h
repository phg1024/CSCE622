#ifndef HDS_MESH_H
#define HDS_MESH_H

#include "meshloader.h"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>

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

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty>
struct DefaultHDSTraits {
  typedef int halfedge_descriptor;
  typedef unsigned int face_descriptor;

  struct AugmentedVertexProperty {
    AugmentedVertexProperty() : he(-1) {}
    AugmentedVertexProperty(VertexProperty vp) : prop(vp), he(-1) {}
    VertexProperty prop;
    halfedge_descriptor he;
  };

  struct AugmentedEdgeProperty {
    AugmentedEdgeProperty() { he[0] = he[1] = -1; }
    AugmentedEdgeProperty(EdgeProperty ep) : prop(ep) {
      he[0] = he[1] = -1;
    }
    EdgeProperty prop;
    halfedge_descriptor he[2];
  };

  struct AugmentedFaceProperty {
    AugmentedFaceProperty() : he(-1) {}
    AugmentedFaceProperty(FaceProperty fp) : prop(fp), he(-1) {}
    FaceProperty prop;
    halfedge_descriptor he;
  };

  typedef AugmentedVertexProperty vertex_property;
  typedef AugmentedEdgeProperty edge_property;
  typedef AugmentedFaceProperty face_property;
};

template <typename VertexProperty, typename EdgeProperty, typename FaceProperty,
          typename HalfEdgeDataStructureTraits=DefaultHDSTraits<VertexProperty, EdgeProperty, FaceProperty>,
          typename G = adjacency_list<listS, vecS, undirectedS,
                                      typename DefaultHDSTraits<VertexProperty, EdgeProperty, FaceProperty>::vertex_property,
                                      typename DefaultHDSTraits<VertexProperty, EdgeProperty, FaceProperty>::edge_property>>
class HalfEdgeDataStructure : public G {
public:
  HalfEdgeDataStructure() {}

  typedef typename graph_traits<G>::vertex_descriptor vertex_descriptor;
  typedef typename graph_traits<G>::edge_descriptor edge_descriptor;
  typedef typename HalfEdgeDataStructureTraits::halfedge_descriptor halfedge_descriptor;
  typedef typename HalfEdgeDataStructureTraits::face_descriptor face_descriptor;

  typedef typename HalfEdgeDataStructureTraits::vertex_property vertex_property;
  typedef typename HalfEdgeDataStructureTraits::edge_property edge_property;
  typedef typename HalfEdgeDataStructureTraits::face_property face_property;

  struct HalfEdge {
    vertex_descriptor v;
    edge_descriptor e;
    face_descriptor f;
    halfedge_descriptor prev;
    halfedge_descriptor flip;
  };

  size_t halfedge_count() const {
    return halfedge_set.size();
  }

  // iterators

  // Use defaults:
  //  1. vertex iterators
  //  2. edge iterators

  // Use friend functions
  //  3. halfedge iterators
  typedef typename std::vector<halfedge_descriptor>::iterator halfedge_iterator;
  typedef typename std::vector<halfedge_descriptor>::const_iterator const_halfedge_iterator;

  std::pair<halfedge_iterator, halfedge_iterator> halfedges() {
    return std::make_pair(halfedge_set.begin(), halfedge_set.end());
  }
  std::pair<const_halfedge_iterator, const_halfedge_iterator> halfedges() const {
    return std::make_pair(halfedge_set.cbegin(), halfedge_set.cend());
  }

  //  4. face iterator
  typedef typename std::vector<face_descriptor>::iterator face_iterator;
  typedef typename std::vector<face_descriptor>::const_iterator const_face_iterator;

  std::pair<face_iterator, face_iterator> faces() {
    return std::make_pair(face_set.begin(), face_set.end());
  }
  std::pair<const_face_iterator, const_face_iterator> faces() const {
    return std::make_pair(face_set.cbegin(), face_set.cend());
  }

  //
  edge_descriptor edge(halfedge_descriptor h) {
    throw "Not defined.";
  }
  halfedge_descriptor halfedge(edge_descriptor e) {
    return (*this)[e].he[0];
  }
  halfedge_descriptor halfedge(vertex_descriptor v) {
    return (*this)[v].he;
  }
  halfedge_descriptor halfedge(face_descriptor f) {
    return face_property_map[f].he;
  }

  // property accessors
  face_property& face(face_descriptor f) {
    return face_property_map[f];
  }
  const face_property& face(face_descriptor f) const {
    return face_property_map[f];
  }


  HalfEdge& halfedge(halfedge_descriptor f) {
    return halfedge_map[f];
  }

  const HalfEdge& halfedge(halfedge_descriptor f) const {
    return halfedge_map[f];
  }

  // mesh modifiers
  vertex_descriptor add_vertex(vertex_property vp) {
    return boost::add_vertex(vertex_property(vp), (*this));
  }

  face_descriptor add_face() {
    face_set.push_back(face_set.size());
    return face_set.back();
  }

  halfedge_descriptor add_halfedge() {
    halfedge_set.push_back(halfedge_set.size());
    return halfedge_set.size();
  }

private:
  std::vector<face_descriptor> face_set;
  std::unordered_map<face_descriptor, typename HalfEdgeDataStructureTraits::face_property> face_property_map;

  std::vector<halfedge_descriptor> halfedge_set;
  std::unordered_map<halfedge_descriptor, HalfEdge> halfedge_map;
};

struct NullType {};

template <typename VertexType, typename EdgeType, typename FaceType>
HalfEdgeDataStructure<VertexType, EdgeType, FaceType> build_half_edge_mesh(
  const vector<MeshLoader::face_t> &inFaces,
  const vector<MeshLoader::vert_t> &inVerts) {
  typedef HalfEdgeDataStructure<VertexType, EdgeType, FaceType> mesh_t;
  mesh_t thismesh;

  std::cout << "building the half edge mesh ..." << std::endl;

  size_t vertsCount = inVerts.size();
  size_t facesCount = inFaces.size();
  size_t curFaceCount = facesCount;
  size_t heCount = 0;

  for(size_t i=0;i<inFaces.size();i++)
    heCount += inFaces[i].v.size();

  std::cout << "he count = " << heCount << std::endl;

  vector<typename mesh_t::vertex_descriptor> verts(vertsCount);
  for(size_t i=0;i<vertsCount;i++) {
    verts[i] = thismesh.add_vertex(inVerts[i]);
  }
  std::cout << "verts added" << std::endl;

  for(size_t i=0;i<facesCount;i++) {
    thismesh.add_face();
  }
  std::cout << "faces added" << std::endl;

  std::map<std::pair<typename mesh_t::vertex_descriptor,
                     typename mesh_t::vertex_descriptor>,
           typename mesh_t::halfedge_descriptor> heMap;
  heMap.clear();

  typename mesh_t::face_iterator fbegin, fend;
  std::tie(fbegin, fend) = thismesh.faces();
  for(auto fit = fbegin; fit != fend; ++fit) {
    auto& Fi = inFaces[*fit];
    auto& curFace = thismesh[*fit];

    vector<typename mesh_t::halfedge_descriptor> hes(Fi.v.size());
    for(size_t j=0;j<Fi.v.size();j++) {
      typename mesh_t::halfedge_descriptor curHe = thismesh.add_halfedge();
      hes[j] = curHe;
      auto curVert = verts[Fi.v[j]];
      auto& curHeProp = thismesh.halfedge(curHe);
      curHeProp.v = curVert;
      curHeProp.f = *fit;

      auto& curVertProp = thismesh[curVert];
      if(curVertProp.he == -1)
        curVertProp.he = curHe;
    }

    // link the half edge of the face
    for(int j=0;j<Fi.v.size();j++)
    {
      int jp = j-1;
      if( jp < 0 ) jp += Fi.v.size();
      int jn = j+1;
      if( jn >= Fi.v.size() ) jn -= Fi.v.size();

      auto curHe = hes[j];
      auto& curHeProp = thismesh.halfedge(curHe);
      curHeProp.prev = hes[jp];

      auto vj = verts[Fi.v[j]];
      auto vjn = verts[Fi.v[jn]];

      std::pair<typename mesh_t::vertex_descriptor,
                typename mesh_t::vertex_descriptor> vPair = std::make_pair(vj, vjn);

      if( heMap.find(vPair) == heMap.end() )
      {
        heMap[vPair] = curHe;
      }
    }

    curFace.he = hes[0];
  }

  std::set<std::pair<typename mesh_t::halfedge_descriptor,
                     typename mesh_t::halfedge_descriptor>> pairedHESet;

  // for each half edge, find its flip
  for(auto heit = heMap.begin(); heit != heMap.end(); heit++)
  {
    typename mesh_t::vertex_descriptor from, to;

    auto hePair = (*heit).first;

    if( pairedHESet.find(hePair) == pairedHESet.end() )
    {
      std::tie(from, to) = hePair;

      auto invPair = std::make_pair(to, from);

      auto invItem = heMap.find(invPair);

      if( invItem != heMap.end() )
      {
        auto he = (*heit).second;
        auto hef = (*invItem).second;

        thismesh.halfedge(he).flip = hef;
        thismesh.halfedge(hef).flip = he;
      }

      pairedHESet.insert( hePair );
      pairedHESet.insert( invPair );
    }
  }

  std::cout << "finished building halfedge structure." << std::endl;
  std::cout << "halfedge count = " << thismesh.halfedge_count() << std::endl;

  return thismesh;
}

#endif
