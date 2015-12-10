#ifndef MESHLOADER_H
#define MESHLOADER_H

#include <iostream>
#include <string>
#include <vector>
using namespace std;

#include "glm/glm.hpp"

class MeshLoader
{
public:
  typedef glm::vec3 vert_t;
  struct face_t {
    face_t() {
      // at least 3 vertices
      v.reserve(3);
    }
    vector<int> v, n, t;
    glm::vec3 normal;
  };
  typedef glm::vec2 texcoord_t;
  typedef glm::vec3 norm_t;

  virtual bool load(const string& filename) = 0;

  const vector<vert_t>& getVerts() const { return verts; }
  const vector<face_t>& getFaces() const { return faces; }
  const vector<norm_t>& getNormals() const { return normals; }
  const vector<texcoord_t>& getTexcoords() const { return texcoords; }

protected:

  bool triangulated;
  bool hasVertexTexCoord;
  bool hasVertexNormal;
  vector<vert_t> verts;
  vector<face_t> faces;
  vector<texcoord_t> texcoords;
  vector<norm_t> normals;

protected:
  void clear();
  void triangulate();
  void estimateNormals();
};

class OBJLoader : public MeshLoader
{
public:
  OBJLoader() {}
  OBJLoader(istream& is);
  OBJLoader(const string& filename);

  virtual bool load(const string& filename);

private:
  bool LoadFromStream(istream& is);
};


#endif // MESHLOADER_H
