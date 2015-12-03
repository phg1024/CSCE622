#ifndef MESHLOADER_H
#define MESHLOADER_H

#include <string>
#include <vector>
using std::vector;
using std::string;

#include "glm/glm.hpp"

class MeshLoader
{
public:
    typedef glm::vec3 vert_t;
    struct face_t {
        face_t() {
            // at least 3 vertices
            v.reserve(8);
            n.reserve(8);
            t.reserve(8);
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
    virtual bool load(const string& filename);
};


#endif // MESHLOADER_H
