#include "meshloader.h"
#include "stringutils.h"

#include <fstream>
using namespace std;

void MeshLoader::clear() {
  verts.clear();
  faces.clear();
  texcoords.clear();
  normals.clear();

  verts.reserve(131076);
  normals.reserve(131076);
  texcoords.reserve(131076);
  faces.reserve(131076*2);
}

void MeshLoader::triangulate(){
  cout << "Triangulating the mesh ..." << endl;
  vector<face_t> newFaces;

  for(size_t i=0;i<faces.size();i++)
  {
    const face_t& Fi = faces[i];

    if( Fi.v.size() > 3 )
    {
      // triangulate this face

      for(size_t j=1;j<Fi.v.size()-1;j++)
      {
        face_t f;
        f.v.push_back(Fi.v[0]);
        f.v.push_back(Fi.v[j]);
        f.v.push_back(Fi.v[j+1]);

        f.t.push_back(Fi.t[0]);
        f.t.push_back(Fi.t[j]);
        f.t.push_back(Fi.t[j+1]);

        f.n.push_back(Fi.n[0]);
        f.n.push_back(Fi.n[j]);
        f.n.push_back(Fi.n[j+1]);

        newFaces.push_back(f);
      }
    }
    else
    {
      newFaces.push_back(Fi);
    }
  }

  faces = newFaces;

  triangulated = true;

  hasVertexNormal = false;
  hasVertexTexCoord = false;

  cout << "done.";
}

void MeshLoader::estimateNormals()
{
  if( hasVertexNormal )
  {
    cout << "already has vertex normal ..." << endl;
    // only estimate face normal
    for(size_t i=0;i<faces.size();i++)
    {
      glm::vec3 p0, p1, p2;

      p0 = verts[faces[i].v[0]];
      p1 = verts[faces[i].v[1]];
      p2 = verts[faces[i].v[2]];

      norm_t n = glm::cross(p0 - p1, p2 - p1);
      n = glm::normalize(n);

      faces[i].normal = n;
    }
  }
  else
  {
    normals.resize(verts.size());
    for(size_t i=0;i<verts.size();i++)
    normals[i] = norm_t(0, 0, 0);

    // for each face, compute its normal
    // add the contribution to its vertices
    for(size_t i=0;i<faces.size();i++)
    {

      glm::vec3 p0, p1, p2;

      p0 = verts[faces[i].v[0]];
      p1 = verts[faces[i].v[1]];
      p2 = verts[faces[i].v[2]];

      norm_t n = glm::cross(p0 - p1, p2 - p1);
      n = normalize(n);

      faces[i].normal = n;

      for(size_t j=0;j<faces[i].v.size();j++)
      {
        int pidx, nidx, idx;
        idx = j;
        pidx = j-1;
        if( pidx < 0 ) pidx += faces[i].v.size();
        nidx = j+1;
        if( nidx > faces[i].v.size() - 1 ) nidx -= faces[i].v.size();

        glm::vec3 vp, vc, vn;
        vp = verts[faces[i].v[pidx]];
        vc = verts[faces[i].v[idx]];
        vn = verts[faces[i].v[nidx]];

        glm::vec3 e1 = vp - vc, e2 = vn - vc;

        float theta = glm::dot(e1, e2) / (e1.length() * e2.length());

        normals[faces[i].v[idx]] += theta * n;
      }
    }

    for(size_t i=0;i<normals.size();i++)
    normals[i] = glm::normalize(normals[i]);
  }
}

istream& operator>>(istream& is, glm::vec3& v) {
  is >> v.x >> v.y >> v.z;
  return is;
}

istream& operator>>(istream& is, glm::vec2& v) {
  is >> v.x >> v.y;
  return is;
}

bool OBJLoader::load(const string& filename) {
  try{
    ifstream file(filename.c_str());
    if( !file ) {
      cerr << "Failed to open file " << filename << endl;
      return false;
    }

    clear();

    triangulated = true;

    while (file) {
      string line;
      getline(file, line);

      stringstream sline;
      sline << line;

      string identifier;
      sline >> identifier;

      //cout << identifier << endl;
      //cout << line << endl;

      if( identifier == "v" )
      {
        //cout << "vertex" << endl;
        vert_t p;
        sline >> p;
        //cout << p << endl;
        verts.push_back( p );
      }
      else if( identifier == "f" )
      {
        //cout << "face" << endl;
        face_t f;
        string vstr;
        int vidx, vtidx, vnidx;
        while( sline >> vstr )
        {
          stringlist vlist;
          /// obj file starts indexing vertices from 1

          vlist = split(vstr, "/");

          auto vit = vlist.begin();

          vidx = atoi((*vit).c_str());
          vit++;
          if( vidx < 0 ) vidx = -vidx;
          f.v.push_back(vidx - 1);

          if( vit != vlist.end() )
          {
            vtidx = atoi((*vit).c_str());
            vit++;
            if( vtidx < 0 ) vtidx = -vtidx;
            #undef max
            f.t.push_back(std::max(vtidx - 1, 0));

          }
          if( vit != vlist.end() )
          {
            vnidx = atoi((*vit).c_str());
            if( vnidx < 0 ) vnidx = -vnidx;
            f.n.push_back(std::max(vnidx - 1, 0));
          }
          //cout << vidx << ", ";
        }
        //cout << endl;

        triangulated &= (f.v.size() <= 3);

        faces.push_back(f);
      }
      else if( identifier == "vt" )
      {
        hasVertexTexCoord = true;
        //cout << "vertex texture" << endl;
        texcoord_t p;
        sline >> p;
        //cout << p << endl;
        texcoords.push_back( p );
      }
      else if( identifier == "vn" )
      {
        hasVertexNormal = true;
        //cout << "vertex normal" << endl;
        norm_t n;
        sline >> n;
        //cout << p << endl;
        normals.push_back( n );
      }
    }

    file.close();

    if(!triangulated) triangulate();

    cout << "finish loading file " << filename << endl;
    cout << "# faces = " << faces.size() << endl;
    cout << "# vertices = " << verts.size() << endl;

    return true;
  }
  catch( exception e )
  {
    cerr << e.what() << endl;
    return false;
  }
}
