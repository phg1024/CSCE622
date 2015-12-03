#include "meshloader.h"
#include "hds_mesh.h"

#include <GL/glut.h>

#include <iostream>
#include <string>
using namespace std;

OBJLoader loader;

ostream& operator<<(ostream& os, glm::vec3 v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}

void tick(void) {
  glutPostRedisplay();
}

void display(void) {
  glClear(GL_COLOR_BUFFER_BIT);
#if 1
  auto faces = loader.getFaces();
  auto verts = loader.getVerts();
  for(auto f : faces) {
    glBegin(GL_LINE_LOOP);
    for(auto v : f.v) {
      glVertex3f(verts[v].x, verts[v].y, verts[v].z);
    }
    glEnd();
  }
#else

#endif
  glutSwapBuffers();
}

void reshape(int width, int height) {
  float ratio = width / (float) height;
  glViewport(0, 0, width, height);
  glClear(GL_COLOR_BUFFER_BIT);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y) {
  switch(key) {
    case 27:
      exit(0);
  }
}

int main(int argc, char** argv) {
  if(argc <= 1) {
    cout << "Usage: ./mesh_simplification mesh_file" << endl;
  }
  string filename(argv[1]);

  loader.load(filename);

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
  glutInitWindowSize(640, 480);
  glutCreateWindow("Mesh Simplification");
  glClearColor(0.0, 0.0, 0.0, 1.0);
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutIdleFunc(tick);
  glutKeyboardFunc(keyboard);
  glutMainLoop();

  return 0;
}
