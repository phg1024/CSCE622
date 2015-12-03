#include "meshloader.h"
#include "hds_mesh.h"

#include <GLFW/glfw3.h>

#include <iostream>
#include <string>
using namespace std;

ostream& operator<<(ostream& os, glm::vec3 v) {
  os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
  return os;
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
  if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
  glfwSetWindowShouldClose(window, GL_TRUE);
}

int main(int argc, char** argv) {
  if(argc <= 1) {
    cout << "Usage: ./mesh_simplification mesh_file" << endl;
  }
  string filename(argv[1]);
  OBJLoader loader;
  loader.load(filename);
  auto hds = build_half_edge_mesh<glm::vec3, double, NullType>(loader.getFaces(), loader.getVerts());

  if (!glfwInit())
  exit(EXIT_FAILURE);

  GLFWwindow* window = glfwCreateWindow(640, 480, "My Title", NULL, NULL);
  if (!window) {
    glfwTerminate();
    exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glfwSetKeyCallback(window, key_callback);

  while (!glfwWindowShouldClose(window)) {
    float ratio;
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    ratio = width / (float) height;
    glViewport(0, 0, width, height);
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);

    glColor3f(1.f, 1.f, 1.f);

    auto faces = hds.faces();
    for(auto fit = faces.first; fit != faces.second; ++fit) {
      glBegin(GL_LINE_LOOP);
      auto he = hds.halfedge(*fit);
      auto& he_prop = hds.halfedge(he);
      auto curHe = he_prop.prev;
      while(curHe != he) {
        //cout << he_prop.v << ": " << hds[he_prop.v].prop << endl;
        //cout << he_prop.v << " -> " << he_prop.prev << endl;
        curHe = he_prop.prev;
        he_prop = hds.halfedge(curHe);
      }
      glEnd();
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
