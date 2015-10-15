// To toggle the fixed version, change the following 0 to 1
#define FIX_IT 0

#include <iostream>
using namespace std;

template <typename T>
class array {
  T* data;
  int n;

public:
#if FIX_IT
  array() : data(nullptr), n(0) {}
#endif
  array (int n_) : data(new T[n_]), n(n_) {}
  ~array () { delete [] data; }

  friend bool operator==(const array<T>& v1, const array<T>& v2) {
    return v1.data == v2.data;
  }
  T& operator[](int i) { return data[i]; }
  int size() { return n; }
};

int main(int argc, char** argv) {
  // This does not compile
  ::array<::array<int>> array_of_array(3);
  return 0;
}
