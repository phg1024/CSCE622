// To toggle the fixed version, change the following 0 to 1
#define FIX_IT 0

#include <iostream>
using namespace std;

template <typename T>
class array {
  T* data;
  int n;

public:
  array (int n_) : data(new T[n_]), n(n_) {}
  ~array () { delete [] data; }

  friend bool operator==(const array<T>& v1, const array<T>& v2) {
    return v1.data == v2.data;
  }

  T& operator[](int i) { return data[i]; }
#if FIX_IT
  const T& operator[](int i) const { return data[i]; }
  int size() const { return n; }
#else
  int size() { return n; }
#endif
};

template <typename T>
bool contains(const ::array<T>& A, const T& element) {
  for(int i=0;i<A.size();++i) {
    if(element == A[i]) return true;
  }
  return false;
}

int main(int argc, char** argv) {
  // This does not compile
  ::array<int> A(3);
  A[0] = 1; A[1] = 2; A[2] = 3;

  if( contains(A, 5) ) cout << "A contains 5." << endl;
  else cout << "A does not contain 5." << endl;
  return 0;
}
