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
  array (const array& other) : data(new T[other.n]), n(other.n) {
    cout << "invoking copy constructor ..." << endl;
    memcpy(data, other.data, sizeof(T)*n);
  }
  ~array () { delete [] data; }

  friend bool operator==(const array<T>& v1, const array<T>& v2) {
    if( v1.n != v2.n ) return false;
    for(int i=0;i<v1.n;++i) {
      if( v1.data[i] != v2.data[i] ) return false;
    }
    return true;
  }

#if FIX_IT
  friend bool operator!=(const array<T>& v1, const array<T>& v2) {
    return !(v1 == v2);
  }
#endif

  T& operator[](int i) { return data[i]; }
  int size() { return n; }
};

int main(int argc, char** argv) {
  ::array<int> a1(3);
  ::array<int> a2(a1);

  // This checks if the two arrays are equal
  // This does not compile unless inequality operator is defined
  cout << ((a1 != a2)?"a1 != a2":"a1 == a2") << endl;
  return 0;
}
