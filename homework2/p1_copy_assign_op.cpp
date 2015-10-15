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
#if FIX_IT
  array& operator=(const array& other){
    cout << "invoking copy assignment operator ..." << endl;
    if(&other != this) {
      delete [] data;
      n = other.n;
      data = new T[n];
      memcpy(data, other.data, sizeof(T)*n);
    }
    return (*this);
  }
#endif
  ~array () { delete [] data; }

  friend bool operator==(const array<T>& v1, const array<T>& v2) {
    return v1.data == v2.data;
  }

  const T* data_ptr() const { return data; }

  T& operator[](int i) { return data[i]; }
  int size() { return n; }
};

int main(int argc, char** argv) {
  // use a nested scope to make sure a1 is destroyed at the end of this scope.
  {
    ::array<int> a1(3);

    // use a nested scope to make sure a2 is destroyed at the end of this scope.
    // when a2 is destroyed, the memory block it holds is deallocated.
    {
      ::array<int> a2(1);
      // invoke the copy assignment operator
      a2 = a1;

      // This checks if the two arrays share the same memory block
      if( a1.data_ptr() == a2.data_ptr() ) {
        cout << "Both arrays share memory block: " << a1.data_ptr() << endl;
      } else {
        cout << "The arrays have different memory blocks: " << endl;
        cout << "a1.data = " << a1.data_ptr() << endl;
        cout << "a2.data = " << a2.data_ptr() << endl;
      }
    }

    // a1 will be destroyed here, but it will crash the program because of its
    // data pointer is dangling at this point.
  }
  return 0;
}
