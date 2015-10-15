// To toggle the fixed version, change the following 0 to 1
#define FIX_IT 1

#include <assert.h>
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
    std::copy(data, data+n, other.data);
  }

  array& operator=(const array& other){
    cout << "invoking copy assignment operator ..." << endl;
    if (this == &other) return *this;
    delete[] data; data = nullptr;
    new(this) array(other);
    return (*this);
  }

#if FIX_IT
  array(array&& other) : data(other.data), n(other.n) {
    cout << "invoking move constructor ..." << endl;
    other.data = nullptr;
  }

  array& operator=(array&& other) {
    cout << "invoking move assignment operator ..." << endl;
    delete[] data;
    std::swap(n, other.n);
    std::swap(data, other.data);
    other.data = nullptr;
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
  ::array<int> a(3);
  auto memory_ptr = a.data_ptr();

  auto f = [&]() {
    auto b(std::move(a));
    return b;
  };

  ::array<int> c = f(), d(5);
  assert(c.data_ptr() == memory_ptr);

  d = std::move(c);
  assert(d.data_ptr() == memory_ptr);
  return 0;
}
