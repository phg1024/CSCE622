#include <iostream>

template <class T>
inline T sum_all(T* first, T* last) {
    T sum;
    for (sum = 0; first != last; ++first)
        sum += *first;
    return sum;
}

struct DummyType {
  DummyType() {}
  DummyType(int x) {}

  DummyType& operator+=(const DummyType& other) {
    return (*this);
  }
};

bool operator==(const DummyType& a, const DummyType& b) {
  return true;
}

int main(int argc, char **argv) {
  DummyType arr[] = {DummyType(1), DummyType(3), DummyType(3)};
  DummyType sum = sum_all(&arr[0], &arr[0]+3);
  return 0;
}
