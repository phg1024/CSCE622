#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
using namespace std;

template <typename BidirectionalIterator>
void func1(BidirectionalIterator begin, BidirectionalIterator end) {
  while (std::next_permutation(begin, end));
}

template <typename T>
string serialize(const T& t) {
  return std::to_string(t);
}

template <>
string serialize<string>(const string& t) {
  return t;
}

template <typename T>
string serialize(const vector<T>& v) {
  string res = "(";
  for(int i=0;i<v.size();++i) {
    res += serialize(v[i]);
    if( i<v.size()-1 ) res += ", ";
  }
  res += ")";
  return res;
}

template <typename T>
bool test_equal(const T& u, const T& v) {
  return u == v;
}

template <typename T>
bool test_func1(const T& seq) {
  // sort the sequence using STL sort
  T seq_sorted(seq);
  std::sort(seq_sorted.begin(), seq_sorted.end());

  // invoke func1
  T seq_cpy(seq);
  func1(seq_cpy.begin(), seq_cpy.end());

  // Compare the results
  if(!std::equal(seq_cpy.begin(), seq_cpy.end(), seq_sorted.begin())) {
    cerr << "test failed: " << serialize(seq_cpy) << endl
         << " != "
         << serialize(seq_sorted) << endl;
    return false;
  } else return true;
}

int main(int argc, char** argv) {
  bool all_test_passed = true;

  // example 1: acedb -> abcde
  all_test_passed &= test_func1(string("acedb"));

  // example 2: (3, 2653, 14, 159) -> (3, 14, 159, 2653)
  all_test_passed &= test_func1(vector<int>{3, 2653, 14, 159});

  // example 3
  const int num_tests = 1000;
  for(int i=0;i<num_tests;++i) {
    if((i+1) % 100 == 0) cout << i+1 << " tests finished." << endl;
    // must keep the number of elements small because of the O(n!) algorithm
    const int num_elements = 10;
    vector<int> v(num_elements);
    for(int j=0;j<num_elements;++j) v[j] = rand();
    all_test_passed &= test_func1(v);
  }

  cout << "All tests passed." << endl;

  return 0;
}
