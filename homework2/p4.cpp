#include <iterator>
#include <iostream>
#include <list>
#include <forward_list>
#include <vector>
using namespace std;

template <typename iterator_category>
void print_category_dispatch();

template <>
void print_category_dispatch<input_iterator_tag>() {
  cout << "Input iterator" << endl;
}

template <>
void print_category_dispatch<output_iterator_tag>() {
  cout << "Output Iterator" << endl;
}

template <>
void print_category_dispatch<forward_iterator_tag>() {
  cout << "Forward Iterator" << endl;
}

template <>
void print_category_dispatch<bidirectional_iterator_tag>() {
  cout << "Bidirectional Iterator" << endl;
}

template <>
void print_category_dispatch<random_access_iterator_tag>() {
  cout << "Random Asscess Iterator" << endl;
}

template <class Iterator>
void print_category(Iterator x) {
  print_category_dispatch<typename iterator_traits<Iterator>::iterator_category>();
}

int main(int argc, char** argv) {
  {
    char a[10];
    print_category(a+10);
  }

  {
    std::list<int> a;
    print_category(a.begin());
  }

  {
    std::forward_list<int> a;
    print_category(a.begin());
  }

  {
    std::istream_iterator<double> a;
    print_category(a);
  }

  {
    std::ostream_iterator<double> a(cout);
    print_category(a);
  }

  return 0;
}
