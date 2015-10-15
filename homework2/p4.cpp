#include <iterator>
#include <iostream>
#include <list>
#include <forward_list>
#include <vector>
using namespace std;

template <class iterator_tag>
struct iterator_name {};

#define SPECIALIZE_ITERATOR_NAME_TEMPLATE(iterator_tag, name_string) \
  template <> \
  struct iterator_name<iterator_tag> { \
    static const string value; \
  }; \
  const string iterator_name<iterator_tag>::value = name_string;

SPECIALIZE_ITERATOR_NAME_TEMPLATE(input_iterator_tag, "Input Iterator")
SPECIALIZE_ITERATOR_NAME_TEMPLATE(output_iterator_tag, "Output Iterator")
SPECIALIZE_ITERATOR_NAME_TEMPLATE(forward_iterator_tag, "Forward Iterator")
SPECIALIZE_ITERATOR_NAME_TEMPLATE(bidirectional_iterator_tag, "Bidirectional Iterator")
SPECIALIZE_ITERATOR_NAME_TEMPLATE(random_access_iterator_tag, "Random Access Iterator")

template <class Iterator>
void print_category(Iterator x) {
  using category = typename iterator_traits<Iterator>::iterator_category;
  cout << iterator_name<category>::value << endl;
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
