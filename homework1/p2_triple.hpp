// Author: Peihong Guo
#ifndef TRIPLE_H_
#define TRIPLE_H_

template <typename T1, typename T2, typename T3>
struct triple {
  using first_type=T1;
  using second_type=T2;
  using third_type=T3;

  T1 first;
  T2 second;
  T3 third;

  triple() : first(), second(), third() {}
  triple(const T1& first_in, const T2& second_in, const T3& third_in)
    : first(first_in), second(second_in), third(third_in) {}

  triple(const triple& other) = default;
  triple& operator=(const triple& other) = default;

  bool is_zero() const;
};

#ifdef IS_ZERO_VERSION_1
template <typename T1, typename T2, typename T3>
bool triple<T1, T2, T3>::is_zero() const {
 return first == T1(0) && second == T2(0) && third == T3(0);
}
#endif

#ifdef IS_ZERO_VERSION_2
template <typename T1, typename T2, typename T3>
bool triple<T1, T2, T3>::is_zero() const {
 return first == 0 && second == 0 && third == 0;
}
#endif

template <typename T1, typename T2, typename T3>
inline bool operator==(const triple<T1, T2, T3>& t1,
                       const triple<T1, T2, T3>& t2) {
  return t1.first == t2.first && t1.second == t2.second && t1.third == t2.third;
}

template <typename T1, typename T2, typename T3>
inline bool operator!=(const triple<T1, T2, T3>& t1,
                       const triple<T1, T2, T3>& t2) {
  return !(t1 == t2);
}

template <typename T1, typename T2, typename T3>
triple<T1, T2, T3> make_triple(T1 t1, T2 t2, T3 t3) {
  return triple<T1, T2, T3>(t1, t2, t3);
}

#endif  // TRIPLE_H_
