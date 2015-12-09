#ifndef TAGGED_VALUE_H
#define TAGGED_VALUE_H

#include <functional>

template <typename ValueType, typename TagType>
struct TaggedValue {
  typedef ValueType value_t;
  typedef TagType tag_t;

  TaggedValue() = default;
  TaggedValue(const ValueType& value_in) : value(value_in) {}

  ValueType& operator*() { return value; }
  const ValueType& operator*() const { return value; }

  bool operator==(const TaggedValue& other) const {
    return value == other.value;
  }

  template <typename VT, typename TT>
  friend ostream& operator<<(ostream& os, const TaggedValue<VT, TT>& val);

  ValueType value;
};

template <typename VT, typename TT>
ostream& operator<<(ostream& os, const TaggedValue<VT, TT>& val) {
  os << *val;
  return os;
}

template <typename TaggedValue>
struct TaggedValueHasher {
  std::size_t operator()(const TaggedValue& value) const {
    return std::hash<typename TaggedValue::value_t>()(*value);
  }
};

#endif  // TAGGED_VALUE_H
