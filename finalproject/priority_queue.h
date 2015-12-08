#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include <iostream>
#include <queue>
#include <vector>
#include <climits>
#include <unordered_map>
using namespace std;

template <typename Key, typename Value, typename Comp=std::greater<Value>>
class priorty_queue {
public:
  priorty_queue() {}
  bool empty() const {
    return data.empty();
  }
  const pair<Key, Value>& top() const {
    return data.front();
  }
  pair<Key, Value>& top() {
    return data.front();
  }
  pair<Key, Value> pop() {
    auto top = data.front();
    index.erase(top.first);
    data.front() = data.back();
    data.pop_back();
    if(!data.empty()) push_down(0);
    return top;
  }
  int push(const Key& key, const Value& value) {
    data.push_back(make_pair(key, value));
    int cur = data.size()-1;
    index[key] = cur;
    cur = push_up(cur);
    return cur;
  }
  void update(const Key& key, const Value& value) {
    auto it = index.find(key);
    if(it == index.end()) {
      cerr << "key = " << key << endl;
      throw std::out_of_range("The key value pair is not in priority queue!");
    } else {
      int cur = it->second;
      data[cur].second = value;
      push_up(cur);
      push_down(cur);
    }
  }

  template <typename KT, typename VT, typename CT>
  friend ostream& operator<<(ostream& os, const priorty_queue<KT, VT, CT>& Q);

protected:
  //    0
  //  1   2
  // 3 4 5 6
  int parent(int x) {
    if(x == 0) return -1;
    else return (x-1) >> 1;
  }

  int left_child(int x) {
    return (x << 1) + 1;
  }

  int right_child(int x) {
    return (x << 1) + 2;
  }

  void swap(int a, int b) {
    index[data[a].first] = b;
    index[data[b].first] = a;
    std::swap(data[a], data[b]);
  }

  int push_up(int cur) {
    Comp comparer;
    int p = parent(cur);
    while(p >= 0 && comparer(data[p].second, data[cur].second)) {
      swap(p, cur);
      cur = p;
      p = parent(cur);
    }
    return cur;
  }

  int push_down(int cur) {
    Comp comparer;
    while(true) {
      auto min_idx = cur;
      int left_child_idx = left_child(cur);
      if(left_child_idx < data.size()) {
        min_idx = comparer(data[min_idx].second, data[left_child_idx].second)?left_child_idx:min_idx;
      }
      int right_child_idx = right_child(cur);
      if(right_child_idx < data.size()) {
        min_idx = comparer(data[min_idx].second, data[right_child_idx].second)?right_child_idx:min_idx;
      }

      if(min_idx > cur) {
        swap(cur, min_idx);
        cur = min_idx;
      } else break;
    }
    return cur;
  }

private:
  vector<pair<Key, Value>> data;
  std::unordered_map<Key, int> index;
};

#endif  // PRIORITY_QUEUE_H
