<div align="center">
<h3>CSCE 622: Generic Programming -- Assignment 3</h3>
<h4>Peihong Guo UIN: 421003404</h4>
</div>

#### Introduction
Blah blah blah

#### Implementation Details
##### Graph Definition
The graph used in this assignment is defined using the `adjacency_list` structure with 2 vertex properties: vertex name and vertex color.
```
using Graph = adjacency_list<
                listS, vecS, directedS,
                property<vertex_name_t, string,
                  property<vertex_color_t, default_color_type>>
              >;
```

##### Visitor Design
The path count algorithm is based on customized depth first search starting from the source vertex. An integer value is attached to each vertex to keep track of the path counts from a vertex to the target vertex. Two kinds of events are interested in this algorithm: `discover_vertex` and `finish_vertex`.

In `discover_vertex`, the path count of a vertex is set to 1 if the vertex is the target vertex, otherwise the path count is set to 0.

In `finish_vertex`, the path count of a vertex is set to the sum of path counts of all its descendants.

The algorithm terminates once the source vertex is finished.

The visitor is designed based on the above observation:
1. The visitor inherits from `default_dfs_visitor` to provide default behavior for the events other than the `discover_vertex` and `finish_vertex`.
2. A `map<Vertex, int>` object is included as a member of the visitor to store the path counts at each vertex. A `map<Vertex, vector<list<string>>>` object is also added to the visitor to store the found paths.
3. The final path count is made available through a reference to external path count variable.

```
template <typename Graph>
PathCounter {
public:
  using VertexDescriptor = typename graph_traits<VertexListGraph>::vertex_descriptor;
  using EdgeSizeType = typename graph_traits<VertexListGraph>::edges_size_type;
  using path_t = list<string>;

  PathCounter(VertexDescriptor s, VertexDescriptor t, EdgeSizeType& path_count)
    : s(s), t(t), path_count(path_count) {}

  template <typename Vertex, typename Graph>
  void discover_vertex(Vertex u, const Graph& g) {
    ...
  }

  template <typename Vertex, typename Graph>
  void finish_vertex(Vertex u, const Graph& g) {
    ...
  }

private:
  VertexDescriptor s, t;
  map<VertexDescriptor, int> path_counter;
  map<VertexDescriptor, vector<path_t>> paths;
  EdgeSizeType& path_count;
}
```

`discover_vertex` is a simple function that marks the path count of a vertex based on whether it is the target vertex or not:
```
template <class Vertex, class Graph>
void discover_vertex(Vertex u, const Graph& g) {
  path_counter[u] = (u==t)?1:0;
  paths[u] = (u==t)?vector<path_t>(1, path_t(1, u)):vector<path_t>();
}
```
Note that the found paths are also updated in a similar way as the path count.

`finish_vertex` needs to sum up the path counts of a vertex's descendants and use that as its new path count:
```
template <class Vertex, class Graph>
void finish_vertex(Vertex u, const Graph& g) {
  // Update the path count
  auto vp = adjacent_vertices(u, g);
  path_counter[u] += std::accumulate(vp.first, vp.second, 0,
                                     [&](size_t acc, Vertex v) {
                                       return acc + path_counter[v];
                                     });

  // Store the found paths
  auto& paths_u = paths[u];
  for(auto vp = adjacent_vertices(u, g); vp.first != vp.second; ++vp.first) {
    const auto& paths_v = paths[*vp.first];
    for(auto p : paths_v) {
      path_t this_path(1, get(vertex_name, g, u));
      this_path.insert(this_path.end(), p.begin(), p.end());
      paths_u.push_back(this_path);
    }
  }

  // Store final result
  if( u == s ) path_count = path_counter[u];
}
```
The found paths are also updated based on the found paths stored in the descendants of this vertex.

##### Graph File Format
The graph files used in the assignment follows the format below
```
n m
v1_name v2_name ... vertex_n_name
0
e1
e2
...
em
```
where `n` is the number of vertices and `m` is the number of edges. Each edge `ek` is a pair of vertex names. For example, `a b` represents an edge from vertex `a` to vertex `b`. The `0` between vertex names and edge list is a placeholder for future use.

##### Random Graph Generation
To generate random directed acyclic graph (DAG), it is important to make sure the edges do not form loops in the graph. This can be achieved by assigning ranks to the vertices and only allow edges from higher rank vertices to lower rank vertices.

#### Summary
Blah blah blah
