#### Path Count Algorithm using Boost DFS
##### Build
To build the programs, make sure CMake and Boost graph library are properly installed.
```
cd [source_code_directory]
mkdir build
cd build
cmake ..
make -j4
```
##### Using graph files
```
path_count_with_file [graph_file] [query_file]
```

For example, to use the included `graph0.txt` and `query0.txt`:
```
path_count_with_file graph0.txt query0.txt
```

The graph file should use the following format
```
n m
vertex_1_name vertex_2_name ... vertex_n_name
0
edge_1
edge_2
...
edge_m
```
where `n` is the number of vertices and `m` is the number of edges. Each edge is a pair of vertex names separated by a space.

##### Using random graphs
```
path_count_random_graph num_vertices num_edges num_queries
```

For example, to test the algorithm on a random graph with 5 nodes and 8 edges using 2 queries:
```
path_count_random_graph 5 8 2
```

*Note: to suppress the progress log, simply add 2>log.txt at the end of the command line.*
