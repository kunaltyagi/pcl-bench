# pcl-bench
Benchmarks for PCL

## Compilation
```
cmake -DCMAKE_BUILD_TYPE=Release ..
./pcl_bench
```
If you want to see perf events, use
```
perf stat -e LLC-loads-misses,branch-misses ./pcl_bench
```
If you just want what took most time, use
```
perf record -g ./pcl_bench
perf report -g 'graph,0.5,caller' -v
```
