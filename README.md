# lifelong generalized_lazy_search

##### Under development for improvements.

Framework implements various lazy search algorithms. The planners have been implemented in OMPL.

Dependencies:
- C++11 or higher
- cmake
- OMPL
- Boost Graph Library

The CMakeLists.txt file supports catkin tools. Once you have created and initialized your workspace,
you should be able to build the package by running `catkin build lgls`.

The planner implemented under LGLS return the shortest path on the roadmap graph it is planning on.

There are three planners available : LPA*, LGLS, GLS
- LPA* : original Lifelong Planning A*, implemented by Jaein Lim
- LGLS : Lifelong-GLS, implemented by Jaein Lim
- GLS  : Generalized Lazy Search original implementation by AVK.

Note:
LGLS is Jaein's implementation of Lifelong-GLS. The initial search of LGLS is equivalent to GLS, but the LGLS implementation is significantly different than AVK's GLS version. 

------

Example:

An example graph and an environment have been provided to run with `examples/test_LGLS.cpp`.

* The source and target locations are assumed to be in [0, 1]
* The obstacle file is for visualization purposes, and should correspond with the GraphML file.
* A short-cut to the default graph and obstacle files in the `examples` folder can be run with the following launch file:
```
roslaunch lgls testLGLS.launch
```
Parameters: `graph`, `obstacles`, `start_x`, `start_y`, `target_x`, `target_y`

Similarly, LPA* and GLS can be run
```
roslaunch lgls testLPA.launch
roslaunch lgls testGLS.launch
```
