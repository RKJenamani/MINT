# MINT

Current stable(hopefully) implementation is of LazySP on implicit tensor graph.
Search Algorithm implemented in OMPL, also using Boost Graph Library.

### Dependencies:
1. C++11 or higher
2. cmake
3. OMPL
4. Boost Graph Library
5. OpenCV 
6. PRL Stack

The CMakeLists.txt file supports catkin tools. Once you have created and initialized your workspace, 
you should be able to build the package by running `catkin build MINT`.

MINT has been implemented as an OMPL planner and can be invoked on a `RealVectorStateSpace`.

------

### Examples:
The executables [`exampleHERB`, `exampleTwoAgentsMINT`] can be found under `build/MINT`.

#### exampleHERB

This executables expects three command-line arguments that might be of interest:

1. Path to Left Graph (--left_graph option) 
	- defaults to data/graphs/HERB_graphs/1/left_arm/herb_halton_l_100_20.graphml
2. Path to Right Graph (--right_graph option) 
	- defaults to data/graphs/HERB_graphs/1/right_arm/herb_halton_r_100_20.graphml
3. Enable to execute final path (--execute)
	- enabled by default
  
From the root folder of the catkin workspace:
 `./build/MINT/exampleHERB`

#### exampleTwoAgentsMINT 
 
The executables expect five command-line arguments that might be of interest:

1. Path to Left Graph (--left_graph option) 
	- defaults to data/graphs/halton_2d_withedges.graphml
2. Path to Right Graph (--right_graph option) 
	- defaults to data/graphs/halton_2d_withedges.graphml
3. Path to obstacle (--obstaclefile option) 
	- defaults to data/obstacles/circle2D.png (requires image format)
4. Source (4D) between 0 and 1 (-s option) 
5. Target (4D) between 0 and 1 (-t option) 

From the root folder of the catkin workspace:
 `./build/MINT/exampleTwoAgentsMINT -s 0.1 0.1 0.2 0.1 -t 0.9 0.9 0.8 0.9`

[lazySP]: https://personalrobotics.ri.cmu.edu/files/courses/16843/notes/lazysp/lazysp-2016.pdf
