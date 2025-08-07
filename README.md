# Computational Methods for Finding the Stable State of Convex Morphable Meshes

This C++ project is used to convert a disk with a convex polyhedral metric into an isometric convex cap. It contains a halfedge data structure library `MeshLib` and an simple opengl viewer.

## System

The code is only tested on Windows, but it should work on Linux and Mac with minor modifications. If there is any problem on the latter two platforms, please let me know.

## Dependencies
 
1. `MeshLib`, a mesh library based on halfedge data structure.
2. `freeglut`, a free-software/open-source alternative to the OpenGL Utility Toolkit (GLUT) library.
3. `Eigen`, an open-source library used for performing vector and matrix operations.

## Directory Structure

``` txt
3rdPartySoftware        -- MeshLib, freeglut, and Eigen libraries.
cutgraph/include        -- The header files of cut graph algorithm.
cutgraph/src            -- The source files of cut graph algorithm. 
data                    -- Some models.
CMakeLists.txt          -- CMake configuration file.
```

## Configuration

### Windows

1. Install [CMake](https://cmake.org/download/).

2. Download the source files of the C++ project.
> E.x. I create a folder `projects` in `C:/`, then copy the source files there.

3. Configure and generate the project for Visual Studio.

> ``` bash
> cd projects
> mkdir build
> cd build
> cmake ..
> ```
> *One can also finish this step using CMake GUI.*

4. Open the \*.sln using Visual Studio, and complie the solution.

5. Finish your code in your IDE.

6. Run the executable program.
> E.x. 
> ``` bash
> cd bin
> ./CutGraph.exe ../data/trihex+.m
> ./CutGraph.exe ../data/pentagon.m
> ```

### Linux & Mac

1. Build and compile the code.

> ``` bash
> cd projects
> mkdir build
> cd build
> cmake ..
> make && make install
> ```

2. Run the executable program.

> ``` bash
> cd ../bin/
> ./CutGraph ../data/trihex+.m
> ./CutGraph ../data/pentagon.m 
> ```


