# Trajectory Optimisation With Key-points (TrajOptKP)
This package showcases an efficient method to perform gradient-based trajectory optimisation by reducing 
the number of expensive finite-differencing computations required to perform optimisation. The basic
methodology is computing key-points over a trajectory where expensive finite-differencing computations
are performed, the remainder of the dynamics derivatives needed for trajectory optimisation are then
approximated via linear interpolation.

This package includes a set of example tasks that can be solved via trajectory optimisation, including 
non-prehensile manipulation and locomotion. This package is implemented in C++, and uses MuJoCo as the
physics simulator.

Please note that this code is still under active development.

## Dependencies
- [MuJoCo 2.32](http://www.mujoco.org/)
- [Eigen 3](https://eigen.tuxfamily.org/dox/GettingStarted.html)
- [YAML](https://github.com/jbeder/yaml-cpp)

## Installation

1. Clone this repository (Please note that this repository uses submodules, 
so you need to clone recursively.
   ```
   git clone --recursive https://github.com/DMackRus/TrajOptKP.git
   ```  
2. Set the following environment variables.
   ```
   export MJ_HOME=*path to the home directory of MuJoCo*
   ```
3. Build the package.
   ```
   cd TrajOptKP
   mkdir build
   cd build
   cmake ..
   make
   ```

## Usage
To run examples for this repository, there are two main config files you need to be aware of. The first
is a general config file called generalConfig.yaml. This file determines which task is going to be
simulated, what type of optimisation will be used, 

For each task, there is a task specific config file. This file determines the parameters of the task,
including the start and desired state of the task, the cost function, and the key-point method used.

To run the code, a bash script is provided in the repository to build the project and then 
automatically run the generated executable.

```
bash run.bash
```


## Examples

## Citing
Coming soon.

