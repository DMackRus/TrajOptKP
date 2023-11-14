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
### Setup
There are two config yaml files that the user can change to run different examples. The first config 
is called **generalConfig.yaml**. This file determines various high level settings (such as optimiser, task, etc.) which are explained 
in the configuration file.

The second config file is specific to the task being loaded, all task config files are located in **taskConfigs** folder. 
There are some high level settings, as follows:
- **modelFile**: Relative path to the model xml file
- **modelName**: Name of the model, used for saving data
- **timeStep**: Time step for simulation
- **keypointMethod**: Key-point method to use in optimisation. See below for more details.
- **minN**: Minimum interval between key-points
- **maxN**: Maximum interval between key-points
- **iterativeErrorThreshold**: Error threshold for iterative error method

As well as these high level settings, there is the task description. Every task is specified by a collection of **robots** amd **bodies**.
**Robots** are actuated whereas **bodies** are not. This list of robots and bodies instantiates the trajectory 
optimisation problem, by defining starting and desired states, as well as cost attributes. Finally, there are also settings
for each DoF that relate to key-point methods, Please see the [Key-points](#Key-points) section for additional details.

### Run the code
To run the code, there is a bash script that handles building and running provided. Simply run the
following command:
```
bash run.bash
```

## Examples
Here are some example trajectories that have been generated using this package.

**Manipulation**

<p align="middle">
  <img src="media/box-sweep.gif" width="300"/>
  <img src="media/push_low_clutter.gif" width="300"/>
</p>

**Locomotion**

**Dynamic motion**


## Key-points
Description coming soon.

## To-Do
- [ ] Add more examples
- [ ] Implement rotation of bodies in state vector and F.D.
- [ ] Rework main.cpp so that all tasks are their own executable instead.
- [ ] Improve README readability.

## Citing
Coming soon.

