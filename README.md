# hpp-timeopt

This is a modified version of the software for optimization of centroidal momentum dyanmics (https://git-amd.tuebingen.mpg.de/bponton/timeoptimization) in order to use in HPP (https://github.com/humanoid-path-planner). This module can generate COM trajectory for humanoid robot using time-optimization by pondon's works.

Dependencies
----
The present software depends on several packages which have to be available in Ubuntu 14.04 / 16.04

-Libraries
  - Eigen3
  - YAML_cpp (sudo apt-get install libyaml-cpp-dev)

-System tools
   - CMake (>=2.6)
   - pkg-config
   - usual compilation tools (GCC/G++, make, etc.)


Install
----
To install this moulde: 

  1. Install HPP 
	- see https://github.com/humanoid-path-planner/hpp-doc
	
  2. Install YAML-cpp
   ```bash
   sudo apt-get install libyaml-cpp-dev
   ```
  3. Use CMake to install the library. For instance:
   ```bash
   mkdir $hpp-timeoptimization/build
   cd $hpp-timeoptimization/build
   cd cmake ..
   make install
   ```
  4. (Optional) If you want to use this module in Python
	- see https://github.com/ggory15/hpp-timeopt-corba
  
Inputs & Outputs of Algorithm
----
   1. Inputs
   	robot's mass, robot's initial com, desired contact sequences, (ref) linear and angular momentum (mainly all zeros)
	robot's desired com at final state, etc.
   2. Outpus
   	time sequence(especially, timeoptimization mode) 
	robot's continous com, linear & angular momentum trajectories, etc. 

Demo
----
The demo is based on DYROS-RED URDFs. The URDF files can be obtained by https://github.com/ggory15/dyrosred.

Also you could use your robot's URDF file, if you rewrite some lines. 

To run the demo script: 

  1. Running the demo code. For example,
  ```bash
  cd build/demos/
  ./demo_momentumopt -i <name_of_cfg_file_within_config_folder>
  ```
  
  2. Dispaly the result of the demo code
  ```bash 
  python <path_to_momentumopt>/scripts/display.py -i <path_to_momentumopt>/config/<name_of_cfg_file_within_config_folder>_results.yaml
  ```
  
 The detial of demo script is shown in https://git-amd.tuebingen.mpg.de/bponton/timeoptimization

Cartesian Coordinate
----
The global frame of this module is as follows:
'x' : forward direction of humanoid.
'y' : segital direction

Related Works
----
video (B. Ponton, "https://www.youtube.com/watch?v=ZGhSCILANDw")

article (B. Ponton et al, "https://arxiv.org/abs/1709.09265")


License
----
The software is distributed under the [GNU General Public License v3.0](http://www.gnu.org/copyleft/gpl.html).

Credits
----

+ Max Planck Society (developer of time and momentum optimization)
+ Alexander Domahidi (developer of interior point solver)
+ Timothy A. Davis (developer of sparse matrix routines)
+ B. Ponton (origianl source, https://git-amd.tuebingen.mpg.de/bponton/timeoptimization)

