State variables synchronization
===============================

This folder ( __StandaloneC/src/other/varState/synchronize__ ) is used to synchronize the [Matlab](http://www.mathworks.nl/products/matlab/) files (in __workR/Simulink__):
* simu_variables.m
* control_variables.m

from the .txt files (in __StandaloneC/src/project/varState__):
* simu_variables.txt
* control_variables.txt

The following commands can be used in a Unix terminal to compile and run this project (similar actions can be done with IDEs, also with Windows, Mac OS and Linux).
For more information on the use of [CMake](http://www.cmake.org/), please have a look at the [README.md](StandaloneC/README.md) file located in _StandaloneC_.

* cd StandaloneC/src/other/varState/synchronize
* mkdir build
* cd build
* cmake ..
* make
* ./Gen_state_var

Beware, the comments in _simu_variables.txt_ and _control_variables.txt_ will not be duplicated !
