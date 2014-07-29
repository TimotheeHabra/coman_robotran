Simulink version
================


Installation
------------

Three things are needed to use the Simulink version:
* [Robotran](http://www.robotran.be/) must be downloaded and installed. Use the following [link](http://www.robotran.be/download). 
* [Matlab](http://www.mathworks.nl/products/matlab/) must be installed with [Simulink](http://www.mathworks.nl/products/simulink/).
* A [Mex](http://www.mathworks.nl/help/matlab/matlab_external/introducing-mex-files.html) compiler must be installed.

MEX stands for Matlab EXecutable, providing an interface between Matlab and subroutines in C, C++ or Fortran. On 32-bit versions of Matlab, a Mex compiler might already be installed. This is usually not the case for 64-bit versions of Matlab.

Type the command __mex -setup__  in the Command Window of Matlab.
If a compiler is available, you can try to use it. If it appears that this compiler does not work, you might have to install another one. For instance, you could see the following message (on Windows):

_Select a compiler :_
[1]  Microsoft Visual C++ 2010 Express in C:\Program Files(x86)\Microsoft Visual Studio 10.0

In this example, press __1__ to select the corresponding Mex compiler.

If no compiler compatible with your Matlab version is available, you will only see a message like this one:

_Select a compiler : [0]  None_

You then need to install an appropriate compiler. The compiler to install depends on your Matlab version, on your OS and differs if you are running a 32-bit or a 64-bit version (type __computer__ in the Command Window of Matlab if you don’t know which version you are using).

Here is a [list](http://www.mathworks.nl/support/sysreq/previous_releases.html) of the supported compilers, depending on your machine (look at the __Supported Compilers__ column).

If Robotran is correctly installed, you should see the following message: 'MBSpath added to Matlab path' when opening Matlab.

The folder of your project MUST be located in the Robotran project folder (defined during the Robotran installation).

### Windows hints ###

* For 2011 Matlab versions (or earlier), you can use _Microsoft Visual C++ 2010 Express_ along with _Microsoft Windows SDK 7.1_, for both
32-bit and 64-bit versions (look at this [link](http://www.mathworks.nl/support/solutions/en/data/1-ECUGQX/)).
* For 2012 Matlab versions (or later), you can use only _Microsoft Windows SDK 7.1_.
* If Matlab crashes during the execution, you might have to look at the file _MBsysLab/mbs_simulink/mbs_sourceC/nrutil.c_ installed by Robotran on your computer. There is a big '\#if defined(STDC) || defined(ANSI) || defined(NRANSI) /* ANSI */   \#else /* ANSI */   \#endif /* ANSI */'. Your compiler might be in the wrong part of this conditional statement (usually in the '\#else', while it should be in the '\#if defined...'). You can try to remove this big '\#if def \#else \#endif', keeping only the first part of this big conditional compilation.
* Matlab 64-bit only recognizes one free compiler on Windows: Windows SDK 7.1 (for more information, consult this [link](http://perso.uclouvain.be/allan.barrea/opencv/opencv.html#installation-of-visual-c-2010-express-and-windows-sdk-7-1)).


### Mac OS hints ###

* Robotran visualization on Mac OS can have some troubles due to Java on some Apple computers. See this [link](http://www.robotran.be/download) to solve the problem.
* When selecting the Mex compiler, (type the command __mex -setup__), you can choose the one called _mexopts.sh_ (the option should already be there).
* When compiling the program, you can have errors because the compiler required by _mexopts.sh_ is not installed on the computer (e.g. an error message is asking for _gcc.4_ which is not installed on the machine). A solution might be to install this compiler, but it is easier to modify the _mexopts.sh_ file. Open this _mexopts.sh_ file (its path should be visible after typing __mex -setup__). Locate the line where the compiler is indicated (e.g. locate the line where _gcc.4_ is written) and replace it with _gcc_. Indeed, _gcc_ is another compiler which should already be installed on Apple machines.
* When compiling the program, you might get a similar error: " /Applications/MATLAB_R2011a.app/extern/include/matrix.h:335:13: error: unknown type name 'char16_t' ". If this is the case, open this _matrix.h_ file and locate the corresponding line. Just above, there should be a line like this one: "typedef CHAR16_T char16_t;". Unfortunately, this line is in a block delimited by " #ifndef __STDC_UTF_16__ ", which prevents it from being compiled. In this case, remove the line " #ifndef __STDC_UTF_16__ " and the corresponding  " #endif ".
* This [link](http://www.mathworks.nl/support/solutions/en/data/1-FR6LXJ/) might help in some cases.


### Linux hints ###

* A problem similar to the one described above for the Mac OS computers (compiler not installed) might happen. Though, installing a new compiler on Linux is quite easy. For instance, you can type _sudo apt-get install gcc-4.4_ on a terminal if you are using _Ubuntu_.



Project overview
----------------

Here is an overview of the different files and folders used by the Simulink simulator:

* __animationR__ and __dataR__ : same usage as with Matlab projects.
* __symbolicR__ : _.c_ and _.h_ files are used to compute the symbolic equations of the multy-body system.
* __userfctR__ : only _user_cons_hJ.m_ is used (see below for more information).
* __workR/Simulink__ : Matlab files used to configure the Simulink project.
* __StandaloneC__ : while mainly used by the [Standalone version](StandaloneC/README.md), three folders located in _StandaloneC_ are also used by the Simulink verion:
	* __src/project/user_files__ : same purpose as the _userfctR_ folder used by Matlab projects.
	* __src/project/simulation_files__ : files used to let the user add new custom C files and to deal with the interface of a potential controller.
	* __src/project/controller_files__ : files used to design a controller in C. This controller is totally independent from the simulation environment.
	* __src/project/interface_controller__ : files used to handle the interface between the simulation (simulation_files) and the controller (controller_files). 

When designing a controller for a robot, it is very important to correctly separate all the files designed by the user between the _user_files_, the _controller_files_ and the _simulation_files_ folders. 
* All files located in __controller_files__ can be transferred to the real robot without any modification. Consequently, it is important to only use information actually available on the real robot in these files.
* The __simulation_files__ and the __user_files__ folders contain the user files which will not be transferred to the real robot. It is better to avoid adding new custom C files in the _user_files_ folder (keep only the generic Robotran fucntions). To add new files and new functions, use the _simulation_files_ folder.



Project creation
----------------

### Multibody equations ###

* Generate a new project (or use an existing one) in the same manner as what is explained in the [Robotran modeling features](http://www.robotran.be/downloads/tutorial/RobotranLearning_ModelingFeatures.pdf). At the end, generate the symbolic files. For the symbolic generation, you need to add a 'C' to the languages box (e.g. 'languages: Matlab, C').


### userfctR ###

* Go to the __userfctR__ folder (located at the root of your project) and open _user_cons_hJ.m_. Configure this file in a similar way to what is done for a Matlab project. Duplicate the Matlab code you just wrote in this file and translate it into C in the _user_cons_hJ.c_ file (located in _StandaloneC/src/project/user_files_). This is the only duplicata which needs to be done. Indeed, _user_cons_hJ.m_ will be used (by Matlab) during the coordinate partitioning while _user_cons_hJ.c_ will be used (by the C executable) during the rest of the simulation.


### workR/Simulink ###

* Go to the __workR/Simulink__ folder. This folder is the one where you will configure the project. In this folder, you should never open the files located in the _make_generate_, _object_files_ and _standalone_ folders. 

* In __generate_mbs_data.m__, configure this file to get the correct _mbs_data_ at the end, in a similar way as what is done for a Matlab Robotran project (again, see the [Robotran modeling features](http://www.robotran.be/downloads/tutorial/RobotranLearning_ModelingFeatures.pdf) slides). For instance, you might have to define the coordinate partitioning in this file.

* In __simu_variables.m__, you define all the simulation state variables (i.e. variables whose state will be saved during the whole simulation and which can be accessed via _MBSdata->user_IO_, see below). The way to define these variables is explained at the beginning of this file. These state variables are not available in the controllers designed in the _controller_files_ folder. Thanks to this file, the C files _user_sf_IO.h_ and _user_sf_IO.c_ will be automatically generated in the _StandaloneC/src/project/user_files_ folder. For the Standalone version, its equivalent is called _simu_variables.txt_ (located in the _StandaloneC/src/project/varState_ folder). It has exactly the same effect than _simu_variables.m_, except if the CMake flag 'FLAG_GENERATE_VARSTATE' is deactivated, in which case this file has no effect.

* In __control_variables.m__, you define all the state variables of your different controllers. This file is quite similar to _simu_variables.m_ but with some different rules (read the instructions provided in this file) and is used to acces the fields available in the controller, i.e. using _MBSdata->user_IO->cvs_, see below. Thanks to this file, the C files _ControllersStruct.h_ and _ControllersStruct.c_ will be automatically generated in the _StandaloneC/src/project/controller_files_ folder. For the Standalone version, its equivalent is called _control_variables.txt_ (located in the _StandaloneC/src/project/varState_ folder). It has exactly the same effect than _control_variables.m_, except if the CMake flag 'FLAG_GENERATE_VARSTATE' is deactivated, in which case this file has no effect.

* In __compile_c_files.m__, you only have to adapt the _compile_all_ flag (1 if you always want to recompile the whole project, 0 otherwise), the _prjname_ variable (put your project name), the _debug_ flag (1 to activate the _debug_ mode, 0 otherwise, see this [link](http://www.mathworks.nl/help/matlab/matlab_external/debugging-c-c-language-mex-files.html) for more information) and the flags used for the compilation.

* In __call_simulink.m__, you must define the simulation start and finish times, the time step, the integration solver and the model name (name of the Simulink diagram you will create, without the .mdl extension, see below). This is also the file you can use to plot graphs at the end of the simulation.

* You can use __synchronize_versions.m__ to synchronize a Standalone project from what you did in the Simulink project. Just modify the flags _stand_state_flag_ and _xml_flag_. Then, launch this script. 
	* __stand_state_flag__: if 1, synchronize the equivalents of _simu_variables.m_ and _control_variables.m_ (called _simu_variables.txt_ and _control_variables.txt_ and located in _StandaloneC/src/project/varState_) whith the Matlab version.
	* __xml_flag__: if 1, generate the .mbsdata file used by the Standalone version to initialize the project, loading the initialization information from the .mbs file located in _dataR_. The coordinate partiotionning is also done, provided the file _generate_mbs_data.m_ is correctly configured.


### StandaloneC/src/project ###

All state variables are gathered in a struct: _MBSdataStruct_ (whose instance is called _MBSdata_).
* User state variables only available in simulation are usually accessed via a __UserIOStruct__ called __user_IO__ (acces via MBSdata->user_IO).
* User state variables used in the controller are accessed via a __ControllerStruct__ called __cvs__ (acces via _MBSdata->user_IO->cvs_). These names might differ, according to what you will configure at the end of the _workR/Simulink/simu_variables.m_ (or _StandaloneC/src/project/varState/simu_variables.txt_) file. There might also be more than one controller. 

With Simulink, the fields available in _UserIOStruct_ are the ones implemented in _simu_variables.m_, while the ones available in _ControllerStruct_ are implemented in _control_variables.m_. For the Standalone version, use the corresponding files _simu_variables.txt_ and _control_variables.txt_ (located in the _StandaloneC/src/project/varState_ folder), except if 'FLAG_GENERATE_VARSTATE' is deactivated in which case you must configure these files manually.

* Go to __StandaloneC/src/project__

* Configure the files located in the __user_files__ folder in a similar way to what would be done for a Matlab project in the _userfctR_ folder (see the [Robotran modeling features](http://www.robotran.be/downloads/tutorial/RobotranLearning_ModelingFeatures.pdf)).
	* _user_sf_IO.h_ and _user_sf_IO.c_ are automatically generated according to _simu_variables.m_ (or _simu_variables.txt_ for the Standalone version). Do not modify these two files (except for the Standalone version, if you deactivate the CMake flag 'FLAG_GENERATE_VARSTATE').

* Configure the files located in the __simulation_files__ folder. You can add new files according to your need.
	* __simu_def.h__ is the main header used for these files. You can modify it and add your own new headers. You can also add new C files in the _simulation_files_ folder.
	* __simulink_outputs.c__ is used to get the signals from the simulation environment in the Matlab workspace via the Simulink block (useless for the _Standalone_ version).
	* __simu_controller_loop.c__ is the main loop of the folder _simulation_files_. It is also used to call the controller main loop. You can add new functions to this loop if needed.
	* __simu_in_out.c__ and __simu_in_out.h__ are two files only used by the _Standalone_ version. The user can define the input structure (called _InputSimu_) prototype in _simu_in_out.h_, providing all the input fields available in the simulator (for instance, parameters to optimize). Then, the function _create_InputSimu_ (in _simu_in_out.c_) must be filled to initialize this structure before launching the simulation (the function _free_InputSimu_ must also be filled to free _InputSimu_ at the end of the simulation). In these two files, the user can also configure in a similar way the output structure _OutputSimu_ to get some outputs from the simulation (for instance, the fitness value at the end of an optimization process).
	* __stop_simu.c__ is used to stop the simulation if a particular event (configured by the user) happens.
	* __Simbody__ is a folder used to define the contacts to compute with [Simbody](https://simtk.org/home/simbody/). The Simbody features can only be used with the [Standalone version](StandaloneC/README.md), it is useless for a _Simulink_ project. For more information, consult the [README.md](StandaloneC/src/project/simulation_files/Simbody/README.md) located in StandaloneC/src/project/simulation_files/Simbody.

* Configure the files located in the __controller_files__ folder. You can add new files according to your need.
	* __ControllersStruct.h__ and __ControllersStruct.c__ are automatically generated according to _control_variables.m_ (or _control_variables.txt_ for the Standalone version). Do not modify these two files (except for the Standalone version, if you deactivate the CMake flag 'FLAG_GENERATE_VARSTATE').
	* __controller_def.h__ is the main header used for these files. You can modify it and add your own new headers.
	* __controller_init.c__ is the initialization file of the controller. The _controller_init_ function must be called in your real robot initialization function.
	* __controller_loop.c__ is the main loop of the controller. Add as many functions as needed in this loop. You can also add new C files in the _controller_files_ folder. The _controller_loop_ function must be called in your real robot main loop.
	* If you do not need a controller, leave this folder (controller_files) as it was initially configured.
	* This folder is used to design a controller which is totally independent from the simulation environment. Consequently, do not add information which is not available on the real robot if you want to use the same files you developped in the _controller_files_ folder for the real robot.

* Configure the files located in the __interface_controller__ folder. You can add new files according to your need.
	* __controller_interface.h__ is the main header for this interface.
	* __controller_interface.c__ is the file used to handle the initialization, the loop calls and the shutdown of the controller.
	* __controller_inputs.c__ is used to set the inputs of the controller. This file can be used to simulate the sensors available on the real robot.
	* __controller_outputs.c__ is used to extract the outputs of the controller, modifying the simulation environment according to these outputs. For instance, it can transform the outputs of the controller into voltages feeding different motors implemented in the simulation environment.
	* _controller_inputs.c_ and _controller_outputs.c_ are the two files responsible for making a clear interface between the controller and its environment (included the robot to control). Consequently, these two files must be re-written if you transfer the controller designed in simulation to a real robot (to adapt these interface files).


### Compiling the code ###

Run the script __workR/Simulink/compile_c_files.m__.

You should get a message similar to this one:

_Loading project... MBsysLab project 'project name' has been loaded_.

If it is not the case, your compilation is not successful. There should be error messages. Read them to know what was wrong.


### Simulink diagram ###

* Building this diagram is similar to what is explained in the [Robotran SFunctions](http://www.robotran.be/downloads/tutorial/RobotranLearning_SFunctions.pdf) slides.

* Open Simulink with the Matlab logo.

* Click on the __New model__ button to create a new Simulink diagram.

* In the _Libraries_, go to the _Robotran>Modules folder_, then drag and drop the green __S-Function_dirdynared__ block in your new Simulink diagram.

* When you add this block or when you double-click on it, the _MBSdirdynaredSfunction_ appears.
	* Click on the __Load__ button (this load the mex file compiled with _compile_c_files.m_).
	* Click on the __Coordinate partitioning__ button.

* The _Coordinate partitioning_ button opens a window.
	* Click on the __Check...__ button, this open a new window (click on __OK__).

* Click on the __OK__ button of _MBS Coordinate Partitioning_.

* Click on the __OK__ button of the _MBSdirdynaredSfunction_ window.

* This procedure must only be done when you create your Simulink project, if you transfer this project to another computer (especially if these two computers have different OS systems) or if you modify the inputs/outputs of the green block.

* Let's have a look at this green block:
	* The left part of this block is used for the inputs coming from Matlab.
	* The right part of the green block is used for the outputs. In Sinks, you find the __To Workspace__ blocks, which are used to transfer the outputs of the simulation to the Matlab workspace. Place these blocks for each output of the green block, except for the last one. Note that the first blocks represent the positions, velocities and accelerations (q, _qd_ and _qdd_) while the next ones follow the same order as the one defined in the _simu_vars_out_ structure of the _simu_variables.m_ file. The last one is always the variable _stop_simu_ (beware, it is not indicated !). For this last output, you must place a __Stop Simulation__ block (also located in the _Sinks_) instead of a __To Workspace__ one. For more information about this _stop_simu_ variable, have a look at the _stop_simu.c_ file located in _StandaloneC/src/project/simulation_files_.

* In _Robotran>Utilities_, drag and drop the __To MBS anim file__.
	* In the _MBStoAnimFile_, write the project name and click on __OK__.
	* This block is used to generate the 3D visualization in _MBSysPad_ and only requires the joint positions _q_.
	* Maintain the __Ctrl__ key pressed, and rely the link between _q_ and its _To Workspace_ block to the anim file block.
	* Save your simulink model under the model name (the one you defined in _call_simulink.m_) in the workR file.


### Launching the simulation ###

Just launch the script __call_simulink.m__. At the end, you should get the .anim file generated in the _animationR_ folder with the name _simdir.anim_. You can wrote lines at the end of the _call_simulink.m_ script to analyze the simulation outputs.


### Preparing the controller designed in simulation for a real robot ###

Controller design is organised such that the files developed for the controller can easily be transferred to the real robot.

* You must import the controller files to your real robot controller and adapt the files located in the _StandaloneC/src/project/interface_controller_ folder to create the corresponding interface with the real robot.
* Here are the instructions to do this if you use the Robotran controller template (with _ControllerStruct_):
	* If you designed a controller in simulation in the __controller_files__ folder, you only need the files located in this folder. Add them to your real robot controller. 
	* On top of that, you must define a structure instance of _ControllerStruct_ (depending on the controller name you wrote in _simu_variables.m_ or _simu_variables.txt_), e.g. _ControllerStruct *cvs;_.
	* In the initialization part of your project, add these lines: 
		* _cvs = init_ControllerStruct();_
		* _controller_init(cvs);_
	* In the loop function of your project:
		* Write a new _controller_inputs_ function to get the same result as the one of _controller_inputs.c_, but adapted to your robot real inputs (sensors...).
		* Call the _controller_loop_ function: _controller_loop(cvs);_
		* Write a new _controller_outputs_ function, similar to _controller_outputs.c_, to command your actuators.
	* In the ending function of your project (optional): _free_ControllerStruct(cvs);_
