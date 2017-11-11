[![pipeline status](https://gitlab.com/powan/CoreRobotics/badges/master/pipeline.svg)](https://gitlab.com/powan/CoreRobotics/commits/master)

# CoreRobotics Project
#### Copyright (c) 2017
#### University of Washington
#### College of Engineering

Created: 2016  
Last Updated: October 20, 2017  
Version: 0.9.1

- C++ Platforms: Windows 8.1/10, Mac OS X 10, and Linux.
- MATLAB Version (not active): R2016a (Windows, Mac, Linux)

## Description:
This git project hosts code for the CoreRobotics open source robotic control library.  The CoreRobotics library is distrubted under the [BSD-3-Clause license](https://opensource.org/licenses/BSD-3-Clause).  This allows unlimited redistribution as long as the copyright notices and disclaimers of warranty are maintained.  The third clause indicates permission is required to the use of names of contributors for endorsement in any derived work.


## General Structure:
- *doc/* contains the doxygen Doxyfile for generating html documentation.  Please install [doxygen](http://www.stack.nl/~dimitri/doxygen/) and run the doxyfile to generate.  The main documentation page will be *doc/html/index.html*
- *external/* contains source for external libraries that are needed.  For example, *cpp/external/* should include the Eigen library for matrix math.
- *python/* contains code for the python wrapper.
- *src/* contains the library source and headers.  "CoreRobotics.hpp" is the main header for the project.  Classes and functions are organized into sub folders.
- *utils/* contains code for testing library functionality.  Tests for each class are broken out into test_<classname> and should test functionality of each method.  The test binary gets written to *./bin*  These scripts are also useful for seeing examples of how to use a particular function of the library.


## Building and compiling
CoreRobotics relies on CMake to compile cross-platform source to platform-specific libraries and binaries.  To build and compile CoreRobotics for development, follow these steps:
1. Download CMake for your platform at [https://cmake.org/](https://cmake.org/).  Make sure you have installed for command line use. (See CMake UI: Tools>How to Install for Command Line Use)
2. Open terminal (or emulator) and change directory (`cd <CoreRoboticsRoot>/`) (replace <CoreRoboticsRoot> for your specific setup with the root directory of the project.)
3. Create a folder `mkdir build`.
4. Change directories `cd build`.
5. Run CMake `cmake -G "<compiler>" ../`  To get a list of compilers available to configure with CMake, type `cmake --help`.
6. Open the project in the *build\* directory and build accordingly. (e.g.: For visual studio, a CoreRobotics.sln will be created.  Open this solution and build all.  The library and binaries will be compiled by the Visual Studio IDE.)


## Using the library
When built using your CMake-generated project, the CoreRobotics creates a ./bin folder which contains the binary test scripts.  You should make sure this executes.  To use the library in your own project, you need to do the following (example using CMake):
1. Set the path to your CoreRobotics installation using the CMake `set` command.  
`set (CR_DIR "path/to/corerobotics/root")`
2. Add the following directories to the header search paths.  
`include_directories(
    ${CR_DIR}/src
    ${CR_DIR}/src/core
    ${CR_DIR}/src/math
    ${CR_DIR}/src/models
    ${CR_DIR}/src/physics
    ${CR_DIR}/src/controllers
    ${CR_DIR}/external/eigen
)`
3. Include the following folder in your linker directory (note if the library is built in debug, the path must be updated accordingly to reflect).  By default, the compiler builds a static library.  
`link_directories(${CR_DIR}/lib/Release)`
4. Lastly, you must link the corerobotics library after you add your executable (example is shown for Unix, Windows will be CoreRobotics.lib)  
`target_link_libraries("exec_name" ${CR_DIR}/lib/Release/libCoreRobotics.a)`


## Developer Guidelines:
For general questions about git, see the [Gitlab help pages](https://gitlab.com/help/)

1. Absolutely all class methods must have existing references in peer-reviewed publications (text books, conference papers, journal papers, etc…), and the reference must be included in the source code.  Do not add your fancy algorithms here until you’ve published.  If you’re considering putting something into the library that has a lot of data, it doesn’t belong there - this is a project for implementing generic robotics math, not specific applications.
2. The project makes extensive use of object-oriented programming (class definitions). If you are unsure about what this is, refer to the [C++ Reference](http://www.learncpp.com/cpp-tutorial/81-welcome-to-object-oriented-programming/).
3. Keep the C++ code crossplatform (i.e. don’t use OS-specific functions unless you check the OS and have a method for each OS.  Plan to support Windows 8.1/10, Mac OSX, and Linux Ubuntu.
4. Any general math functions that you think might come in handy in someone else’s work should be included in the Math class.  This is a static method class that acts as a container to hold all the useful math routines.
5. If you use a 3rd party library for your contribution, make sure its license (specifically derived-works) is compatible with the BSD 3-clause license. Include the library source in the *external/* folder and make sure the documentation for that implementation acknowledges the use of the library.
6. Comment your classes for Doxygen.  Following existing code for examples on how to do this.
7. Use the *utils/* folder to write tests for your contirbutions.
8. Please keep the code in working condition.  Only commit changes you've made to code if it is working properly (this means compile and run BEFORE you commit!).

## Brief Style Guide
Refer to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for detailed explanations.
- Use intuitive naming. Use discretion with acronymns or when abbreviating anything that isn’t well known.
- Classes should have `CR` in front of the name, e.g.: `CRTransferFunction`
- Methods and members should use Camel case, e.g.: `setProperty(<args>)`, `getTransferFunction()`
- Use upper case and underscores for enumerators, e.g.: `MY_CRAPPY_NUMERATOR`.
- Class members should be protected, and setting/getting the member value should be defined explicitly.  The exception is when a class member is a type associated with a 3rd party library (this is so one can access the property without having to modify the containing CoreRobotics class)
- All class members should have the `m_` prefix for readibility, e.g.: `m_classProperty`
- Method arguments should utilize the `i_` prefix to indicate an input argument, e.g.: `i_transferFunction`, and the `o_` prefix to indicate an output argument, e.g.: `o_transferFunction`
- Consider passing input arguments by value or pointer and output arguments by reference, e.g.: `CRClass::solveProblem(double i_argInOne, double* i_argInTwo, double& o_argOut)`.  Use discretion when making exceptions.
- When a method returns only one output, use the return, e.g.: `double getProperty(void)`.  If the method has more than one output, pass by function argument, e.g.: `void getProperties(double& o_propOne, double& o_propTwo)`, and reserve the return for indicating a result.
- If a result is returned, use the `CoreRobotics::CRResult` enumerator (contained in `CoreRobotics::CRTypes`).  Add result types as needed and comment which results can be expected by a particular method.


## Git commands for reference:
Some basic git commands through the command line are:
- `git status` check the status of repository.
- `git add "<path/file>"` add the file to the git path.
- `git add .` add all the files you changed in your local directory.
- `git commit -m "<commit message>"` commit the change.  Only do this if your code is working!
- `git push <branch>` pushes the commits to the specified branch.
- `git branch` lists the branches
- `git checkout <branch>` checks out the specified branch
- `git pull` pulls new commits from the server.


Happy developing!


-Parker
