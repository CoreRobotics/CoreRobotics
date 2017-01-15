# CoreRobotics Open Source Project
#### University of Washington
#### College of Engineering
Created: January 14, 2016

Currently in beta.
Platforms: Windows 8.1, Mac OS X 10, and Linux.

## Description:
This git project hosts code for the CoreRobotics open source robotic control library.  The CoreRobotics library is distrubted under the [BSD-3-Clause license](https://opensource.org/licenses/BSD-3-Clause).  This allows unlimited redistribution as long as the copyright notices and disclaimers of warranty are maintained.  The third clause indicates permission is required to the use of names of contributors for endorsement in any derived work.


## General Structure:
The project is split into 2 general folders: *matlab/* for the MATLAB library and *cpp/* for the C++ library.  The structure for both folders is similar, with a few differences due to the languages.
- *bin/* contains binary (compiled) files.  This folder does not exist in the *matlab/* version because it is an interpreted language.
- *doc/* contains html documentation.  html files can be compiled from .mlx files within MATLAB, while the C++ version should use [doxygen](http://www.stack.nl/~dimitri/doxygen/).
- *examples/* contains example applications.
- *external/* contains source for external libraries that are needed.  For example, *cpp/external/* should include the Eigen library for matrix math.
- *include/* contains headers for interfacing with library classes.  This is only in the *cpp/* version.
- *src/* (MATLAB: *+CoreRobotics/*) contains the library source code.
- *utils/* contains utilities for developers, such as performance evaluations, testing and document generation.


## Developer Guidelines:
1. Absolutely all class methods must have existing references in peer-reviewed publications (text books, conference papers, journal papers, etc…), and the reference must be included in the source code.  Do not add your fancy algorithms here until you’ve published.  If you’re considering putting something into the library that has a lot of data, it doesn’t belong there - this is a project for implementing generic robotics math, not specific applications.
2. Please keep everything in object-oriented form (class definitions). If you are unsure about what this is, refer to the [C++ Reference](http://www.learncpp.com/cpp-tutorial/81-welcome-to-object-oriented-programming/) or the [MATLAB Documentation](https://www.mathworks.com/discovery/object-oriented-programming.html).
3. Keep the C++ code crossplatform (i.e. don’t use OS-specific functions unless you check the OS and have a method for each OS.  Plan to support Windows 8.1, Mac OS, and Linux Ubuntu.
4. Please use handle classes (MATLAB).  You can do this by inheriting the InputHandler.m handle class, which automatically makes the derived class a handle class.  This class provides access to methods for consistent input handling.
5. Any general math functions that you think might come in handy in someone else’s work should be included in the Math class.  This is a static method class that acts as a container to hold all the useful math routines.  No need to make the math method an object, just call the method directly (MATLAB e.g.: `output = CoreRobotics.Math.<fancyMethod>(args)`)
6. Please use only functions available in the basic distribution of the corresponding language. This is particularly important for MATLAB - a lot of people using this library may not have purchased niche MATLAB toolboxes.  If you must absolutely include a particular toolbox, make sure to make it clear in the documentation that the user needs the toolbox for that class to work.
7. Maintain consistent documentation in the master.  I will only merge code to the master with sufficient documentation.
8. Please keep the code in working condition.  Only commit changes you've made to code if it is working properly.  **_DO NOT COMMIT BROKEN CODE._**  This is important if we ever have to revert to a prior release (hopefully will never happen).


## Brief Style Guide
Refer to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for a more detailed explanations.
- Use intuitive method and property names, don’t abbreviate anything that isn’t well known.  Use discretion with acronymns.
- Use Camel Case for method/function names, e.g.: `MySweetMethod()` or `MyLqrSolver()`.
- Use lower case and underscores for property/variable names, e.g.: `my_lame_property`.
- Use upper case and underscores for enumerators, e.g.: `MY_OK_NUMERATOR`.
- Use name/Value pairs for MATLAB constructor inputs, e.g.: `obj = MyClass(name,value)`.
- Use `set()` methods for all public properties to check input validity.  See native MATLAB [set methods](https://www.mathworks.com/help/matlab/matlab_oop/property-set-methods.html) and [get methods](https://www.mathworks.com/help/matlab/matlab_oop/property-get-methods.html) for more info.


## Git commands for reference:
Some basic git commands through the command line are:
- `git status` check the status of repository.
- `git add "<path/file>"` add the file to the git path.
- `git add .` add all the files you changed in your local directory.
- `git commit -m "<commit message>"` commit the change.  Only do this if your code is working!
- `git push origin master` pushes the commits to the server.
- `git pull` pulls new commits from the server.
- `git tag <tag_name>` Gives a commit a name that you can reference specifically. These should be rare.
- `git checkout <tag_name>` Equivalent of a git pull, only you pull the commit that the tag corresponds to (which may not be the latest version).


Happy developing!


-Parker
