## Guidelines
For general questions about git, see the [Gitlab help pages](https://gitlab.com/help/)

1. Class methods must have existing references in peer-reviewed publications (text books, conference papers, journal papers, etc…), and the reference must be included in the source code.  Do not add your fancy algorithms here until you’ve published.  If you’re considering putting something into the library that has a lot of data, it doesn’t belong there - this is a project for implementing generic robotics math, not specific applications.
2. The project makes extensive use of object-oriented programming (class definitions). If you are unsure about what this is, refer to the [C++ Reference](http://www.learncpp.com/cpp-tutorial/81-welcome-to-object-oriented-programming/).
3. Keep the C++ code crossplatform (i.e. don’t use OS-specific functions unless you check the OS and have a method for each OS.  Plan to support Windows 10, Mac OSX, and Ubuntu.
4. Any general math functions that you think might come in handy in someone else’s work should be included in the Math classes.  These are static method classes that act as containeres for useful math routines.
5. If you use a 3rd party library for your contribution, make sure its license (specifically derived-works) is compatible with the BSD 3-clause license. Document that it is a depedency.
6. Document in your classes for Doxygen.  Following existing code for examples on how to do this.
7. Write tests while you write your class.
8. Make sure tests pass before you commit to a main branch.
9. Use the gitlab issue tracker for communicating feature requests and issues.


## Style Guide
Refer to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html) for detailed explanations.
- Use intuitive naming. Use discretion with acronymns or when abbreviating anything that isn’t well known.
- New classes should not use `CR` in front of the name, e.g.: `TransferFunction`
- Methods and members should use Camel case, e.g.: `setProperty(<args>)`, `getTransferFunction()`
- Use upper case and underscores for enumerators, e.g.: `MY_CLEVER_ENUMERATOR`.
- Class members should be protected, and setting/getting the member value should be defined explicitly.  The exception is when a class member is a type associated with a 3rd party library (this is so one can access the property without having to modify the containing CoreRobotics class)
- Class members should have the `m_` prefix for readibility, e.g.: `m_property`
- Method arguments should utilize the `i_` prefix to indicate an input argument, e.g.: `i_transferFunction`, and the `o_` prefix to indicate an output argument, e.g.: `o_transferFunction`
- Consider passing input arguments by value or pointer and output arguments by reference, e.g.: `MyClass::solveProblem(double i_argInOne, double* i_argInTwo, double& o_argOut)`.  Use discretion when making exceptions.
- When a method returns only one output, use the return, e.g.: `double getProperty(void)`.  If the method has more than one output, pass by function argument, e.g.: `void getProperties(double& o_propOne, double& o_propTwo)`, and reserve the return for indicating a result.
- If a result is returned, use the `cr::core::Result` enumerator.  Add result types as needed and comment which results can be expected by a particular method.
- Use smart points over raw pointers, e.g. `using MyClassPtr = std::shared_ptr<MyClass>`
- Refer to previous files for other style reference.


## Git reference
Some basic git commands through the command line are:
- `git status` check the status of repository.
- `git add "<path/file>"` add the file to the git path.
- `git add .` add all the files you changed in your local directory.
- `git commit -m "<commit message>"` commit the change.  Only do this if your code is working!
- `git push <branch>` pushes the commits to the specified branch.
- `git branch` lists the branches
- `git checkout <branch>` checks out the specified branch
- `git pull` pulls new commits from the server.
