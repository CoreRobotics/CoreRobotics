---
title: "CoreRobotics: An object-oriented C++ library with cross-language wrappers for cross-platform robot control"
tags:
- robotics
- control
- realtime
authors:
- name: Parker Owan
  orcid: 0000-0003-1160-0426
  affiliation: 1
- name: Cameron Devine
  affiliation: 1
- name: W. Tony Piaskowy
  affiliation: 1
affiliations:
- name: University of Washington
  index: 1
date: 21 November 2017
bibliography: paper.bib
---

# Summary

Real-time controllers for robot manipulators are typically developed and implemented on a custom basis due to 1) the complexity of control in real application, and 2) requirement of only incremental changes in hardware and software as the design matures.  Modular approaches to actuators have made it easier to quickly assemble custom robot designs to address the increasing thrust for physical automation in society.  However, development of controllers for such robot platforms is often performed in resource-intensive software suites such as Robot Operating System (ROS) [@ros].  This approach requires unnecessarily high processor performance when the controller does not fully utilize the suite.

The CoreRobotics libraries were developed in an effort to provide generalized implementations of algorithms facilitating rapid development of real-time robot control. CoreRobotics utilizes an object-oriented approach in C++ to implement fast cross-platform thread management and timing, core math solvers [@kreyszig], manipulator control [@craig,@murray,@DLS,@NS,@hessian], and trajectory shaping [@minJerk], and modeling for state estimation [@thrun,@crassidis,@smc].

CoreRobotics has been compiled on Windows 8.1, 10, Linux, and MacOS on various hardware architectures.  Linear algebra is handled with Eigen [@eigen].  CMake [@cmake] is used to unify the compile process across multi-platform developer environments, and an option is provided to compile Python and MATLAB wrappers using SWIG [@swig].  The CoreRobotics library is used in several research projects at the University of Washington, Seattle.

The authors would like to thank Professor Santosh Devasia and Professor Joseph Garbini for their support and direction during development of the software library.

# References
