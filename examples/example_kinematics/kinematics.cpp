//=====================================================================
/*
Software License Agreement (BSD-3-Clause License)
Copyright (c) 2017, CoreRobotics.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.

* Neither the name of CoreRobotics nor the names of its contributors
may be used to endorse or promote products derived from this
software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\project CoreRobotics Project
\url     www.corerobotics.org
\author  Parker Owan

*/
//---------------------------------------------------------------------

#include <iostream>
#include <cr/core>
#include <cr/world>

#ifdef USE_OSG
#include <osg/ref_ptr>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Material>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#endif

/// Declare helper functions
void setupRobotKinematics(const cr::world::RobotPtr a_robot,
                          const cr::world::LinkPtr a_link[5],
                          const cr::world::NodePtr a_tool);
void setupFiducial(const cr::world::NodePtr);
void setSceneKinematics(const osg::ref_ptr<osg::MatrixTransform> a_graphicLink[5],
                        const cr::world::LinkPtr a_link[5]);


/// main function
int main( int argc, char **argv )
{
    // Create a new core robotics world
    cr::world::RobotPtr robot = cr::world::Robot::create();
    cr::world::LinkPtr link[5];
    for (std::size_t i = 0; i < 5; ++i) {
        link[i] = cr::world::Link::create();
    }
    cr::world::NodePtr tool = cr::world::Node::create();
    setupRobotKinematics(robot, link, tool);
    
    cr::world::NodePtr node = cr::world::Node::create();
    node->setName("My fiducial");
    setupFiducial(node);
    
    cr::world::OriginPtr world = cr::world::Origin::create();
    world->setName("My world");
    world->addChild(robot);
    world->addChild(node);
    world->print(std::cout);
    
    // Get the Jacobian
    Eigen::MatrixXd J = robot->jacobian(tool, cr::physics::EulerMode::CR_EULER_MODE_XYZ);
    std::cout << "Robot Jacobian \n" << J << std::endl;
    
    
    // examples:
    // https://www.cs.vassar.edu/~cs378/SampleCode.pdf
    // https://github.com/openscenegraph/OpenSceneGraph/blob/master/examples/osgpoints/osgpoints.cpp
    // https://gist.github.com/vicrucann/874ec3c0a7ba4a814bd84756447bc798
    
    // Create a primative shape
    // osg::ref_ptr<osg::Geode> myGeode = new osg::Geode;
    // myGeode->addDrawable(new osg::ShapeDrawable(new  osg::Sphere()));
    
    // create a root node
    osg::ref_ptr<osg::Group> myRoot = new osg::Group;
    
    // create a floor
    // osg::ref_ptr<osg::Geode> myFloor = new osg::Geode;
    // myFloor->addDrawable(new osg::ShapeDrawable(new osg::InfinitePlane()));
    // myRoot->addChild(myFloor.get());
    
    // create a cylinder
    osg::ref_ptr<osg::Geode> myCylinder = new osg::Geode;
    myCylinder->addDrawable(new osg::ShapeDrawable(new osg::Cylinder()));
    
    // change the cylinder material and states
    osg::StateSet* stateSet = myCylinder->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    
    // change the sphere position
    osg::ref_ptr<osg::MatrixTransform> myCylinderTransform = new osg::MatrixTransform;
    osg::Matrixd mat;
    mat.makeScale(osg::Vec3(0.1, 0.1, 1.0));
    myCylinderTransform->setMatrix(mat);
    myCylinderTransform->addChild(myCylinder.get());
    myRoot->addChild(myCylinderTransform.get());
    
    // create a node from file
    // osg::ref_ptr<osg::Node> myNode = osgDB::readNodeFile("Tool.obj");
    
    // change the node position
    // myRoot->addChild(myNode.get());
    
    // create the open scene graph viewer
    osgViewer::Viewer viewer;
    viewer.setSceneData(myRoot);
    viewer.addEventHandler(new osgViewer::StatsHandler);
    viewer.addEventHandler(new osgViewer::WindowSizeHandler);
    // viewer.addEventHandler(new osgGA::StateSetManipulator(viewer.getCamera()->getOrCreateStateSet()));
    viewer.setCameraManipulator(new osgGA::TrackballManipulator());
    // viewer.realize();
    
     
    return viewer.run();
}


/// Setup the 5DOF robot kinematics
void setupRobotKinematics(const cr::world::RobotPtr a_robot,
                          const cr::world::LinkPtr a_link[5],
                          const cr::world::NodePtr a_tool)
{
    cr::physics::FrameEuler frame;
    cr::physics::RigidBody body;
    
    frame.setMode(cr::physics::EulerMode::CR_EULER_MODE_XYZ);
    
    // build the robot kinematics
    cr::world::NodePtr base = cr::world::Node::create();
    base->setName("Base");
    
    a_link[0]->setName("Link 0");
    frame.setFreeVariable(cr::physics::EulerFreeVariable::CR_EULER_FREE_ANG_G);
    frame.setPosition(0.0, 0.0, 0.1);
    a_link[0]->setLocalTransform(frame);
    
    a_link[1]->setName("Link 1");
    frame.setFreeVariable(cr::physics::EulerFreeVariable::CR_EULER_FREE_ANG_B);
    frame.setPosition(0.0, 0.0, 0.2);
    a_link[1]->setLocalTransform(frame);
    
    a_link[2]->setName("Link 2");
    frame.setFreeVariable(cr::physics::EulerFreeVariable::CR_EULER_FREE_ANG_B);
    frame.setPosition(0.0, 0.0, 0.4);
    a_link[2]->setLocalTransform(frame);
    
    a_link[3]->setName("Link 3");
    frame.setFreeVariable(cr::physics::EulerFreeVariable::CR_EULER_FREE_ANG_B);
    frame.setPosition(0.0, 0.0, 0.4);
    a_link[3]->setLocalTransform(frame);
    
    a_link[4]->setName("Link 4");
    frame.setFreeVariable(cr::physics::EulerFreeVariable::CR_EULER_FREE_ANG_G);
    frame.setPosition(0.0, 0.0, 0.1);
    a_link[4]->setLocalTransform(frame);
    
    a_tool->setName("Tool");
    frame.setFreeVariable(cr::physics::EulerFreeVariable::CR_EULER_FREE_NONE);
    frame.setPosition(0.0, 0.0, 0.6);
    a_tool->setLocalTransform(frame);
    
    // tree graph
    a_robot->addChild(base);
    base->addChild(a_link[0]);
    a_link[0]->addChild(a_link[1]);
    a_link[1]->addChild(a_link[2]);
    a_link[2]->addChild(a_link[3]);
    a_link[3]->addChild(a_link[4]);
    a_link[4]->addChild(a_tool);
    
    // important! add the links to the robot to build the chain,
    // this allows us to use the configuration space methods.
    a_robot->addLink(a_link[0]);
    a_robot->addLink(a_link[1]);
    a_robot->addLink(a_link[2]);
    a_robot->addLink(a_link[3]);
    a_robot->addLink(a_link[4]);
}


/// Setup the fiducial node location
void setupFiducial(const cr::world::NodePtr a_node)
{
    cr::physics::Frame frame;
    Eigen::Vector3d p;
    p << 0.5, 0, 0.5;
    frame.setTranslation(p);
    a_node->setLocalTransform(frame);
}


/// Convert from OSG to a 3x3 matrix of doubles.
inline const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> fromOsg(const osg::Matrix3d& matrix)
{
    return Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(matrix.ptr());
}


/// Convert a fixed-size 3x3 matrix of doubles to OSG.
template <int MOpt> inline
    const osg::Matrix3d toOsg(const Eigen::Matrix<double, 3, 3, MOpt>& matrix)
{
    osg::Matrix3d osgMatrix;
    Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::ColMajor>>(osgMatrix.ptr()) = matrix;
    return osgMatrix;
}


/// Convert from OSG to a 4x4 matrix of doubles.
inline const Eigen::Matrix<double, 4, 4, Eigen::RowMajor> fromOsg(const osg::Matrixd& matrix)
{
    return Eigen::Map<const Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(matrix.ptr());
}


/// Convert a fixed-size 4x4 matrix of doubles to OSG.
template <int MOpt> inline
const osg::Matrixd toOsg(const Eigen::Matrix<double, 4, 4, MOpt>& matrix)
{
    osg::Matrixd osgMatrix;
    Eigen::Map<Eigen::Matrix<double, 4, 4, Eigen::ColMajor>>(osgMatrix.ptr()) = matrix;
    return osgMatrix;
}


/// Push the core robotics transforms to the osg graphic
void setSceneKinematics(const osg::ref_ptr<osg::MatrixTransform> a_graphicLink[5],
                        const cr::world::LinkPtr a_link[5])
{
    // change the sphere position
    // double angle_in_rads = 0.0;
    // osg::Vec3 axis_of_rot_vec(0.0, 0.0, 1.0);
    // osg::ref_ptr<osg::MatrixTransform> myCylinderTransform = new osg::MatrixTransform;
    // osg::ref_ptr<osg::PositionAttitudeTransform> myCylinderTransform = new osg::PositionAttitudeTransform;
    // Eigen::Vector3d p;
    // p = robot->getGlobalTransform().getTranslation();
    // myCylinderTransform->setPosition(osg::Vec3(p(0),p(1),p(2)));
    // myCylinderTransform->setAttitude(osg::Quat(angle_in_rads, axis_of_rot_vec));
    osg::Matrixd mat;
    // mat.makeScale(osg::Vec3(0.1, 0.1, 0.1));
    // a_graphicLink[0]->setMatrix(mat);
    // myCylinderTransform->setScale(osg::Vec3(0.1, 0.1, 1.0));
    //  a_graphicLink[0]->addChild(myCylinder.get());
    // myRoot->addChild(a_graphicLink[0].get());
}
