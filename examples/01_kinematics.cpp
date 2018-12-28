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
#include "CoreRobotics.hpp"

#ifdef USE_OSG
#include <osg/ref_ptr>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/StateSet>
#include <osg/Material>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#endif


// Use the CoreRobotics namespace
using namespace cr;


// main function
int main( int argc, char **argv )
{
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
    osg::ref_ptr<osg::Geode> mySphere = new osg::Geode;
    mySphere->addDrawable(new osg::ShapeDrawable(new osg::Cylinder()));
    
    // change the sphere material and states
    osg::StateSet* stateSet = mySphere->getOrCreateStateSet();
    osg::Material* material = new osg::Material;
    material->setColorMode( osg::Material::AMBIENT_AND_DIFFUSE );
    stateSet->setAttributeAndModes( material, osg::StateAttribute::ON );
    stateSet->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    
    // change the sphere position
    double xdist = 0.0;
    double ydist = 1.0;
    double zdist = 1.0;
    osg::ref_ptr<osg::PositionAttitudeTransform> mySphereTransform = new osg::PositionAttitudeTransform;
    mySphereTransform->setPosition(osg::Vec3(xdist, ydist, zdist));
    // mySphereTransform->setScale(osg::Vec3(1.0, 1.0, 1.0));
    // mySphereTransform->setAttitude(osg::Quat(angle_in_rads, axis_of_rot_vec));
    mySphereTransform->addChild(mySphere.get());
    myRoot->addChild(mySphereTransform.get());
    
    // create a node from file
    osg::ref_ptr<osg::Node> myNode = osgDB::readNodeFile("Tool.obj");
    
    // change the node position
    myRoot->addChild(myNode.get());
    
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
