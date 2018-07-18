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
// Begin header definition

#ifndef CR_ITEM_HPP_
#define CR_ITEM_HPP_

#include <string>
#include <iostream>

//---------------------------------------------------------------------
// Begin namespace
namespace cr {
namespace core {
    
//---------------------------------------------------------------------
/*!
 \class Item
 \ingroup core
 
 \brief This abstract class defines common properties and methods for
 classes that have names, icons, and loadable properties, etc...
 */
//---------------------------------------------------------------------
class Item {
    
    // Constructor and destructor
    public:
    
        //! constructor
		Item() {}
    
        //! destructor
        virtual ~Item() {}
    
    
    // Get/set common properties
    public:
    
        //! set the item name
		virtual void setName(std::string i_name) { m_name = i_name; }

		//! return the item name
		std::string getName() { return m_name; }
    
		//! set the item icon
		virtual void setIcon(std::string i_icon) { m_icon = i_icon; }

		//! return the item icon
		std::string getIcon() { return m_icon; }
    
    
    // Protected members
    protected:
    
        //! name
		std::string m_name;

		//! icon
		std::string m_icon;
};
    
}
}
// end namespace
//---------------------------------------------------------------------

#endif
