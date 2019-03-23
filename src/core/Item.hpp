/*
 * Copyright (c) 2017-2019, CoreRobotics
 * www.corerobotics.org
 * Licensed under BSD-3, https://opensource.org/licenses/BSD-3-Clause 
 */

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
    
        //! return the item type
        std::string getType() { return m_type; }
    
    
    // Protected members
    protected:
    
        //! name
		std::string m_name;

		//! icon
		std::string m_icon;
    
        //! type (read only)
        std::string m_type = "Generic";
};
    
}
}
// end namespace
//---------------------------------------------------------------------

#endif
