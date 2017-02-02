//
//  CRSignalType.hpp
//  testStuite
//
//  Created by Parker Owan on 2/1/17.
//  Copyright © 2017 CoreRobotics. All rights reserved.
//

#ifndef CRSignalType_hpp
#define CRSignalType_hpp

//=====================================================================
// CoreRobotics namespace
namespace CoreRobotics {
    
//=====================================================================
/*!
 \file CRSignalType.hpp
 \brief Signal type enumerator.
 */
//=====================================================================
//! Enumerator for signal types
enum CRSignalType {
    CR_SIGNAL_FORCE,
    CR_SIGNAL_POSITION,
    CR_SIGNAL_VELOCITY,
    CR_SIGNAL_ACCELERATION,
    CR_SIGNAL_GENERIC
};
//=====================================================================
// End namespace
}
#endif /* CRSignalType_hpp */
