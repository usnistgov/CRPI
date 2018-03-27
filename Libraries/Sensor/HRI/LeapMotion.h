///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Human-Robot Interaction
//  Workfile:        LeapMotion.h
//  Revision:        1.0 - 11 September, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Interface wrapper for the Leap Motion
//
///////////////////////////////////////////////////////////////////////////////

#ifndef LEAPMOTION_H
#define LEAPMOTION_H

#include <vector>
#include <string.h>

#if defined(_MSC_VER)
#include <../../portable.h>
#include "MatrixMath.h"
#include "Leap.h"
#include "ulapi.h"
#else
#include "../../../portable.h"
#include "../../Math/MatrixMath.h"
#include "../../ThirdParty/LeapSDK/include/Leap.h"
#include "../../ulapi/src/ulapi.h"
#endif



using namespace Leap;
using namespace Math;

namespace Sensor
{
  class CrpiListener : public Listener {
    public:
      virtual void onInit(const Controller&);
      virtual void onConnect(const Controller&);
      virtual void onDisconnect(const Controller&);
      virtual void onExit(const Controller&);
      virtual void onFrame(const Controller&);
      virtual void onFocusGained(const Controller&);
      virtual void onFocusLost(const Controller&);
      virtual void onDeviceChange(const Controller&);
      virtual void onServiceConnect(const Controller&);
      virtual void onServiceDisconnect(const Controller&);
    private:
  };


  //! @ingroup Sensor
  //!
  //! @brief   Leap Motion interface Class
  //!
  class LIBRARY_API LeapMotion
  {
  public:

    //! @brief Default constructor
    //!
    LeapMotion ();

    //! @brief Default destructor
    //!
    ~LeapMotion ();
    
    //! @brief Hand model accessor
    //!
    bool getHands (HandList *hands);

    //! @brief Gesture model accessor
    //!
    bool getGestures (GestureList *gestures);

    //! @brief Tool (long, slender object) model accessor
    //!
    bool getTools (ToolList *tools);

    //! @brief Access the raw image stream from the two cameras
    //!
    //! @param left   The image from the left camera
    //! @param right  The image from the right camera
    //! @param dewarp Whether or not to compensate for warping caused by the camera lenses
    //!
    //! @return True if method completes successfully, False otherwise
    //!
    bool getImages (Math::matrix& left, Math::matrix& right, bool dewarp = true);

    //! @brief Depth map accessor
    //!
    //! @param out 
    //!
    //! @return True if the function executes properly, False otherwise
    //!
    bool getDepthMap (Math::matrix& mapout);

  private:
    //! @brief Device controller
    //!
    Controller *controller_;

    //! @brief Device callback listener
    //!
    CrpiListener *listener_;

    //! @brief Collection of identified hands
    //!
    HandList hands_;

    //! @brief Collection of identified gestures
    //!
    GestureList gestures_;

    //! @brief Collection of identified tools (long, slender objects)
    //!
    ToolList tools_;

    //! @brief Raw image information from the Leap cameras
    //!
    ImageList images_;

  }; // LeapMotion
} // Sensor namespace

#endif
