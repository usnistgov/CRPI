///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Leap Motion Sensor
//  Workfile:        LeapMotion.cpp
//  Revision:        1.0 - 20 November, 2015
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simplified interface library for the Leap Motion sensor system
//
///////////////////////////////////////////////////////////////////////////////


#include <iostream>
#include <cstring>
#include "LeapMotion.h"


#include "opencv2/core/core.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/contrib/contrib.hpp"
#include <stdio.h>

//#define LEAP_NOISY

using namespace cv;
using namespace std;
using namespace Leap;

namespace Sensor
{
  const string fingerNames[] = {"Thumb", "Index", "Middle", "Ring", "Pinky"};
  const string boneNames[] = {"Metacarpal", "Proximal", "Middle", "Distal"};
  const string stateNames[] = {"STATE_INVALID", "STATE_START", "STATE_UPDATE", "STATE_END"};

  ulapi_mutex_struct* datamutex;
  Frame globalFrame;

  void CrpiListener::onInit(const Controller& controller)
  {
#ifdef LEAP_NOISY
    cout << "Initialized" << endl;
#endif
  }

  void CrpiListener::onConnect(const Controller& controller)
  {
#ifdef LEAP_NOISY
    cout << "Connected" << endl;
#endif
    controller.enableGesture(Gesture::TYPE_CIRCLE);
    controller.enableGesture(Gesture::TYPE_KEY_TAP);
    controller.enableGesture(Gesture::TYPE_SCREEN_TAP);
    controller.enableGesture(Gesture::TYPE_SWIPE);
  }

  void CrpiListener::onDisconnect(const Controller& controller)
  {
#ifdef LEAP_NOISY
    cout << "Disconnected" << endl;
#endif
  }

  void CrpiListener::onExit(const Controller& controller)
  {
#ifdef LEAP_NOISY
    cout << "Exited" << endl;
#endif
  }

  void CrpiListener::onFrame(const Controller& controller)
  {
    static bool firstrun = true;

    //! Get the most recent frame
    ulapi_mutex_take (datamutex);
    globalFrame = controller.frame();
    ulapi_mutex_give (datamutex);

#ifdef LEAP_NOISY
    //! Report basic frame information
    cout << "Frame id: " << globalFrame.id()
         << ", timestamp: " << globalFrame.timestamp()
         << ", hands: " << globalFrame.hands().count()
         << ", extended fingers: " << globalFrame.fingers().extended().count()
         << ", tools: " << globalFrame.tools().count()
         << ", gestures: " << globalFrame.gestures().count() << endl;
#endif
  }


  void CrpiListener::onFocusGained(const Controller& controller)
  {
#ifdef LEAP_NOISY
    cout << "Focus Gained" << endl;
#endif
  }

  void CrpiListener::onFocusLost(const Controller& controller) 
  {
#ifdef LEAP_NOISY
    cout << "Focus Lost" << endl;
#endif
  }

  void CrpiListener::onDeviceChange(const Controller& controller) 
  {
#ifdef LEAP_NOISY
    cout << "Device Changed" << endl;
#endif
    const DeviceList devices = controller.devices();

    for (int i = 0; i < devices.count(); ++i)
    {
#ifdef LEAP_NOISY
      cout << "id: " << devices[i].toString() << endl;
      cout << "  isStreaming: " << (devices[i].isStreaming() ? "true" : "false") << endl;
#endif
    } // for (int i = 0; i < devices.count(); ++i)
  }

  void CrpiListener::onServiceConnect(const Controller& controller) 
  {
#ifdef LEAP_NOISY
    cout << "Service Connected" << endl;
#endif
  }


  void CrpiListener::onServiceDisconnect(const Controller& controller)
  {
#ifdef LEAP_NOISY
    cout << "Service Disconnected" << endl;
#endif
  }



  LIBRARY_API LeapMotion::LeapMotion ()
  {
    datamutex = ulapi_mutex_new(101);
    listener_ = new CrpiListener();
    controller_ = new Controller();

    //! Configure the controller to receive events from the controller
    controller_->addListener(*listener_);
    controller_->setPolicy(Controller::POLICY_BACKGROUND_FRAMES);
    controller_->setPolicy(Controller::POLICY_IMAGES);

  }


  LIBRARY_API LeapMotion::~LeapMotion()
  {
    controller_->removeListener(*listener_);

    delete listener_;
    delete controller_;
  }


  LIBRARY_API bool LeapMotion::getHands (HandList *hands)
  {
    ulapi_mutex_take (datamutex);
    hands_ = globalFrame.hands();
    ulapi_mutex_give (datamutex);

    *hands = hands_;

#ifdef LEAP_NOISY
    for (HandList::const_iterator hl = hands_.begin(); hl != hands_.end(); ++hl)
    {
      //! Get the first hand
      const Hand hand = *hl;
      string handType = hand.isLeft() ? "Left hand" : "Right hand";
      cout << string(2, ' ') << handType << ", id: " << hand.id()
           << ", palm position: " << hand.palmPosition() << endl;
      //! Get the hand's normal vector and direction
      const Leap::Vector normal = hand.palmNormal();
      const Leap::Vector direction = hand.direction();

      //! Calculate the hand's pitch, roll, and yaw angles
      cout << string(2, ' ') <<  "pitch: " << direction.pitch() * RAD_TO_DEG << " degrees, "
           << "roll: " << normal.roll() * RAD_TO_DEG << " degrees, "
           << "yaw: " << direction.yaw() * RAD_TO_DEG << " degrees" << endl;

      //! Get the Arm bone
      Arm arm = hand.arm();
      cout << string(2, ' ') <<  "Arm direction: " << arm.direction()
           << " wrist position: " << arm.wristPosition()
           << " elbow position: " << arm.elbowPosition() << endl;

      // Get fingers
      const FingerList fingers = hand.fingers();
      for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl)
      {
        const Finger finger = *fl;
        cout << string(4, ' ') <<  fingerNames[finger.type()]
             << " finger, id: " << finger.id()
             << ", length: " << finger.length()
             << "mm, width: " << finger.width() << endl;
        // Get finger bones
        for (int b = 0; b < 4; ++b) 
        {
          Bone::Type boneType = static_cast<Bone::Type>(b);
          Bone bone = finger.bone(boneType);
          cout << string(6, ' ') <<  boneNames[boneType]
               << " bone, start: " << bone.prevJoint()
               << ", end: " << bone.nextJoint()
               << ", direction: " << bone.direction() << endl;
        } // for (int b = 0; b < 4; ++b)
      } // for (FingerList::const_iterator fl = fingers.begin(); fl != fingers.end(); ++fl)
    } // for (HandList::const_iterator hl = hands_.begin(); hl != hands_.end(); ++hl)
#endif

    return false;
  }


  LIBRARY_API bool LeapMotion::getGestures (GestureList *gestures)
  {
    ulapi_mutex_take (datamutex);
    gestures_ = globalFrame.gestures();
    ulapi_mutex_give (datamutex);
    
    *gestures = gestures_;

#ifdef LEAP_NOISY_X
    // Get gestures
    for (int g = 0; g < gestures_.count(); ++g)
    {
      Gesture gesture = gestures_[g];

      switch (gesture.type())
      {
        case Gesture::TYPE_CIRCLE:
        {
          CircleGesture circle = gesture;
          string clockwiseness;

          if (circle.pointable().direction().angleTo(circle.normal()) <= PI/2)
          {
            clockwiseness = "clockwise";
          }
          else
          {
            clockwiseness = "counterclockwise";
          }

          // Calculate angle swept since last frame
          float sweptAngle = 0;
          if (circle.state() != Gesture::STATE_START)
          {
            CircleGesture previousUpdate = CircleGesture(controller.frame(1).gesture(circle.id()));
            sweptAngle = (circle.progress() - previousUpdate.progress()) * 2 * PI;
          }
          cout << string(2, ' ')
               << "Circle id: " << gesture.id()
               << ", state: " << stateNames[gesture.state()]
               << ", progress: " << circle.progress()
               << ", radius: " << circle.radius()
               << ", angle " << sweptAngle * RAD_TO_DEG
               <<  ", " << clockwiseness << endl;
          break;
        }
        case Gesture::TYPE_SWIPE:
        {
          SwipeGesture swipe = gesture;
          cout << string(2, ' ')
               << "Swipe id: " << gesture.id()
               << ", state: " << stateNames[gesture.state()]
               << ", direction: " << swipe.direction()
               << ", speed: " << swipe.speed() << endl;
          break;
        }
        case Gesture::TYPE_KEY_TAP:
        {
          KeyTapGesture tap = gesture;
          cout << string(2, ' ')
               << "Key Tap id: " << gesture.id()
               << ", state: " << stateNames[gesture.state()]
               << ", position: " << tap.position()
               << ", direction: " << tap.direction()<< endl;
          break;
        }
        case Gesture::TYPE_SCREEN_TAP:
        {
          ScreenTapGesture screentap = gesture;
          cout << string(2, ' ')
               << "Screen Tap id: " << gesture.id()
               << ", state: " << stateNames[gesture.state()]
               << ", position: " << screentap.position()
               << ", direction: " << screentap.direction()<< endl;
          break;
        }
        default:
          cout << string(2, ' ')  << "Unknown gesture type." << endl;
          break;
      } // switch (gesture.type())
    } // for (int g = 0; g < gestures.count(); ++g)

    if (!globalFrame.hands().isEmpty() || !gestures_.isEmpty())
    {
      cout << endl;
    }
#endif

    return false;
  }


  LIBRARY_API bool LeapMotion::getTools (ToolList *tools)
  {
    ulapi_mutex_take (datamutex);
    tools_ = globalFrame.tools();
    ulapi_mutex_give (datamutex);

    *tools = tools_;

#ifdef LEAP_NOISY
    // Get tools
    for (ToolList::const_iterator tl = tools_.begin(); tl != tools_.end(); ++tl)
    {
      const Tool tool = *tl;
      cout << string(2, ' ') <<  "Tool, id: " << tool.id()
           << ", position: " << tool.tipPosition()
           << ", direction: " << tool.direction() << endl;
    } // for (ToolList::const_iterator tl = tools.begin(); tl != tools.end(); ++tl)
#endif
    return false;
  }


  LIBRARY_API bool LeapMotion::getImages (Math::matrix& left, Math::matrix& right, bool dewarp)
  {
    ulapi_mutex_take (datamutex);
    images_ = globalFrame.images();
    ulapi_mutex_give (datamutex);

    Image srcL, srcR;
    srcR = images_[0];
    srcL = images_[1];

    if (!srcL.isValid())
    {
      return false;
    }

    if (!dewarp)
    {
      left.resize(srcR.height(), srcR.width());
      right.resize(srcL.height(), srcL.width());

    }
    else
    {
      int targetWidth = 400;
      int targetHeight = 400;

      left.resize(targetHeight, targetWidth);
      right.resize(targetHeight, targetWidth);
      unsigned char brightness;

      Leap::Vector pixel;
      Vec3b color;

      for (int x = 0; x < targetWidth; ++x)
      {
        for (int y = 0; y < targetHeight; ++y)
        {
          Leap::Vector vec((float)x/targetWidth, (float)y/targetHeight, 0);
          vec.x = (vec.x - srcL.rayOffsetX()) / srcL.rayScaleX();
          vec.y = (vec.y - srcL.rayOffsetY()) / srcL.rayScaleY();

          //! Dewarp left image
          pixel = srcL.warp(vec);
          if(pixel.x >= 0 && pixel.x < srcL.width() && pixel.y >= 0 && pixel.y < srcL.height()) 
          {
            int data_index = (int)(floor(pixel.y) * srcL.width() + floor(pixel.x)); //xy to buffer index
            brightness = srcL.data()[data_index]; //Look up brightness value
            brightness = (brightness == 0 ? 1 : brightness);
          } 
          else
          {
            brightness = 0; //! Display invalid pixels as black
          }
          left.at(y, x) = brightness;

          //! Dewarp right image
          pixel = srcR.warp(vec);
          if(pixel.x >= 0 && pixel.x < srcR.width() && pixel.y >= 0 && pixel.y < srcR.height()) 
          {
            int data_index = (int)(floor(pixel.y) * srcR.width() + floor(pixel.x)); //xy to buffer index
            brightness = srcR.data()[data_index];
            brightness = (brightness == 0 ? 1 : brightness);
          } 
          else
          {
            brightness = 0;
          }
          right.at(y, x) = brightness;
        } // for (int y = 0; y < targetHeight; ++y)
      } // for (int x = 0; x < targetWidth; ++x)
    } // if (!dewarp) ... else

    return true;
  }


  LIBRARY_API bool LeapMotion::getDepthMap (Math::matrix& mapout)
  {
    Math::matrix left, right;

    if (!getImages(left, right))
    {
      return false;
    }

    Mat gL, gR;
    Mat disp, disp8;

    //Draw the undistorted image using the warp() function
    int targetWidth = left.cols;
    int targetHeight = left.rows;
    Mat tIR(targetWidth, targetHeight, CV_8UC1),
        tIL(targetWidth, targetHeight, CV_8UC1);

    for (int x = 0; x < targetWidth; ++x)
    {
      for (int y = 0; y < targetHeight; ++y)
      {
        tIL.at<unsigned char>(y, x) = (unsigned char)left.at(y, x);
        tIR.at<unsigned char>(y, x) = (unsigned char)right.at(y, x);
      }
    }
#ifdef LEAP_NOISY
    imshow("unwarped left", tIL);
    imshow("unwarped right", tIR);
#endif

    Vec3b color;


    StereoBM sbm;
    sbm.state->SADWindowSize = 7;
    sbm.state->numberOfDisparities = 112;
    sbm.state->preFilterSize = 5;
    sbm.state->preFilterCap = 39; //61
    sbm.state->minDisparity = -80; //-80
    sbm.state->textureThreshold = 607; // 507
    sbm.state->uniquenessRatio = 8;
    sbm.state->speckleWindowSize = 0;
    sbm.state->speckleRange = 8;
    sbm.state->disp12MaxDiff = 1;
    
    if (mapout.rows != targetHeight || mapout.cols != targetWidth)
    {
      mapout.resize(targetWidth, targetHeight);
    }

    sbm(tIL, tIR, disp);
#ifdef LEAP_NOISY
    Mat out(targetWidth, targetHeight, CV_8UC3);
#endif
    short val;
    for (int x = 0; x < targetWidth; ++x)
    {
      for (int y = 0; y < targetHeight; ++y)
      {
        val = (((disp.at<short>(x, y) < -350) || (disp.at<short>(x, y) > 150)) ? -1 : abs((disp.at<short>(x, y)+350)/2));
        if (val < 0)
        {
#ifdef LEAP_NOISY
          disp.at<short>(x, y) = -1000;
          color[0] = 255;
          color[1] = color[2] = 0;
#endif
          mapout.at(x, y) = -1;
        }
        else
        {
#ifdef LEAP_NOISY
          color[0] = color[1] = color[2] = (unsigned int)val;
#endif
          mapout.at(x, y) = val;
        }
#ifdef LEAP_NOISY
        out.at<Vec3b>(Point(y, x)) = color;
#endif
      }
    }


#ifdef LEAP_NOISY
    imshow("disparity", out);
    waitKey(5);
#endif

    return true;
  }



}
