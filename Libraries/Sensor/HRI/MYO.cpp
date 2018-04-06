///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Human-Robot Interaction
//  Workfile:        MYO.cpp
//  Revision:        1.0 - 13 July, 2016
//  Author:          J. Marvel
//
//  Description
//  ===========
//  Simplified interface library for Thalmic Labs Myo armband
//
///////////////////////////////////////////////////////////////////////////////

#include "MYO.h"

#include <iostream>
#include <stdexcept>
#include <fstream>
#include <cassert>
#include <ctime>
#include <time.h>


namespace Sensor
{
  //! @brief Thread to constantly poll the Myo sensors in the background
  //!
  void livemanMyo(void *param)
  {
    MyoObj *cb = (MyoObj*)param;

    try
    {
      myo::Hub hub("crpi.myo.interface");

      // Instantiate the PrintMyoEvents class we defined above, and attach it as a listener to our Hub.
      hub.addListener(cb);

      while (true)
      {
        // Process events for 10 milliseconds at a time.
        hub.run(10);
      }
    }
    catch (const exception& e)
    {
      cerr << "Error: " << e.what() << endl;
      cerr << "Press enter to continue.";
      cin.ignore();
      return;
    }

  }


  LIBRARY_API MyoObj::MyoObj()
  {
    handle_ = ulapi_mutex_new(21);

    //! Create new thread to force a constant polling of the Myo objects
    task_ = ulapi_task_new();
    ulapi_task_start((ulapi_task_struct*)task_, livemanMyo, this, ulapi_prio_lowest(), 0);
  }


  LIBRARY_API void MyoObj::onPair(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    myo->setStreamEmg(myo::Myo::streamEmgEnabled);
    Myos_.push_back(myo);

    ulapi_mutex_take(handle_);
    MyoSubject sub;
    subjects_.push_back(sub);
    ulapi_mutex_give(handle_);
  }


  LIBRARY_API void MyoObj::onPose(myo::Myo* myo, uint64_t timestamp, myo::Pose pose)
  {
    size_t index = identifyMyo(myo);
    if (index > 0)
    {
      --index;
      ulapi_mutex_take(handle_);
      subjects_.at(index).pose = pose.toString();
      ulapi_mutex_give(handle_);
    }
  }


  LIBRARY_API void MyoObj::onEmgData(myo::Myo* myo, uint64_t timestamp, const int8_t* emg)
  {
    size_t index = identifyMyo(myo);
    if (index > 0)
    {
      --index;
      ulapi_mutex_take(handle_);
      for (int i = 0; i < 8; ++i)
      {
        subjects_.at(index).emgSamples[i] = emg[i];
      }
      ulapi_mutex_give(handle_);
    }
  }


  LIBRARY_API void MyoObj::onAccelerometerData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float> &accel)
  {
    size_t index = identifyMyo(myo);
    if (index > 0)
    {
      --index;
      ulapi_mutex_take(handle_);
      subjects_.at(index).accelSamples.at(0) = accel.x();
      subjects_.at(index).accelSamples.at(1) = accel.y();
      subjects_.at(index).accelSamples.at(2) = accel.z();
      ulapi_mutex_give(handle_);
    }
  }


  LIBRARY_API void MyoObj::onGyroscopeData(myo::Myo* myo, uint64_t timestamp, const myo::Vector3<float> &gyro)
  {
    size_t index = identifyMyo(myo);
    if (index > 0)
    {
      --index;
      ulapi_mutex_take(handle_);
      subjects_.at(index).gyroSamples.at(0) = gyro.x();
      subjects_.at(index).gyroSamples.at(1) = gyro.y();
      subjects_.at(index).gyroSamples.at(2) = gyro.z();
      ulapi_mutex_give(handle_);
    }
  }


  LIBRARY_API void MyoObj::onOrientationData(myo::Myo* myo, uint64_t timestamp, const myo::Quaternion<float> &rotation)
  {
    size_t index = identifyMyo(myo);
    if (index > 0)
    {
      --index;
      //! Note:  our quaternion representation is (w, x, y, z) in vector format
      ulapi_mutex_take(handle_);
      subjects_.at(index).orientSamples.at(0) = rotation.w();
      subjects_.at(index).orientSamples.at(1) = rotation.x();
      subjects_.at(index).orientSamples.at(2) = rotation.y();
      subjects_.at(index).orientSamples.at(3) = rotation.z();
      ulapi_mutex_give(handle_);
    }
  }


  LIBRARY_API void MyoObj::onConnect(myo::Myo* myo, uint64_t timestamp, myo::FirmwareVersion firmwareVersion)
  {
    //! Not currently used
  }


  LIBRARY_API void MyoObj::onDisconnect(myo::Myo* myo, uint64_t timestamp)
  {
    //! Not currently used
  }


  LIBRARY_API size_t MyoObj::identifyMyo(myo::Myo* myo)
  {
    // Walk through the list of Myo devices that we've seen pairing events for.
    for (size_t i = 0; i < Myos_.size(); ++i)
    {
      // If two Myo pointers compare equal, they refer to the same Myo device.
      if (Myos_[i] == myo)
      {
        return i + 1;
      }
    }

    return 0;
  }


  LIBRARY_API void MyoObj::getData(vector<MyoSubject> &data)
  {
    ulapi_mutex_take(handle_);
    vector<MyoSubject>::iterator iter;
    data.clear();

    for (iter = subjects_.begin(); iter != subjects_.end(); ++iter)
    {
      data.push_back(*iter);
    }
    ulapi_mutex_give(handle_);
  }


  LIBRARY_API vector<MyoSubject>& MyoObj::getData()
  {
    vector<MyoSubject> temp;
    ulapi_mutex_take(handle_);
    int v = subjects_.size();

    for (int i = 0; i < v; ++i)
    {
      temp.push_back(subjects_.at(i));
    }
    ulapi_mutex_give(handle_);
    return temp;
  }


}