///////////////////////////////////////////////////////////////////////////////
//
//  Original System: CRPI
//  Subsystem:       Human-Robot Interaction
//  Workfile:        ManusGlove.h
//  Revision:        1.0 - 7 August, 2017
//  Author:          M. Zimmerman
//
//  Description
//  ===========
//  Interface wrapper for Manus VR Gloves
//
///////////////////////////////////////////////////////////////////////////////


#ifndef MANUSGLOVE_H
#define MANUSGLOVE_H
#include <string.h>
#include <iostream>
#if defined(_MSC_VER)
#include "..\..\Libraries\ThirdParty\Manus\Manus.h"
#else
#include "../../ThirdParty/Manus/Manus.h"
#endif


using namespace std;

namespace Sensor
{
	class ManusGloves
	{
	public:

		//! @brief Default constructor
		//!
		ManusGloves();

		//! @breif Alternate constructor
		//!
		//! @params handn	which hand to set to default - 0 for left, 1 for right;
		ManusGloves(int handn);

		//! @brief Default destructor
		//!
		~ManusGloves();

		//! @brief Method of determining connectivity of both hands
		//!
		//! @return True if both hands are connected, false if not
		//!
		bool allConnected();

		//! @brief Method of determining connectivity of current hand
		//!	
		//! @return True if hand is connected, false if not
		//!
		bool isConnected();

		//! @brief Method of determining connectivity of a given hand
		//!
		//! @param handn		which hand to obtain, 0 for left, 1 for right - any other values (say, 2), Hand will be whichever it was most recently set to
		//!
		//! @return True if hand is connected, false if not 
		//!
		bool isConnected(int handn);

		//! @brief Formated hand output getter
		//!
		//! @param formhand		pointer to hand object, returns hand to both format and the pointer object refered to 
		//! @param handn		which hand to obtain, 0 for left, 1 for right - any other values (say, 2), Hand will be whichever it was most recently set to
		//!
		//! @return True if the function executes properly, False otherwise (ie. timeout)
		//!
		bool GetHand(manus_hand_t* formhand, int handn);

		//! @brief Raw hand output getter
		//!
		//! @param rawpt		pointer to raw hand object, returns hand to both format and the pointer object refered to 
		//! @param handn		which hand to obtain, 0 for left, 1 for right - any other values (say, 2), Hand will be whichever it was most recently set to
		//!
		//! @return True if the function executes properly, False otherwise (ie. timeout)
		//!
		bool GetRawHand(manus_hand_raw_t* rawpt, int handn);

		//! @brief Sends vibration to hand
		//!
		//! @param length		Duration in ms of vibration
		//! @param power		Power of vibration motor ranging from 0 to 1 (ie. 0.5 = 50% power)
		//!
		//! @return True if the function executes properly, False otherwise (ie. timeout)
		//!
		bool SendVibration(unsigned short length, float power, int handn);

		//! @breif Prints raw data from given hand to console
		//!
		//! @param handn		which hand to use, 0 for left, 1 for right - any other values (say, 2), Hand will be whichever it was most recently set to
		//!
		//! @return True if the function executes properly, False otherwise (ie. timeout)
		//!
		bool PrintRaw(int handn);

		//! @breif Prints formated quaternion data from given hand to console
		//!
		//! @param handn		which hand to use, 0 for left, 1 for right - any other values (say, 2), Hand will be whichever it was most recently set to
		//!
		//! @return True if the function executes properly, False otherwise (ie. timeout)
		//!
		bool PrintQuat(int handn);

		//! @breif Prints formated cartesian data from given hand to console
		//!
		//! @param handn		which hand to use, 0 for left, 1 for right - any other values (say, 2), Hand will be whichever it was most recently set to
		//!
		//! @return True if the function executes properly, False otherwise (ie. timeout)
		//!
		bool PrintFormated(int handn);


		//! @breif Overloaded << opperator
		//!
		//! @return filled ostream with comma seperated raw data values
		//!
		friend std::ostream& operator<<(std::ostream& out, const ManusGloves &inGlove);

	private:
		//! @breif Session accessors 
		//!
		manus_session_t sess;
		manus_session_t* sessp;

		//! @breif Current hand we are working with, 0 is left, 1 is right - initially set to right
		//!
		device_type_t hand;

		//! @breif raw data accessors 
		//!
		manus_hand_raw_t raw;
		manus_hand_raw_t* rawp;

		//! @breif formated data accessors
		//!
		manus_hand_t format;
		manus_hand_t* formatp;

		bool formset;
		bool rawset;


	};


}


#endif
