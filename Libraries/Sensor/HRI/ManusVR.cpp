#include "ManusVR.h"
using namespace std;

namespace Sensor
{

	/* Methods for ManusGloves ---------------------------------------------------------------------*/
	ManusGloves::ManusGloves()
	{
		//!pointer initialization
		sessp = &sess;
		rawp = &raw;
		formatp = &format;

		//setting hand to default
		hand = GLOVE_RIGHT;

		//setting indicator bits
		formset = false;
		rawset = false;

		//initializing manus session
		ManusInit(sessp);
	}

	ManusGloves::~ManusGloves()
	{
		//exiting manus session
		ManusExit(*sessp);

		//clearing pointers for sanity check
		sessp = NULL;
		rawp = NULL;
		formatp = NULL;
	}

	bool ManusGloves::allConnected()
	{
		bool isr, isl;
		isr = ManusIsConnected(*sessp, GLOVE_RIGHT);
		isl = ManusIsConnected(*sessp, GLOVE_LEFT);

		if (isr && isl)
		{
			return true;
		}
		else
		{
			return false;
		}
	}


	bool ManusGloves::isConnected()
	{

		return ManusIsConnected(*sessp, hand);

	}

	bool ManusGloves::isConnected(int handn)
	{
		bool isc;

		if (handn == 0)
		{
			hand = GLOVE_LEFT;
		}
		else if (handn == 1)
		{
			hand = GLOVE_RIGHT;
		}

		isc = ManusIsConnected(*sessp, hand);


		return isc;

	}


	bool ManusGloves::GetHand(manus_hand_t* formhand, int handn)
	{
		//timeout counter
		int count = 0;
		bool iscon = false;

		if (handn == 0)
		{
			hand = GLOVE_LEFT;
		}
		else if (handn == 1)
		{
			hand = GLOVE_RIGHT;
		}

		//making sure hand is connected
		while (!iscon)
		{
			iscon = ManusIsConnected(*sessp, hand);

			//timeout functionality
			count++;
			if (count > 100)
			{
				return false;
			}
		}

		//get data
		ManusGetHand(*sessp, hand, formatp);

		//set pointer and indicator bit
		formhand = formatp;
		formset = true;

		return true;


	}
	bool ManusGloves::GetRawHand(manus_hand_raw_t* rawpt, int handn)
	{
		//timeout counter
		int count = 0;
		bool iscon = false;

		if (handn == 0)
		{
			hand = GLOVE_LEFT;
		}
		else if (handn == 1)
		{
			hand = GLOVE_RIGHT;
		}

		//making sure hand is connected
		while (!iscon)
		{
			iscon = ManusIsConnected(*sessp, hand);

			//timeout functionality
			count++;
			if (count > 100)
			{
				return false;
			}
		}

		//get data
		ManusGetHandRaw(*sessp, hand, rawp);

		//set pointer and indicator bit
		rawpt = rawp;
		rawset = true;

		return true;


	}

	bool ManusGloves::SendVibration(unsigned short length, float power, int handn)
	{
		//timeout counter
		int count = 0;
		bool iscon = false;

		if (handn == 0)
		{
			hand = GLOVE_LEFT;
		}
		else if (handn == 1)
		{
			hand = GLOVE_RIGHT;
		}

		//making sure hand is connected
		while (!iscon)
		{
			iscon = ManusIsConnected(*sessp, hand);

			//timeout functionality
			count++;
			if (count > 100)
			{
				return false;
			}
		}

		ManusSetVibration(*sessp, hand, power, length);

		return true;
	}

	bool ManusGloves::PrintFormated(int handn)
	{
		if (!formset)
		{
			return false;
		}
		manus_hand_t current;
		if (!GetHand(&current, handn))
		{
			return false;
		}

		std::cout << "Thumb - x,y,z ------" << endl;
		std::cout << "carpal: " << current.thumb.carpal.translation.x << ", " << current.thumb.carpal.translation.y << ", " << current.thumb.carpal.translation.z << endl;
		std::cout << "distal: " << current.thumb.distal.translation.x << ", " << current.thumb.distal.translation.y << ", " << current.thumb.distal.translation.z << endl;
		std::cout << "middle: " << current.thumb.intermediate.translation.x << ", " << current.thumb.intermediate.translation.y << ", " << current.thumb.intermediate.translation.z << endl;
		std::cout << "proxim: " << current.thumb.proximal.translation.x << ", " << current.thumb.proximal.translation.y << ", " << current.thumb.proximal.translation.z << endl;
		std::cout << "mtacpl: " << current.thumb.metacarpal.translation.x << ", " << current.thumb.metacarpal.translation.y << ", " << current.thumb.metacarpal.translation.z << endl;

		std::cout << "Index finger - x,y,z ------" << endl;
		std::cout << "carpal: " << current.index.carpal.translation.x << ", " << current.index.carpal.translation.y << ", " << current.index.carpal.translation.z << endl;
		std::cout << "distal: " << current.index.distal.translation.x << ", " << current.index.distal.translation.y << ", " << current.index.distal.translation.z << endl;
		std::cout << "middle: " << current.index.intermediate.translation.x << ", " << current.index.intermediate.translation.y << ", " << current.index.intermediate.translation.z << endl;
		std::cout << "proxim: " << current.index.proximal.translation.x << ", " << current.index.proximal.translation.y << ", " << current.index.proximal.translation.z << endl;
		std::cout << "mtacpl: " << current.index.metacarpal.translation.x << ", " << current.index.metacarpal.translation.y << ", " << current.index.metacarpal.translation.z << endl;

		std::cout << "Middle Finger - x,y,z ------" << endl;
		std::cout << "carpal: " << current.middle.carpal.translation.x << ", " << current.middle.carpal.translation.y << ", " << current.middle.carpal.translation.z << endl;
		std::cout << "distal: " << current.middle.distal.translation.x << ", " << current.middle.distal.translation.y << ", " << current.middle.distal.translation.z << endl;
		std::cout << "middle: " << current.middle.intermediate.translation.x << ", " << current.middle.intermediate.translation.y << ", " << current.middle.intermediate.translation.z << endl;
		std::cout << "proxim: " << current.middle.proximal.translation.x << ", " << current.middle.proximal.translation.y << ", " << current.middle.proximal.translation.z << endl;
		std::cout << "mtacpl: " << current.middle.metacarpal.translation.x << ", " << current.middle.metacarpal.translation.y << ", " << current.middle.metacarpal.translation.z << endl;

		std::cout << "Ring Finger - x,y,z ------" << endl;
		std::cout << "carpal: " << current.ring.carpal.translation.x << ", " << current.ring.carpal.translation.y << ", " << current.ring.carpal.translation.z << endl;
		std::cout << "distal: " << current.ring.distal.translation.x << ", " << current.ring.distal.translation.y << ", " << current.ring.distal.translation.z << endl;
		std::cout << "ring: " << current.ring.intermediate.translation.x << ", " << current.ring.intermediate.translation.y << ", " << current.ring.intermediate.translation.z << endl;
		std::cout << "proxim: " << current.ring.proximal.translation.x << ", " << current.ring.proximal.translation.y << ", " << current.ring.proximal.translation.z << endl;
		std::cout << "mtacpl: " << current.ring.metacarpal.translation.x << ", " << current.ring.metacarpal.translation.y << ", " << current.ring.metacarpal.translation.z << endl;

		std::cout << "Pinky Finger - x,y,z ------" << endl;
		std::cout << "carpal: " << current.pinky.carpal.translation.x << ", " << current.pinky.carpal.translation.y << ", " << current.pinky.carpal.translation.z << endl;
		std::cout << "distal: " << current.pinky.distal.translation.x << ", " << current.pinky.distal.translation.y << ", " << current.pinky.distal.translation.z << endl;
		std::cout << "pinky: " << current.pinky.intermediate.translation.x << ", " << current.pinky.intermediate.translation.y << ", " << current.pinky.intermediate.translation.z << endl;
		std::cout << "proxim: " << current.pinky.proximal.translation.x << ", " << current.pinky.proximal.translation.y << ", " << current.pinky.proximal.translation.z << endl;
		std::cout << "mtacpl: " << current.pinky.metacarpal.translation.x << ", " << current.pinky.metacarpal.translation.y << ", " << current.pinky.metacarpal.translation.z << endl;

		return true;
	}

	bool ManusGloves::PrintQuat(int handn)
	{
		if (!formset)
		{
			return false;
		}
		manus_hand_t current;
		if (!GetHand(&current, handn))
		{
			return false;
		}

		std::cout << "Thumb - x,y,z,w ------" << endl;
		std::cout << "carpal: " << current.thumb.carpal.rotation.x << ", " << current.thumb.carpal.rotation.y << ", " << current.thumb.carpal.rotation.z << ", " << current.thumb.carpal.rotation.w << endl;
		std::cout << "distal: " << current.thumb.distal.rotation.x << ", " << current.thumb.distal.rotation.y << ", " << current.thumb.distal.rotation.z << ", " << current.thumb.distal.rotation.w << endl;
		std::cout << "middle: " << current.thumb.intermediate.rotation.x << ", " << current.thumb.intermediate.rotation.y << ", " << current.thumb.intermediate.rotation.z << ", " << current.thumb.intermediate.rotation.w << endl;
		std::cout << "proxim: " << current.thumb.proximal.rotation.x << ", " << current.thumb.proximal.rotation.y << ", " << current.thumb.proximal.rotation.z << ", " << current.thumb.proximal.rotation.w << endl;
		std::cout << "mtacpl: " << current.thumb.metacarpal.rotation.x << ", " << current.thumb.metacarpal.rotation.y << ", " << current.thumb.metacarpal.rotation.z << ", " << current.thumb.metacarpal.rotation.w << endl;

		std::cout << "Index finger - x,y,z,w ------" << endl;
		std::cout << "carpal: " << current.index.carpal.rotation.x << ", " << current.index.carpal.rotation.y << ", " << current.index.carpal.rotation.z << ", " << current.index.carpal.rotation.w << endl;
		std::cout << "distal: " << current.index.distal.rotation.x << ", " << current.index.distal.rotation.y << ", " << current.index.distal.rotation.z << ", " << current.index.distal.rotation.w << endl;
		std::cout << "middle: " << current.index.intermediate.rotation.x << ", " << current.index.intermediate.rotation.y << ", " << current.index.intermediate.rotation.z << ", " << current.index.intermediate.rotation.w << endl;
		std::cout << "proxim: " << current.index.proximal.rotation.x << ", " << current.index.proximal.rotation.y << ", " << current.index.proximal.rotation.z << ", " << current.index.proximal.rotation.w << endl;
		std::cout << "mtacpl: " << current.index.metacarpal.rotation.x << ", " << current.index.metacarpal.rotation.y << ", " << current.index.metacarpal.rotation.z << ", " << current.index.metacarpal.rotation.w << endl;

		std::cout << "Middle Finger - x,y,z,w ------" << endl;
		std::cout << "carpal: " << current.middle.carpal.rotation.x << ", " << current.middle.carpal.rotation.y << ", " << current.middle.carpal.rotation.z << ", " << current.middle.carpal.rotation.w << endl;
		std::cout << "distal: " << current.middle.distal.rotation.x << ", " << current.middle.distal.rotation.y << ", " << current.middle.distal.rotation.z << ", " << current.middle.distal.rotation.w << endl;
		std::cout << "middle: " << current.middle.intermediate.rotation.x << ", " << current.middle.intermediate.rotation.y << ", " << current.middle.intermediate.rotation.z << ", " << current.middle.intermediate.rotation.w << endl;
		std::cout << "proxim: " << current.middle.proximal.rotation.x << ", " << current.middle.proximal.rotation.y << ", " << current.middle.proximal.rotation.z << ", " << current.middle.proximal.rotation.w << endl;
		std::cout << "mtacpl: " << current.middle.metacarpal.rotation.x << ", " << current.middle.metacarpal.rotation.y << ", " << current.middle.metacarpal.rotation.z << ", " << current.middle.metacarpal.rotation.w << endl;

		std::cout << "Ring Finger - x,y,z,w ------" << endl;
		std::cout << "carpal: " << current.ring.carpal.rotation.x << ", " << current.ring.carpal.rotation.y << ", " << current.ring.carpal.rotation.z << ", " << current.ring.carpal.rotation.w << endl;
		std::cout << "distal: " << current.ring.distal.rotation.x << ", " << current.ring.distal.rotation.y << ", " << current.ring.distal.rotation.z << ", " << current.ring.distal.rotation.w << endl;
		std::cout << "ring: " << current.ring.intermediate.rotation.x << ", " << current.ring.intermediate.rotation.y << ", " << current.ring.intermediate.rotation.z << ", " << current.ring.intermediate.rotation.w << endl;
		std::cout << "proxim: " << current.ring.proximal.rotation.x << ", " << current.ring.proximal.rotation.y << ", " << current.ring.proximal.rotation.z << ", " << current.ring.proximal.rotation.w << endl;
		std::cout << "mtacpl: " << current.ring.metacarpal.rotation.x << ", " << current.ring.metacarpal.rotation.y << ", " << current.ring.metacarpal.rotation.z << ", " << current.ring.metacarpal.rotation.w << endl;

		std::cout << "Pinky Finger - x,y,z,w ------" << endl;
		std::cout << "carpal: " << current.pinky.carpal.rotation.x << ", " << current.pinky.carpal.rotation.y << ", " << current.pinky.carpal.rotation.z << ", " << current.pinky.carpal.rotation.w << endl;
		std::cout << "distal: " << current.pinky.distal.rotation.x << ", " << current.pinky.distal.rotation.y << ", " << current.pinky.distal.rotation.z << ", " << current.pinky.distal.rotation.w << endl;
		std::cout << "pinky: " << current.pinky.intermediate.rotation.x << ", " << current.pinky.intermediate.rotation.y << ", " << current.pinky.intermediate.rotation.z << ", " << current.pinky.intermediate.rotation.w << endl;
		std::cout << "proxim: " << current.pinky.proximal.rotation.x << ", " << current.pinky.proximal.rotation.y << ", " << current.pinky.proximal.rotation.z << ", " << current.pinky.proximal.rotation.w << endl;
		std::cout << "mtacpl: " << current.pinky.metacarpal.rotation.x << ", " << current.pinky.metacarpal.rotation.y << ", " << current.pinky.metacarpal.rotation.z << ", " << current.pinky.metacarpal.rotation.w << endl;

		return true;
	}

	bool ManusGloves::PrintRaw(int handn)
	{
		if (!formset)
		{
			return false;
		}
		manus_hand_raw_t current;

		if (!GetRawHand(&current, handn))
		{
			return false;
		}

		std::cout << "Pinky - 0,1 ---------: " << endl;
		std::cout << current.finger_sensor[0] << "," << current.finger_sensor[1] << endl;
		std::cout << "Ring - 2,3  ---------: " << endl;
		std::cout << current.finger_sensor[2] << ", " << current.finger_sensor[3] << endl;
		std::cout << "Middle - 4,5 --------: " << endl;
		std::cout << current.finger_sensor[4] << ", " << current.finger_sensor[5] << endl;
		std::cout << "Index - 6,7 ---------: " << endl;
		std::cout << current.finger_sensor[6] << ", " << current.finger_sensor[7] << endl;
		std::cout << "Thumb - 8,9 ---------: " << endl;
		std::cout << current.finger_sensor[8] << ", " << current.finger_sensor[9] << endl;

		return true;
	}

	std::ostream& operator<<(std::ostream& out, const ManusGloves &inGlove)
	{

		out << inGlove.rawp->finger_sensor[0] << ", " << inGlove.rawp->finger_sensor[1] << ", " << inGlove.rawp->finger_sensor[2] << ", " << inGlove.rawp->finger_sensor[3] << ", " << inGlove.rawp->finger_sensor[4] << ", " << inGlove.rawp->finger_sensor[5] << ", " << inGlove.rawp->finger_sensor[6] << ", " << inGlove.rawp->finger_sensor[7] << ", " << inGlove.rawp->finger_sensor[8] << ", " << inGlove.rawp->finger_sensor[9] << endl;
		return out;
	}


}