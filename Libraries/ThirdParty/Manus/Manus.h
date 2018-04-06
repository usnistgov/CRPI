#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef _WIN32
#ifdef MANUS_EXPORTS
#define MANUS_API __declspec(dllexport)
#else
#define MANUS_API __declspec(dllimport)
#endif
#elif defined(__linux__)
#define MANUS_API
#endif

typedef enum {
	MANUS_SUCCESS = 0,
	MANUS_ERROR = 1,
	MANUS_INVALID_ARGUMENT = 2,
	MANUS_DISCONNECTED = 3,
	MANUS_FILESYSTEM_ERROR = 4,
	MANUS_INVALID_SESSION = 5,
	MANUS_NOT_IMPLEMENTED = 100
} manus_error_t;

// Note, we're returning uint32_t in stead of manus_error_t in the external C API
// to ensure a known data size. When using enums it's up to the compiler.
typedef uint32_t manus_ret_t;

typedef enum {
	COOR_X_UP = 0,
	COOR_Y_UP,
	COOR_Z_UP
} coor_up_t;

typedef enum {
	COOR_LEFT_HANDED = 0,
	COOR_RIGHT_HANDED
} coor_handed_t;

typedef enum {
	GLOVE_LEFT = 0,
	GLOVE_RIGHT
} device_type_t;

typedef struct  {
	double x, y, z, w;
} quat_t;

typedef struct {
	double x, y, z;
} vector_t;

/*! Pose structure representing an orientation and position. */
typedef struct {
	vector_t translation;
	quat_t rotation;
} pose_t;

typedef struct {
	union {
		struct {
			pose_t carpal;
			pose_t metacarpal;
			pose_t proximal;
			pose_t intermediate;
			pose_t distal;
		};
		pose_t joints[5];
	};
} finger_t;

// imu[0] = hand
// imu[1] = thumb
// finger_sensor[0,2,4,6,8] = lower sensor (closest to arm)
// finger_sensor[1,3,5,7,9] = upper sensor (furthest from arm)
// finger[0,1] = little finger
// ...
// finger[8,9] = thumb
typedef struct {
	quat_t imu[2];
	double finger_sensor[10];
} manus_hand_raw_t;

typedef struct {
	manus_hand_raw_t raw;
	quat_t wrist;
	union {
		struct {
			finger_t thumb;
			finger_t index;
			finger_t middle;
			finger_t ring;
			finger_t pinky;
		};
		finger_t fingers[5];
	};
} manus_hand_t;

typedef struct {
	union {
		struct {
			double metacarpal;
			double proximal;
			double intermediate;
			double distal;
		};
		double bones[4];
	};
} finger_profile_t;

typedef struct {
	double wrist;
	union {
		struct {
			finger_profile_t thumb;
			finger_profile_t index;
			finger_profile_t middle;
			finger_profile_t ring;
			finger_profile_t pinky;
		};
		finger_profile_t fingers[5];
	};
} hand_profile_t;

/*! Skeletal model of the arm which contains a quaternion for each joint and a position for the hand. */
typedef struct {
	pose_t shoulder; // OUTPUT
	quat_t upperArm; // OUTPUT
	pose_t lowerArm; // INPUT, set this at the controller position
} ik_arm_t;

/*! Skeletal model for the upper body of the player. */
typedef struct {
	pose_t head; // INPUT, set this at the headset position
	ik_arm_t left, right;
} ik_body_t;

typedef struct {
	double shoulderLength;
	double upperArmLength;
	double lowerArmLength;
	double upperNeckLength;
	double lowerNeckLength;
	vector_t upperNeckOffset;
	hand_profile_t handProfile;
} ik_profile_t;

typedef struct manus_struct_t* manus_session_t;

#ifdef __cplusplus
extern "C" {
#endif
	/*! Initialize the Manus SDK.
	*
	*   Must be called before any other function in the SDK.
	*   This function should only be called once.
	*/
	MANUS_API manus_ret_t ManusInit(manus_session_t* session);

	/*! Shutdown the Manus SDK.
	*
	*   Must be called when the SDK is no longer
	*   needed.
	*/
	MANUS_API manus_ret_t ManusExit(manus_session_t session);

	/*! Get the dongle id of every dongle in use. Returns the number of dongles in use if count is
	* not the same as the number of dongles in use.
	*/
	MANUS_API uint32_t ManusGetDongleIDs(manus_session_t session,  uint32_t* out_ids, uint32_t count);

	/*! Set the ouput power of the vibration motor.
	*
	*   This function should only be called once every 2ms. Higher update rates will be
	*   supported in later versions of the SDK.
	*
	*  \param device_type The device type to vibrate.
	*  \param power The power of the vibration motor ranging from 0 to 1 (ex. 0.5 = 50% power).
	*  \param milliseconds The duration of the vibration.
	*/
	MANUS_API manus_ret_t ManusSetVibration(manus_session_t session, device_type_t device_type, float power, uint16_t milliseconds);
	/*! Set the ouput power of the vibration motor.
	*
	*   This function should only be called once every 2ms. Higher update rates will be
	*   supported in later versions of the SDK.
	*
	*  \param dongle_id The ID of the dongle that this call is meant for.
	*  \param device_type The device type to vibrate.
	*  \param power The power of the vibration motor ranging from 0 to 1 (ex. 0.5 = 50% power).
	*  \param milliseconds The duration of the vibration.
	*/
	MANUS_API manus_ret_t ManusSetVibration_id(manus_session_t session, uint32_t dongle_id, device_type_t device_type, float power, uint16_t milliseconds);

	/*! Switch to a different saved user profile.
	*
	*  \param index The profile index.
	*
	*  \see ManusGetProfileCount
	*/
	MANUS_API manus_ret_t ManusSelectProfile(manus_session_t session, uint32_t index);

	/*! Get the IK user profile identifier.
	*
	*  \param index The profile index.
	*
	*  \see ManusGetProfileCount
	*/
	MANUS_API const char* ManusGetProfileName(manus_session_t session, uint32_t index);

	/*! Creates a new user profile and returns its index.
	*
	*   The profile can be selected by passing the index to ManusSelectProfile().
	*
	*  \param name User profile identifier.
	*  \returns The index of the newly added profile.
	*
	*  \see ManusSelectProfile
	*/
	MANUS_API uint32_t ManusCreateProfile(manus_session_t session, const char* name);

	/*! Get the total number of user profiles.
	*/
	MANUS_API uint32_t ManusGetProfileCount(manus_session_t session);

	/*! Get the data from the currently selected user profile.
	*
	*   This profile is used by the IK system for inverse kinematics and for the skeletal hand model.
	*   It should also be used by the game to scale the in-game model to fit the user profile.
	*
	*  \see ManusUpdateIK
	*/
	MANUS_API manus_ret_t ManusGetProfile(manus_session_t session, ik_profile_t* out_profile);

	/*! Change currently selected user profile data.
	*
	*   This changes the working copy of the currently selected profile. If you want to save
	*   the current working copy to disk you should call the commit function.
	*
	*  \see ManusCommitProfile
	*
	*  \params profile The new profile data to set in the working copy.
	*/
	MANUS_API manus_ret_t ManusSetProfile(manus_session_t session, ik_profile_t profile);

	/*! Revert the working copies to the profiles saved on the file system.
	*
	*   This reverts all changes made to the working copy that hasn't been committed.
	*
	*  \params ManusCommitProfile
	*/
	MANUS_API manus_ret_t ManusRevertProfiles(manus_session_t session);

	/*! Write the working copies of all user profiles to the file system.
	*
	*   After this call the old profile data can no longer be retrieved.
	*
	*  \params ManusRevertProfile
	*/
	MANUS_API manus_ret_t ManusCommitProfiles(manus_session_t session);

	/*! Calculate the shoulder and elbow position.
	*
	*   This calculates shoulder and elbow position based on the wrist and head position.
	*
	*  \param model The model of the body with both the inputs and the outputs of this calculation.
	*  \param settings The settings of the person used for the calculations, such as arm lengths.
	*/
	MANUS_API manus_ret_t ManusUpdateIK(manus_session_t session, ik_body_t* inout_model);

	/*! Check if the device is connected.
	*/
	MANUS_API bool ManusIsConnected(manus_session_t session, device_type_t device);
	/*! Check if the device is connected.
	*/
	MANUS_API bool ManusIsConnected_id(manus_session_t session, uint32_t dongle_id, device_type_t device);

	/*! Sets the coordinate system for all orientations and vectors returned from the SDK.
	*/
	MANUS_API manus_ret_t ManusSetCoordinateSystem(manus_session_t session, coor_up_t up, coor_handed_t handed);

	/*! Get the current battery level in percent.
	*/
	MANUS_API manus_ret_t ManusGetBatteryLevel(manus_session_t session, device_type_t device, uint8_t* out_percentage);
	/*! Get the current battery level in percent.
	*/
	MANUS_API manus_ret_t ManusGetBatteryLevel_id(manus_session_t session, uint32_t dongle_id, device_type_t device, uint8_t* out_percentage);

	/*! Get the current battery level in millivolt.
	*/
	MANUS_API manus_ret_t ManusGetBatteryVoltage(manus_session_t session, device_type_t device_type, uint16_t* out_millivolt);
	/*! Get the current battery level in millivolt.
	*/
	MANUS_API manus_ret_t ManusGetBatteryVoltage_id(manus_session_t session, uint32_t dongle_id, device_type_t device_type, uint16_t* out_millivolt);

	/*! Get the signal strength.
	*/
	MANUS_API manus_ret_t ManusGetSignalStrength(manus_session_t session, device_type_t device, int16_t* out_rssi);
	/*! Get the signal strength.
	*/
	MANUS_API manus_ret_t ManusGetSignalStrength_id(manus_session_t session, uint32_t dongle_id, device_type_t device, int16_t* out_rssi);

	/*! Get the data from the glove and convert it to a skeletal model. The skeletal model is scaled
	*   according to the user profile.
	*
	*  \see ManusGetProfile
	*
	*  \params device The device type to query.
	*  \params out_hand The output struct, also contains the raw data from the device.
	*  \params timeout The time in milliseconds to block waiting on a new packet.
	*/
	MANUS_API manus_ret_t ManusGetHand(manus_session_t session, device_type_t device, manus_hand_t* out_hand);

	/*! Sets the dongle channel
	*
	*  \params channel The channel to set the dongle to.
	*/
	MANUS_API manus_ret_t ManusSetChannel(manus_session_t session, uint32_t channel);
	/*! Sets the dongle channel for a specific dongle ID
	*
	*  \see ManusSetChannel
	*
	*  \params dongle_id The dongle ID of the dongle that the channel should be set for.
	*  \params channel The channel to set the dongle to.
	*/
	MANUS_API manus_ret_t ManusSetChannel_id(manus_session_t session, uint32_t dongle_id, uint32_t channel);

	/*! Get the raw data from the glove. This does not include any of the skeletal model data. If you're also
	*   interested in the skeletal model you should call ManusGetHand() instead.
	*
	*  \see ManusGetHand
	*
	*  \params device The device type to query.
	*  \params out_hand The output struct.
	*  \params timeout The time in milliseconds to block waiting on a new packet.
	*/
	MANUS_API manus_ret_t ManusGetHandRaw(manus_session_t session, device_type_t device, manus_hand_raw_t* out_hand);
	/*! Get the raw data from the glove. This does not include any of the skeletal model data. If you're also
	*   interested in the skeletal model you should call ManusGetHand() instead.
	*
	*  \see ManusGetHand
	*
	*  \params dongle_id The dongle ID of the dongle that you want to get the data from.
	*  \params device The device type to query.
	*  \params out_hand The output struct.
	*  \params timeout The time in milliseconds to block waiting on a new packet.
	*/
	MANUS_API manus_ret_t ManusGetHandRaw_id(manus_session_t session, uint32_t dongle_id, device_type_t device, manus_hand_raw_t* out_hand);

	/*! Gets a string that describes the error code. This can be used to display error messages
	*   in the user interface.
	*/
	MANUS_API const char* ManusErrorString(uint32_t error);

	/*! Adds a virtual dongle that emulates two gloves connected to the API.
	*
	*  \see ManusRemoveDebugDevice
	*/
	MANUS_API manus_ret_t ManusAddDebugDevice(manus_session_t session);

	/*! Removes the virtual dongle.
	*
	*  \see ManusAddDebugDevice
	*/
	MANUS_API manus_ret_t ManusRemoveDebugDevice(manus_session_t session);
#ifdef __cplusplus
}
#endif
