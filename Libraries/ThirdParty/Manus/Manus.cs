/*
   Copyright 2016 Manus VR

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

using System;
using System.Diagnostics;
using System.Runtime.InteropServices;

namespace ManusVR
{
    using manus_session_t = IntPtr;

    public enum manus_ret_t
    {
        MANUS_SUCCESS = 0,
        MANUS_ERROR = 1,
        MANUS_INVALID_ARGUMENT = 2,
        MANUS_DISCONNECTED = 3,
        MANUS_FILESYSTEM_ERROR = 4,
        MANUS_INVALID_SESSION = 5,
        MANUS_NOT_IMPLEMENTED = 100
    }

    public enum coor_up_t
    {
        COOR_X_UP = 0,
        COOR_Y_UP,
        COOR_Z_UP
    }

    public enum coor_handed_t
    {
        COOR_LEFT_HANDED = 0,
        COOR_RIGHT_HANDED
    }

    public enum device_type_t
    {
        GLOVE_LEFT = 0,
        GLOVE_RIGHT
    }

    public struct quat_t
    {
        public double x, y, z, w;

#if UNITY_EDITOR || UNITY_STANDALONE
        public static implicit operator UnityEngine.Quaternion(quat_t b)
        {
            return new UnityEngine.Quaternion((float)b.x, (float)b.y, (float)b.z, (float)b.w);
        }

        public static implicit operator quat_t(UnityEngine.Quaternion b)
        {
            quat_t a = new quat_t();
            a.x = b.x;
            a.y = b.y;
            a.z = b.z;
            a.w = b.w;
            return a;
        }
#endif
    }

    public struct vector_t
    {
        public double x, y, z;

#if UNITY_EDITOR || UNITY_STANDALONE
        public static implicit operator UnityEngine.Vector3(vector_t b)
        {
            return new UnityEngine.Vector3((float)b.x, (float)b.y, (float)b.z);
        }

        public static implicit operator vector_t(UnityEngine.Vector3 b)
        {
            vector_t a = new vector_t();
            a.x = b.x;
            a.y = b.y;
            a.z = b.z;
            return a;
        }
#endif
    }

    /*! Pose structure representing an orientation and position. */
    public struct pose_t
    {
        public vector_t translation;
        public quat_t rotation;

#if UNITY_EDITOR || UNITY_STANDALONE
        public static explicit operator pose_t(UnityEngine.Transform b)
        {
            pose_t a = new pose_t();
            a.rotation = b.rotation;
            a.translation = b.position;
            Debug.Assert(a.rotation.w != 0.0);
            return a;
        }
#endif
    }

    public struct finger_t
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)]
        public pose_t[] joints;
    }

    public struct manus_hand_raw_t
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 2)]
        public quat_t[] imu;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 10)]
        public double[] finger_sensor;
    }

    public struct manus_hand_t
    {
        public manus_hand_raw_t raw;
        public quat_t wrist;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)]
        public finger_t[] fingers;
    }

    public struct finger_profile_t
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] bones;
    }

    public struct hand_profile_t
    {
        public double wrist;
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 5)]
        public finger_profile_t[] fingers;
    }

    /*! Skeletal model of the arm which contains a quaternion for each joint and a position for the hand. */
    public struct ik_arm_t
    {
        public pose_t shoulder; // OUTPUT
        public quat_t upperArm; // OUTPUT
        public pose_t lowerArm; // INPUT, set this at the controller position
    }

    /*! Skeletal model for the upper body of the player. */
    public struct ik_body_t
    {
        public pose_t head; // INPUT, set this at the headset position
        public ik_arm_t left, right;
    }

    public struct ik_profile_t
    {
        public double shoulderLength;
        public double upperArmLength;
        public double lowerArmLength;
        public double upperNeckLength;
        public double lowerNeckLength;
        public vector_t upperNeckOffset;
        public hand_profile_t handProfile;
    }

    /*!
    *   \brief Glove class
    *
    */
    public class Manus
    {
        /*! Initialize the Manus SDK.
        *
        *   Must be called before any other function in the SDK.
        *   This function should only be called once.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusInit(out manus_session_t session);

        /*! Shutdown the Manus SDK.
        *
        *   Must be called when the SDK is no longer
        *   needed.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusExit(manus_session_t session);

        /*! Set the ouput power of the vibration motor.
        *
        *   This function should only be called once every 2ms. Higher update rates will be
        *   supported in later versions of the SDK.
        *
        *  \param device The device type to vibrate.
        *  \param power The power of the vibration motor ranging from 0 to 1 (ex. 0.5 = 50% power).
        *  \param milliseconds The duration of the vibration.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusSetVibration(manus_session_t session, device_type_t device, float power, UInt16 milliseconds);

        /*! Switch to a different saved user profile.
        *
        *  \param index The profile index.
        *
        *  \see ManusGetProfileCount
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusSelectProfile(manus_session_t session, UInt32 index);

        /*! Get the IK user profile identifier.
        *
        *  \param index The profile index.
        *
        *  \see ManusGetProfileCount
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern string ManusGetProfileName(manus_session_t session, UInt32 index);

        /*! Get the total number of user profiles.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern UInt32 ManusGetProfileCount(manus_session_t session);

        /*! Get the data from the currently selected user profile.
        *
        *   This profile is used by the IK system for inverse kinematics and for the skeletal hand model.
        *   It should also be used by the game to scale the in-game model to fit the user profile.
        *
        *  \see ManusUpdateIK
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusGetProfile(manus_session_t session, out ik_profile_t out_profile);

        /*! Change currently selected user profile data.
        *
        *   This changes the working copy of the currently selected profile. If you want to save
        *   the current working copy to disk you should call the commit function.
        *
        *  \see ManusCommitProfile
        *
        *  \params profile The new profile data to set in the working copy.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusSetProfile(manus_session_t session, ik_profile_t profile);

        /*! Revert the working copies to the profiles saved on the file system.
        *
        *   This reverts all changes made to the working copy that hasn't been committed.
        *
        *  \params ManusCommitProfile
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusRevertProfiles(manus_session_t session);

        /*! Write the working copies of all user profiles to the file system.
        *
        *   After this call the old profile data can no longer be retrieved.
        *
        *  \params ManusRevertProfile
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusCommitProfiles(manus_session_t session);

        /*! Calculate the shoulder and elbow position.
        *
        *   This calculates shoulder and elbow position based on the wrist and head position.
        *
        *  \param model The model of the body with both the inputs and the outputs of this calculation.
        *  \param settings The settings of the person used for the calculations, such as arm lengths.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusUpdateIK(manus_session_t session, ref ik_body_t inout_model);

        /*! Check if the device is connected.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern bool ManusIsConnected(manus_session_t session, device_type_t device);

        /*! Sets the coordinate system for all orientations and vectors returned from the SDK.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusSetCoordinateSystem(manus_session_t session, coor_up_t up, coor_handed_t handed);

        /*! Get the current battery level.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusGetBatteryLevel(manus_session_t session, device_type_t device, out UInt16 out_millivolts);

        /*! Get the signal strength.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusGetSignalStrength(manus_session_t session, device_type_t device, out UInt16 out_rssi);

        /*! Get the data from the glove and convert it to a skeletal model. The skeletal model is scaled
        *   according to the user profile.
        *
        *  \see ManusGetProfile
        *
        *  \params device The device type to query.
        *  \params out_hand The output struct, also contains the raw data from the device.
        *  \params timeout The time in milliseconds to block waiting on a new packet.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusGetHand(manus_session_t session, device_type_t device, out manus_hand_t out_hand);

        /*! Get the raw data from the glove, this does not include any of the skeletal model data. If you're also
        *   interested in the skeletal model you should call ManusGetHand() instead.
        *
        *  \see ManusGetHand
        *
        *  \params device The device type to query.
        *  \params out_hand The output struct.
        *  \params timeout The time in milliseconds to block waiting on a new packet.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusGetHandRaw(manus_session_t session, device_type_t device, out manus_hand_raw_t out_hand);

        /*! Gets a string that describes the error code. This can be used to display error messages
        *   in the user interface.
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern string ManusErrorString(UInt32 error);

        /*! Adds a virtual dongle that emulates two gloves connected to the API.
        *
        *  \see ManusRemoveDebugDevice
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusAddDebugDevice(manus_session_t session);

        /*! Removes the virtual dongle.
        *
        *  \see ManusAddDebugDevice
        */
        [DllImport("Manus", CallingConvention = CallingConvention.Cdecl)]
        public static extern manus_ret_t ManusRemoveDebugDevice(manus_session_t session);
    }
}
