#ifndef  _MAVLINK_CONVERSIONS_H_
#define  _MAVLINK_CONVERSIONS_H_

/* enable math defines on Windows */
#ifdef _MSC_VER
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#endif
#include <math.h>

#ifndef M_PI_2
    #define M_PI_2 ((float)asin(1))
#endif

/**
 * @file mavlink_conversions.h
 *
 * These conversion functions follow the NASA rotation standards definition file
 * available online.
 *
 * Their intent is to lower the barrier for MAVLink adopters to use gimbal-lock free
 * (both rotation matrices, sometimes called DCM, and quaternions are gimbal-lock free)
 * rotation representations. Euler angles (roll, pitch, yaw) will be phased out of the
 * protocol as widely as possible.
 *
 * @author James Goppert
 */


/**
 * Converts a quaternion to a rotation matrix
 *
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 * @param dcm a 3x3 rotation matrix
 */
inline void mavlink_quaternion_to_dcm(const float quaternion[4], float dcm[3][3]);


/**
 * Converts a rotation matrix to euler angles
 *
 * @param dcm a 3x3 rotation matrix
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 */
inline void mavlink_dcm_to_euler(const float dcm[3][3], float* roll, float* pitch, float* yaw);


/**
 * Converts a quaternion to euler angles
 *
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 */
inline void mavlink_quaternion_to_euler(const float quaternion[4], float* roll, float* pitch, float* yaw);


/**
 * Converts euler angles to a quaternion
 *
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 */
inline void mavlink_euler_to_quaternion(float roll, float pitch, float yaw, float quaternion[4]);


/**
 * Converts a rotation matrix to a quaternion
 *
 * @param dcm a 3x3 rotation matrix
 * @param quaternion a [w, x, y, z] ordered quaternion (null-rotation being 1 0 0 0)
 */
inline void mavlink_dcm_to_quaternion(const float dcm[3][3], float quaternion[4]);


/**
 * Converts euler angles to a rotation matrix
 *
 * @param roll the roll angle in radians
 * @param pitch the pitch angle in radians
 * @param yaw the yaw angle in radians
 * @param dcm a 3x3 rotation matrix
 */
inline void mavlink_euler_to_dcm(float roll, float pitch, float yaw, float dcm[3][3]);

#endif
