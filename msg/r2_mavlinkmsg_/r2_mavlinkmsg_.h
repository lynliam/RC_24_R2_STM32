/** @file
 *  @brief MAVLink comm protocol generated from r2_mavlinkmsg_.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_R2_MAVLINKMSG__H
#define MAVLINK_R2_MAVLINKMSG__H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_R2_MAVLINKMSG_.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#define MAVLINK_R2_MAVLINKMSG__XML_HASH -3388473571168506211

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{1, 51, 24, 24, 0, 0, 0}, {2, 241, 12, 12, 0, 0, 0}, {3, 182, 24, 24, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_R2_MAVLINKMSG_

// ENUM DEFINITIONS


/** @brief 上层机构控制命令 */
#ifndef HAVE_ENUM_UP_CONTROL_CMD
#define HAVE_ENUM_UP_CONTROL_CMD
typedef enum UP_CONTROL_CMD
{
   RESET=0, /* 无命令状态 复位 | */
   CHASE_BALL=1, /* 找球 | */
   CATCH_BALL=2, /* 抓球 | */
   PUT_BALL=3, /* 放球 | */
   UP_CONTROL_CMD_ENUM_END=4, /*  | */
} UP_CONTROL_CMD;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 3
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_speed.h"
#include "./mavlink_msg_gyro.h"
#include "./mavlink_msg_pose.h"

// base include



#if MAVLINK_R2_MAVLINKMSG__XML_HASH == MAVLINK_PRIMARY_XML_HASH
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_SPEED, MAVLINK_MESSAGE_INFO_GYRO, MAVLINK_MESSAGE_INFO_POSE}
# define MAVLINK_MESSAGE_NAMES {{ "GYRO", 2 }, { "POSE", 3 }, { "SPEED", 1 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_R2_MAVLINKMSG__H
