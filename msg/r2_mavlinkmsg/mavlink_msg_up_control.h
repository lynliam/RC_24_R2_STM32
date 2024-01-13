#pragma once
// MESSAGE UP_CONTROL PACKING

#define MAVLINK_MSG_ID_UP_CONTROL 4


typedef struct __mavlink_up_control_t {
 uint8_t cmd; /*<   命令*/
} mavlink_up_control_t;

#define MAVLINK_MSG_ID_UP_CONTROL_LEN 1
#define MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN 1
#define MAVLINK_MSG_ID_4_LEN 1
#define MAVLINK_MSG_ID_4_MIN_LEN 1

#define MAVLINK_MSG_ID_UP_CONTROL_CRC 58
#define MAVLINK_MSG_ID_4_CRC 58



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_UP_CONTROL { \
    4, \
    "UP_CONTROL", \
    1, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_up_control_t, cmd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_UP_CONTROL { \
    "UP_CONTROL", \
    1, \
    {  { "cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_up_control_t, cmd) }, \
         } \
}
#endif

/**
 * @brief Pack a up_control message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd   命令
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_up_control_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UP_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UP_CONTROL_LEN);
#else
    mavlink_up_control_t packet;
    packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UP_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UP_CONTROL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
}

/**
 * @brief Pack a up_control message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd   命令
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_up_control_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UP_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_UP_CONTROL_LEN);
#else
    mavlink_up_control_t packet;
    packet.cmd = cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_UP_CONTROL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_UP_CONTROL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
}

/**
 * @brief Encode a up_control struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param up_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_up_control_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_up_control_t* up_control)
{
    return mavlink_msg_up_control_pack(system_id, component_id, msg, up_control->cmd);
}

/**
 * @brief Encode a up_control struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param up_control C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_up_control_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_up_control_t* up_control)
{
    return mavlink_msg_up_control_pack_chan(system_id, component_id, chan, msg, up_control->cmd);
}

/**
 * @brief Send a up_control message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd   命令
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_up_control_send(mavlink_channel_t chan, uint8_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_UP_CONTROL_LEN];
    _mav_put_uint8_t(buf, 0, cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UP_CONTROL, buf, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
#else
    mavlink_up_control_t packet;
    packet.cmd = cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UP_CONTROL, (const char *)&packet, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
#endif
}

/**
 * @brief Send a up_control message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_up_control_send_struct(mavlink_channel_t chan, const mavlink_up_control_t* up_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_up_control_send(chan, up_control->cmd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UP_CONTROL, (const char *)up_control, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
#endif
}

#if MAVLINK_MSG_ID_UP_CONTROL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This variant of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_up_control_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, cmd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UP_CONTROL, buf, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
#else
    mavlink_up_control_t *packet = (mavlink_up_control_t *)msgbuf;
    packet->cmd = cmd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_UP_CONTROL, (const char *)packet, MAVLINK_MSG_ID_UP_CONTROL_MIN_LEN, MAVLINK_MSG_ID_UP_CONTROL_LEN, MAVLINK_MSG_ID_UP_CONTROL_CRC);
#endif
}
#endif

#endif

// MESSAGE UP_CONTROL UNPACKING


/**
 * @brief Get field cmd from up_control message
 *
 * @return   命令
 */
static inline uint8_t mavlink_msg_up_control_get_cmd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a up_control message into a struct
 *
 * @param msg The message to decode
 * @param up_control C-struct to decode the message contents into
 */
static inline void mavlink_msg_up_control_decode(const mavlink_message_t* msg, mavlink_up_control_t* up_control)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    up_control->cmd = mavlink_msg_up_control_get_cmd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_UP_CONTROL_LEN? msg->len : MAVLINK_MSG_ID_UP_CONTROL_LEN;
        memset(up_control, 0, MAVLINK_MSG_ID_UP_CONTROL_LEN);
    memcpy(up_control, _MAV_PAYLOAD(msg), len);
#endif
}
