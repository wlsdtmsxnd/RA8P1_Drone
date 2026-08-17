#include "rc_command.h"

#include "../driver/crsf.h"

#include <stddef.h>

/* AETR 通道顺序。 */
#define RC_ROLL_CHANNEL_INDEX          (0U)
#define RC_PITCH_CHANNEL_INDEX         (1U)
#define RC_THROTTLE_CHANNEL_INDEX      (2U)
#define RC_YAW_CHANNEL_INDEX           (3U)
#define RC_ARM_CHANNEL_INDEX           (4U)
#define RC_MODE_CHANNEL_INDEX          (5U)

/* 本机 RadioMaster Pocket + ELRS 实测端点和中位。 */
#define RC_ROLL_MIN_US                 (989U)
#define RC_ROLL_CENTER_US              (1502U)
#define RC_ROLL_MAX_US                 (2011U)
#define RC_PITCH_MIN_US                (989U)
#define RC_PITCH_CENTER_US             (1490U)
#define RC_PITCH_MAX_US                (2001U)
#define RC_YAW_MIN_US                  (990U)
#define RC_YAW_CENTER_US               (1506U)
#define RC_YAW_MAX_US                  (2011U)

/* 中位死区和油门有效范围。 */
#define RC_CENTER_DEADBAND_US          (20U)
#define RC_THROTTLE_START_US           (1050U)
#define RC_THROTTLE_MAX_US             (2010U)

/* CH5 安全档位和 CH6 三档分界。 */
#define RC_ARM_LOW_MAX_US              (1300U)
#define RC_ARM_HIGH_MIN_US             (1700U)
#define RC_MODE_LOW_MAX_US             (1250U)
#define RC_MODE_HIGH_MIN_US            (1750U)


static float rc_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }

    if (value > maximum)
    {
        return maximum;
    }

    return value;
}


/* 带中位死区的非对称双边归一化。 */
static float rc_normalize_centered(uint16_t value_us,
                                   uint16_t minimum_us,
                                   uint16_t center_us,
                                   uint16_t maximum_us)
{
    uint16_t lower_deadband_us =
        (uint16_t) (center_us - RC_CENTER_DEADBAND_US);
    uint16_t upper_deadband_us =
        (uint16_t) (center_us + RC_CENTER_DEADBAND_US);
    float normalized;

    if (value_us < lower_deadband_us)
    {
        normalized = -((float) lower_deadband_us - (float) value_us) /
                     ((float) lower_deadband_us - (float) minimum_us);
    }
    else if (value_us > upper_deadband_us)
    {
        normalized = ((float) value_us - (float) upper_deadband_us) /
                     ((float) maximum_us - (float) upper_deadband_us);
    }
    else
    {
        normalized = 0.0f;
    }

    return rc_clamp(normalized, -1.0f, 1.0f);
}


static float rc_normalize_throttle(uint16_t value_us)
{
    float normalized;

    if (value_us <= RC_THROTTLE_START_US)
    {
        return 0.0f;
    }

    normalized = ((float) value_us - (float) RC_THROTTLE_START_US) /
                 ((float) RC_THROTTLE_MAX_US -
                  (float) RC_THROTTLE_START_US);

    return rc_clamp(normalized, 0.0f, 1.0f);
}


static rc_mode_t rc_decode_mode(uint16_t value_us)
{
    if (value_us <= RC_MODE_LOW_MAX_US)
    {
        return RC_MODE_LOW;
    }

    if (value_us >= RC_MODE_HIGH_MIN_US)
    {
        return RC_MODE_HIGH;
    }

    return RC_MODE_MIDDLE;
}


void rc_command_get(rc_command_t * p_command)
{
    crsf_data_t rc_data;
    uint16_t throttle_us;
    uint16_t arm_us;

    if (NULL == p_command)
    {
        return;
    }

    *p_command = (rc_command_t) {0};
    crsf_get_data(&rc_data);

    if (false == rc_data.connected)
    {
        return;
    }

    throttle_us = rc_data.channel_us[RC_THROTTLE_CHANNEL_INDEX];
    arm_us = rc_data.channel_us[RC_ARM_CHANNEL_INDEX];

    p_command->roll = rc_normalize_centered(
        rc_data.channel_us[RC_ROLL_CHANNEL_INDEX],
        RC_ROLL_MIN_US,
        RC_ROLL_CENTER_US,
        RC_ROLL_MAX_US);

    p_command->pitch = rc_normalize_centered(
        rc_data.channel_us[RC_PITCH_CHANNEL_INDEX],
        RC_PITCH_MIN_US,
        RC_PITCH_CENTER_US,
        RC_PITCH_MAX_US);

    p_command->throttle = rc_normalize_throttle(throttle_us);

    p_command->yaw = rc_normalize_centered(
        rc_data.channel_us[RC_YAW_CHANNEL_INDEX],
        RC_YAW_MIN_US,
        RC_YAW_CENTER_US,
        RC_YAW_MAX_US);

    p_command->mode = rc_decode_mode(
        rc_data.channel_us[RC_MODE_CHANNEL_INDEX]);
    p_command->connected = true;
    p_command->throttle_low = throttle_us <= RC_THROTTLE_START_US;
    p_command->arm_switch_low = arm_us <= RC_ARM_LOW_MAX_US;
    p_command->arm_switch_high = arm_us >= RC_ARM_HIGH_MIN_US;
}
