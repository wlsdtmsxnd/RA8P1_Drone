#include "motor_output.h"

#include "r_gpt.h"

/* PCLKD = 250 MHz，50 Hz 周期为 5,000,000 个计数。 */
#define MOTOR_PWM_PERIOD_COUNTS       (5000000U)
#define MOTOR_PWM_INITIAL_COUNTS      (250000U)
#define MOTOR_PWM_PERIOD_US           (20000U)

typedef struct
{
    gpt_instance_ctrl_t * p_ctrl;
    gpt_io_pin_t pin;
    uint32_t counts_per_us;
} motor_channel_t;

static gpt_instance_ctrl_t g_motor_gpt5_ctrl;
static gpt_instance_ctrl_t g_motor_gpt10_ctrl;
static gpt_instance_ctrl_t g_motor_gpt6_ctrl;

#define MOTOR_GPT_EXTENDED_CFG(enable_b)                         \
    {                                                            \
        .gtioca = { true, GPT_PIN_LEVEL_LOW },                   \
        .gtiocb = { (enable_b), GPT_PIN_LEVEL_LOW },             \
        .start_source = GPT_SOURCE_NONE,                         \
        .stop_source = GPT_SOURCE_NONE,                          \
        .clear_source = GPT_SOURCE_NONE,                         \
        .capture_a_source = GPT_SOURCE_NONE,                     \
        .capture_b_source = GPT_SOURCE_NONE,                     \
        .count_up_source = GPT_SOURCE_NONE,                      \
        .count_down_source = GPT_SOURCE_NONE,                    \
        .capture_filter_gtioca = GPT_CAPTURE_FILTER_NONE,        \
        .capture_filter_gtiocb = GPT_CAPTURE_FILTER_NONE,        \
        .capture_a_ipl = BSP_IRQ_DISABLED,                       \
        .capture_b_ipl = BSP_IRQ_DISABLED,                       \
        .compare_match_c_ipl = BSP_IRQ_DISABLED,                 \
        .compare_match_d_ipl = BSP_IRQ_DISABLED,                 \
        .compare_match_e_ipl = BSP_IRQ_DISABLED,                 \
        .compare_match_f_ipl = BSP_IRQ_DISABLED,                 \
        .capture_a_irq = FSP_INVALID_VECTOR,                     \
        .capture_b_irq = FSP_INVALID_VECTOR,                     \
        .compare_match_c_irq = FSP_INVALID_VECTOR,               \
        .compare_match_d_irq = FSP_INVALID_VECTOR,               \
        .compare_match_e_irq = FSP_INVALID_VECTOR,               \
        .compare_match_f_irq = FSP_INVALID_VECTOR,               \
        .compare_match_value = {0U, 0U, 0U, 0U, 0U, 0U},        \
        .compare_match_status = 0U,                              \
        .p_pwm_cfg = NULL,                                       \
        .gtior_setting.gtior = 0U,                              \
        .gtioca_polarity = GPT_GTIOC_POLARITY_NORMAL,            \
        .gtiocb_polarity = GPT_GTIOC_POLARITY_NORMAL             \
    }

static const gpt_extended_cfg_t g_motor_gpt5_extend =
    MOTOR_GPT_EXTENDED_CFG(true);
static const gpt_extended_cfg_t g_motor_gpt10_extend =
    MOTOR_GPT_EXTENDED_CFG(false);
static const gpt_extended_cfg_t g_motor_gpt6_extend =
    MOTOR_GPT_EXTENDED_CFG(false);

#define MOTOR_TIMER_CFG(channel_number, extend_cfg)              \
    {                                                            \
        .mode = TIMER_MODE_PWM,                                  \
        .period_counts = MOTOR_PWM_PERIOD_COUNTS,                 \
        .duty_cycle_counts = MOTOR_PWM_INITIAL_COUNTS,            \
        .source_div = (timer_source_div_t) 0,                     \
        .channel = (channel_number),                              \
        .p_callback = NULL,                                      \
        .p_context = NULL,                                       \
        .p_extend = &(extend_cfg),                                \
        .cycle_end_ipl = BSP_IRQ_DISABLED,                       \
        .cycle_end_irq = FSP_INVALID_VECTOR                       \
    }

static const timer_cfg_t g_motor_gpt5_cfg =
    MOTOR_TIMER_CFG(5U, g_motor_gpt5_extend);
static const timer_cfg_t g_motor_gpt10_cfg =
    MOTOR_TIMER_CFG(10U, g_motor_gpt10_extend);
static const timer_cfg_t g_motor_gpt6_cfg =
    MOTOR_TIMER_CFG(6U, g_motor_gpt6_extend);

static motor_channel_t g_motor_channels[MOTOR_OUTPUT_COUNT] =
{
    {&g_motor_gpt5_ctrl, GPT_IO_PIN_GTIOCA, 0U},  /* M1: P700, left front */
    {&g_motor_gpt5_ctrl, GPT_IO_PIN_GTIOCB, 0U},  /* M2: P701, right front */
    {&g_motor_gpt10_ctrl, GPT_IO_PIN_GTIOCA, 0U}, /* M3: P109, right rear */
    {&g_motor_gpt6_ctrl, GPT_IO_PIN_GTIOCA, 0U}   /* M4: P702, left rear */
};

static bool g_motor_output_ready = false;


static fsp_err_t motor_timer_open(gpt_instance_ctrl_t * p_ctrl,
                                  timer_cfg_t const * p_cfg,
                                  uint32_t * p_counts_per_us)
{
    fsp_err_t err;
    timer_info_t info;

    err = R_GPT_Open(p_ctrl, p_cfg);

    if (FSP_SUCCESS != err)
    {
        return err;
    }

    err = R_GPT_InfoGet(p_ctrl, &info);

    if (FSP_SUCCESS != err)
    {
        (void) R_GPT_Close(p_ctrl);
        return err;
    }

    *p_counts_per_us = info.period_counts / MOTOR_PWM_PERIOD_US;

    if (0U == *p_counts_per_us)
    {
        (void) R_GPT_Close(p_ctrl);
        return FSP_ERR_INVALID_HW_CONDITION;
    }

    return FSP_SUCCESS;
}


motor_output_status_t motor_output_init(void)
{
    fsp_err_t err;
    uint32_t counts_per_us;

    g_motor_output_ready = false;

    err = motor_timer_open(&g_motor_gpt5_ctrl,
                           &g_motor_gpt5_cfg,
                           &counts_per_us);

    if (FSP_SUCCESS != err)
    {
        return MOTOR_OUTPUT_STATUS_FSP_ERROR;
    }

    g_motor_channels[0].counts_per_us = counts_per_us;
    g_motor_channels[1].counts_per_us = counts_per_us;

    err = motor_timer_open(&g_motor_gpt10_ctrl,
                           &g_motor_gpt10_cfg,
                           &counts_per_us);

    if (FSP_SUCCESS != err)
    {
        (void) R_GPT_Close(&g_motor_gpt5_ctrl);
        return MOTOR_OUTPUT_STATUS_FSP_ERROR;
    }

    g_motor_channels[2].counts_per_us = counts_per_us;

    err = motor_timer_open(&g_motor_gpt6_ctrl,
                           &g_motor_gpt6_cfg,
                           &counts_per_us);

    if (FSP_SUCCESS != err)
    {
        (void) R_GPT_Close(&g_motor_gpt10_ctrl);
        (void) R_GPT_Close(&g_motor_gpt5_ctrl);
        return MOTOR_OUTPUT_STATUS_FSP_ERROR;
    }

    g_motor_channels[3].counts_per_us = counts_per_us;
    g_motor_output_ready = true;
    motor_output_all_stop();

    err = R_GPT_Start(&g_motor_gpt5_ctrl);

    if (FSP_SUCCESS == err)
    {
        err = R_GPT_Start(&g_motor_gpt10_ctrl);
    }

    if (FSP_SUCCESS == err)
    {
        err = R_GPT_Start(&g_motor_gpt6_ctrl);
    }

    if (FSP_SUCCESS != err)
    {
        g_motor_output_ready = false;
        return MOTOR_OUTPUT_STATUS_FSP_ERROR;
    }

    return MOTOR_OUTPUT_STATUS_OK;
}


motor_output_status_t motor_output_set_us(uint32_t motor_index,
                                          uint32_t pulse_us)
{
    fsp_err_t err;
    uint32_t duty_counts;

    if (false == g_motor_output_ready)
    {
        return MOTOR_OUTPUT_STATUS_NOT_READY;
    }

    if (motor_index >= MOTOR_OUTPUT_COUNT)
    {
        return MOTOR_OUTPUT_STATUS_ARGUMENT_ERROR;
    }

    if (pulse_us < MOTOR_OUTPUT_MIN_US)
    {
        pulse_us = MOTOR_OUTPUT_MIN_US;
    }
    else if (pulse_us > MOTOR_OUTPUT_MAX_US)
    {
        pulse_us = MOTOR_OUTPUT_MAX_US;
    }

    duty_counts = pulse_us * g_motor_channels[motor_index].counts_per_us;
    err = R_GPT_DutyCycleSet(g_motor_channels[motor_index].p_ctrl,
                             duty_counts,
                             (uint32_t) g_motor_channels[motor_index].pin);

    return (FSP_SUCCESS == err) ?
           MOTOR_OUTPUT_STATUS_OK :
           MOTOR_OUTPUT_STATUS_FSP_ERROR;
}


void motor_output_all_stop(void)
{
    uint32_t motor_index;

    for (motor_index = 0U;
         motor_index < MOTOR_OUTPUT_COUNT;
         motor_index++)
    {
        (void) motor_output_set_us(motor_index,
                                   MOTOR_OUTPUT_MIN_US);
    }
}


bool motor_output_is_ready(void)
{
    return g_motor_output_ready;
}
