// =============================================================================
// control_loop.c — ROV 6-DOF Control Loop  (STM32 port)
//
// Ported from: ROV_6DOF_TUNING.ino  (RP2350 / arduino-pico)
// Target:      STM32  (TIM7 fires at 50 Hz → dt = 0.02 s, always)
//
// External dependencies expected from the BSP / Day-2 infrastructure:
//   void     pwm_set_us(uint8_t ch, uint16_t us)   — set ESC PWM channel
//   uint32_t g_tick                                 — SysTick ms counter
//   bool     link_ok                                — set/cleared here; read by comms layer
// =============================================================================

#include "control_loop.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// SECTION 1 — CONSTANTS
// =============================================================================
#define N_DOF           6
#define N_THR           8

#define PWM_MIN         1100
#define PWM_MAX         1900
#define PWM_NEUTRAL     1500
#define PWM_RAMP_STEP   50          // µs per tick at 50 Hz → 2500 µs/s slew

#define CMD_TIMEOUT_MS  500u        // ms before failsafe triggers

#define THRUST_DEADZONE 0.02f

volatile uint8_t telem_pending = 0;
volatile uint8_t log_pending = 0;

// =============================================================================
// SECTION 2 — ALLOCATION MATRICES
//
// FORWARD  B  (6×8)   tau = B · T
// INVERSE  B⁺ (8×6)   T  = B⁺ · U
// =============================================================================
static const float B_forward[N_DOF][N_THR] = {
    { 0.0f,     0.0f,     0.0f,     0.0f,    0.7070f, -0.7070f,  0.7070f,  0.7070f},
    { 0.0f,     0.0f,     0.0f,     0.0f,   -0.7070f,  0.7070f,  0.7070f, -0.7070f},
    { 1.0f,     1.0f,     1.0f,     1.0f,    0.0f,     0.0f,     0.0f,     0.0f   },
    { 0.1700f, -0.1200f,  0.1200f, -0.1200f,-0.0140f,  0.0140f, -0.0140f,  0.0140f},
    {-0.1200f, -0.1200f,  0.1500f,  0.1500f,-0.0140f, -0.0140f, -0.0140f, -0.0140f},
    { 0.0f,     0.0f,     0.0f,     0.0f,    0.0346f,  0.0346f, -0.3250f,  0.3250f}
};

static const float B_pinv[N_THR][N_DOF] = {
    {-0.0349f, -0.0776f,  0.2493f,  2.0482f, -1.6573f, -0.2618f},
    {-0.0448f, -0.0089f,  0.3060f, -2.0430f, -2.0306f,  0.0586f},
    { 0.0439f,  0.0148f,  0.1989f,  1.6903f,  1.9984f, -0.0310f},
    { 0.0358f,  0.0717f,  0.2459f, -1.6955f,  1.6895f,  0.2341f},
    { 0.0765f, -0.5493f,  0.0036f, -0.0622f, -0.1885f, -1.0178f},
    { 0.0617f,  0.8347f,  0.0029f, -0.0503f, -0.1523f,  1.9589f},
    { 0.7072f,  0.7072f,  0.0000f,  0.0000f,  0.0000f,  0.0000f},
    { 0.6925f,  0.6768f, -0.0007f,  0.0120f,  0.0363f,  2.9767f}
};

// =============================================================================
// SECTION 3 — PID GAINS AND LIMITS
//                          Surge   Sway    Heave   Roll    Pitch   Yaw
// =============================================================================
static const float kp[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float ki[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float kd[N_DOF]        = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float kff[N_DOF]       = { 0.00f,  0.00f,  0.00f,  0.00f,  0.00f,  0.00f };
static const float FF_OFFSET[N_DOF] = { 0.0f,   0.0f,   0.0f,   0.0f,   0.0f,   0.0f  };
static const float I_CLAMP[N_DOF]   = { 20.0f,  20.0f,  20.0f,  5.0f,   5.0f,   5.0f  };
static const float U_MAX[N_DOF]     = { 80.0f,  80.0f,  80.0f,  20.0f,  20.0f,  20.0f };

// =============================================================================
// SECTION 4 — STATIC STATE VARIABLES
// =============================================================================
static float pose[N_DOF]     = {0};
static float target[N_DOF]   = {0};
static float errState[N_DOF] = {0};
static float errPrev[N_DOF]  = {0};
static float errInt[N_DOF]   = {0};
static float U[N_DOF]        = {0};
static float T_out[N_THR]    = {0};

static int      g_pwm_current[N_THR];
static bool     g_armed     = false;
static uint32_t last_cmd_ms = 0;

bool link_ok = false;

// =============================================================================
// SECTION 5 — INTERNAL HELPERS
// =============================================================================
static inline float wrapAngle180(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

static int thrust_to_pwm(float u)
{
    if (u >  1.0f) u =  1.0f;
    if (u < -1.0f) u = -1.0f;
    if (fabsf(u) < THRUST_DEADZONE) return PWM_NEUTRAL;
    return (u >= 0.0f)
        ? PWM_NEUTRAL + (int)(u * (float)(PWM_MAX - PWM_NEUTRAL))
        : PWM_NEUTRAL + (int)(u * (float)(PWM_NEUTRAL - PWM_MIN));
}

// =============================================================================
// SECTION 6 — PUBLIC API
// =============================================================================
void control_loop_init(void)
{
    for (int i = 0; i < N_THR; i++) {
        g_pwm_current[i] = PWM_NEUTRAL;
        pwm_set_us((uint8_t)i, (uint16_t)PWM_NEUTRAL);
    }
    memset(errInt,   0, sizeof(errInt));
    memset(errPrev,  0, sizeof(errPrev));
    memset(errState, 0, sizeof(errState));
    memset(U,        0, sizeof(U));
    memset(T_out,    0, sizeof(T_out));
    g_armed     = false;
    link_ok     = false;
    last_cmd_ms = 0;
}

void control_loop_tick(void)
{
    const float dt = 0.02f;

    checkCommandTimeout();

    if (g_armed) {
        computePID(dt);
        computeAllocation();
    }

    applyPWM();
}

void cmd_update(const float new_pose[N_DOF],
                const float new_target[N_DOF],
                bool        arm_flag)
{
    memcpy(pose,   new_pose,   N_DOF * sizeof(float));
    memcpy(target, new_target, N_DOF * sizeof(float));
    g_armed     = arm_flag;
    link_ok     = true;
    last_cmd_ms = g_tick;
}

// =============================================================================
// SECTION 7 — computePID()
// =============================================================================
void computePID(float dt)
{
    if (dt <= 0.0f) return;

    for (int i = 0; i < N_DOF; i++) {
        float raw_err = target[i] - pose[i];
        if (i == 5) raw_err = wrapAngle180(raw_err);

        errState[i] = raw_err;

        errInt[i] += errState[i] * dt;
        if      (errInt[i] >  I_CLAMP[i]) errInt[i] =  I_CLAMP[i];
        else if (errInt[i] < -I_CLAMP[i]) errInt[i] = -I_CLAMP[i];

        float eDot  = (errState[i] - errPrev[i]) / dt;
        errPrev[i]  = errState[i];

        float ff  = kff[i] * target[i] + FF_OFFSET[i];
        float raw = kp[i] * errState[i]
                  + ki[i] * errInt[i]
                  + kd[i] * eDot
                  + ff;

        if      (raw >  U_MAX[i]) raw =  U_MAX[i];
        else if (raw < -U_MAX[i]) raw = -U_MAX[i];
        U[i] = raw;
    }
}

// =============================================================================
// SECTION 8 — computeAllocation()
// =============================================================================
void computeAllocation(void)
{
    float T_trans[N_THR] = {0};
    float T_yaw[N_THR]   = {0};
    float T_vert[N_THR]  = {0};

    for (int i = 0; i < N_THR; i++) {
        T_trans[i] = B_pinv[i][0]*U[0] + B_pinv[i][1]*U[1];
        T_yaw[i]   = B_pinv[i][5]*U[5];
        T_vert[i]  = B_pinv[i][2]*U[2] + B_pinv[i][3]*U[3] + B_pinv[i][4]*U[4];
    }

    float maxV = 0.0f;
    for (int i = 0; i < 4; i++)
        if (fabsf(T_vert[i]) > maxV) maxV = fabsf(T_vert[i]);
    if (maxV > 1.0f)
        for (int i = 0; i < 4; i++) T_vert[i] /= maxV;

    float maxTn = 0.0f;
    for (int i = 4; i < N_THR; i++)
        if (fabsf(T_trans[i]) > maxTn) maxTn = fabsf(T_trans[i]);
    if (maxTn > 1.0f)
        for (int i = 4; i < N_THR; i++) T_trans[i] /= maxTn;

    float maxY = 0.0f;
    for (int i = 4; i < N_THR; i++)
        if (fabsf(T_yaw[i]) > maxY) maxY = fabsf(T_yaw[i]);
    if (maxY > 1.0f)
        for (int i = 4; i < N_THR; i++) T_yaw[i] /= maxY;

    for (int i = 0; i < N_THR; i++) {
        T_out[i] = (i < 4) ? T_vert[i] : (T_trans[i] + T_yaw[i]);
        if      (T_out[i] >  1.0f) T_out[i] =  1.0f;
        else if (T_out[i] < -1.0f) T_out[i] = -1.0f;
    }
}

// =============================================================================
// SECTION 9 — applyPWM()
// =============================================================================
void applyPWM(void)
{
    for (int i = 0; i < N_THR; i++) {
        if (!g_armed) {
            g_pwm_current[i] = PWM_NEUTRAL;
        } else {
            float thrust   = (i == 0) ? -T_out[i] : T_out[i];
            int target_pwm = thrust_to_pwm(thrust);
            int delta      = target_pwm - g_pwm_current[i];
            if      (delta >  PWM_RAMP_STEP) delta =  PWM_RAMP_STEP;
            else if (delta < -PWM_RAMP_STEP) delta = -PWM_RAMP_STEP;
            g_pwm_current[i] += delta;

            if      (g_pwm_current[i] > PWM_MAX) g_pwm_current[i] = PWM_MAX;
            else if (g_pwm_current[i] < PWM_MIN) g_pwm_current[i] = PWM_MIN;
        }

        pwm_set_us((uint8_t)i, (uint16_t)g_pwm_current[i]);
    }
}

// =============================================================================
// SECTION 10 — enterFailsafe()
// =============================================================================
void enterFailsafe(void)
{
    g_armed = false;
    link_ok = false;

    for (int i = 0; i < N_DOF; i++)
        errInt[i] = errPrev[i] = errState[i] = U[i] = 0.0f;
}

// =============================================================================
// SECTION 11 — checkCommandTimeout()
// =============================================================================
void checkCommandTimeout(void)
{
    if (last_cmd_ms != 0 && (g_tick - last_cmd_ms) > CMD_TIMEOUT_MS) {
        enterFailsafe();
    }
}


void control_loop_get_pwm(uint16_t *out, uint8_t len)
{
    if (out == NULL)
    {
        return;
    }

    if (len > N_THR)
    {
        len = N_THR;
    }

    for (uint8_t i = 0; i < len; i++)
    {
        out[i] = (uint16_t)g_pwm_current[i];
    }
}

bool control_loop_get_armed(void)
{
    return g_armed;
}

bool control_loop_get_link(void)
{
    return link_ok;
}
