#include "shared.h"
#include "board.h"

QueueHandle_t mb_attitude, mb_wheels, mb_balance, mb_drive;
TaskHandle_t h_safety, h_sensor, h_balance, h_motor, h_telemetry, h_logger;
volatile uint32_t hb_sensor, hb_balance, hb_motor;
stats_t g_stats;
volatile float g_vbatt = VBATT_NOMINAL_V;

static SemaphoreHandle_t s_gains_mutex;
static ctrl_gains_t s_gains;

static volatile robot_state_t s_state = STATE_BOOT;
static volatile uint8_t s_faults;
static volatile arm_req_t s_arm_req = ARM_REQ_NONE;

static QueueHandle_t make_mailbox(size_t item)
{
    QueueHandle_t q = xQueueCreate(1, item);
    configASSERT(q != NULL);
    return q;
}

void shared_init(const ctrl_gains_t *initial)
{
    mb_attitude = make_mailbox(sizeof(attitude_t));
    mb_wheels   = make_mailbox(sizeof(wheel_state_t));
    mb_balance  = make_mailbox(sizeof(balance_out_t));
    mb_drive    = make_mailbox(sizeof(drive_cmd_t));

    /* Publish zeros so the first xQueuePeek never fails. */
    attitude_t a = {0};
    wheel_state_t w = {0};
    balance_out_t b = {0};
    drive_cmd_t d = {0};
    xQueueOverwrite(mb_attitude, &a);
    xQueueOverwrite(mb_wheels, &w);
    xQueueOverwrite(mb_balance, &b);
    xQueueOverwrite(mb_drive, &d);

    s_gains_mutex = xSemaphoreCreateMutex();
    configASSERT(s_gains_mutex != NULL);
    if (initial) s_gains = *initial;
    else ctrl_default_gains(&s_gains);
}

void gains_get(ctrl_gains_t *out)
{
    xSemaphoreTake(s_gains_mutex, portMAX_DELAY);
    *out = s_gains;
    xSemaphoreGive(s_gains_mutex);
}

void gains_set(const ctrl_gains_t *in)
{
    xSemaphoreTake(s_gains_mutex, portMAX_DELAY);
    s_gains = *in;
    xSemaphoreGive(s_gains_mutex);
}

static bool finite_nonneg(float x)
{
    return x == x && x >= 0.0f && x < 1e6f;   /* rejects NaN, negatives, huge */
}

bool gains_set_loop(int loop, float kp, float ki, float kd)
{
    if (loop < 0 || loop >= LOOP_COUNT) return false;
    if (!finite_nonneg(kp) || !finite_nonneg(ki) || !finite_nonneg(kd)) return false;
    xSemaphoreTake(s_gains_mutex, portMAX_DELAY);
    s_gains.g[loop].kp = kp;
    s_gains.g[loop].ki = ki;
    s_gains.g[loop].kd = kd;
    xSemaphoreGive(s_gains_mutex);
    return true;
}

void gains_set_trim(float trim)
{
    if (trim != trim) return;
    if (trim >  DEG2RAD(10.0f)) trim =  DEG2RAD(10.0f);
    if (trim < -DEG2RAD(10.0f)) trim = -DEG2RAD(10.0f);
    xSemaphoreTake(s_gains_mutex, portMAX_DELAY);
    s_gains.angle_trim = trim;
    xSemaphoreGive(s_gains_mutex);
}

robot_state_t state_get(void) { return s_state; }
uint8_t faults_get(void) { return s_faults; }

void state_set(robot_state_t s, uint8_t faults)
{
    taskENTER_CRITICAL();
    s_state = s;
    s_faults = faults;
    taskEXIT_CRITICAL();
}

void arm_request(arm_req_t r) { s_arm_req = r; }

arm_req_t arm_request_take(void)
{
    taskENTER_CRITICAL();
    arm_req_t r = s_arm_req;
    s_arm_req = ARM_REQ_NONE;
    taskEXIT_CRITICAL();
    return r;
}
