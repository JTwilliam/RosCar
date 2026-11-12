/* Host regression of the actual safety facade and CubeMX startup function.
 * No hardware, motion, reset or firmware download is performed. */
#include <assert.h>
#include <setjmp.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "RobotSafety.h"
#include "RobotHealth.h"
#include "cmsis_os.h"

static osKernelState_t kernel_state;
static uint32_t ipsr;
static unsigned mutex_count, acquire_count, thread_count, queue_count, flags_count;
static unsigned fail_mutex_number;
static unsigned handles[16];
static bool acquire_fails, held, expect_error;
static uint16_t history;
static jmp_buf fatal_jump;

/* Including these real files allows checking startup ordering and private
 * pending-fault state without adding a production fault-clear interface. */
#include "../Core/Src/RobotSafety.c"
#include "../Core/Src/freertos.c"

uint32_t safety_host_ipsr(void) { return ipsr; }
uint32_t HAL_GetTick(void) { return 1234U; }
osKernelState_t osKernelGetState(void) { return kernel_state; }

osMutexId_t osMutexNew(const osMutexAttr_t *attr)
{
    assert(kernel_state == osKernelReady && ipsr == 0U);
    assert(attr != NULL && thread_count == 0U);
    ++mutex_count;
    if (mutex_count == fail_mutex_number) return NULL;
    assert(mutex_count < 16U);
    return &handles[mutex_count];
}

osStatus_t osMutexAcquire(osMutexId_t mutex, uint32_t timeout)
{
    assert(kernel_state == osKernelRunning && mutex != NULL);
    assert(timeout == osWaitForever && !held);
    ++acquire_count;
    if (acquire_fails) return osError;
    held = true;
    return osOK;
}

osStatus_t osMutexRelease(osMutexId_t mutex)
{
    assert(mutex != NULL && held);
    held = false;
    return osOK;
}

osThreadId_t osThreadNew(void (*function)(void *), void *argument,
                         const osThreadAttr_t *attr)
{
    (void)argument;
    /* This assertion fails if the safety init call is removed or moved later. */
    assert(robot_safety_initialized && !robot_safety_internal_fault_pending);
    assert(robot_safety_manager.state == SAFETY_STARTUP);
    assert(kernel_state == osKernelReady && function != NULL && attr != NULL);
    ++thread_count;
    return &handles[thread_count];
}

osMessageQueueId_t osMessageQueueNew(uint32_t count, uint32_t size,
                                     const osMessageQueueAttr_t *attr)
{
    assert(attr != NULL && size > 0U && (count == 1U || count == 4U));
    ++queue_count;
    return &handles[queue_count];
}

osEventFlagsId_t osEventFlagsNew(const osEventFlagsAttr_t *attr)
{
    assert(attr != NULL);
    ++flags_count;
    return &handles[0];
}

void robot_health_safety_event(uint16_t reason, uint32_t timestamp)
{ (void)reason; (void)timestamp; }
void robot_health_note_history(uint16_t bits, uint32_t timestamp)
{ (void)timestamp; history |= bits; }

void Error_Handler(void)
{
    assert(expect_error);
    longjmp(fatal_jump, 1);
}

/* Task entry points must only be created, never executed by this host test. */
void default_task_run(void *argument) { (void)argument; assert(false); }
void chassis_task_run(void *argument) { (void)argument; assert(false); }
void serial_task_run(void *argument) { (void)argument; assert(false); }
void arm_task_run(void *argument) { (void)argument; assert(false); }
void health_task_run(void *argument) { (void)argument; assert(false); }
void osThreadExit(void) { assert(false); }
void robot_health_reset_with_reason(uint16_t reason)
{ (void)reason; assert(false); }

static void fresh(void)
{
    memset(&robot_safety_manager, 0, sizeof(robot_safety_manager));
    robot_safety_mutex = NULL;
    robot_safety_initialized = false;
    robot_safety_internal_fault_pending = false;
    robot_safety_pending_time_valid = false;
    robot_safety_pending_time_ms = 0U;
    kernel_state = osKernelReady;
    ipsr = 0U;
    mutex_count = acquire_count = thread_count = queue_count = flags_count = 0U;
    fail_mutex_number = 0U;
    acquire_fails = held = expect_error = false;
    history = 0U;
}

static void test_bootstrap_before_every_worker(void)
{
    fresh();
    MX_FREERTOS_Init();
    assert(mutex_count == 3U && acquire_count == 0U);
    assert(thread_count == 5U && queue_count == 2U && flags_count == 1U);
    assert(robot_safety_initialized && robot_safety_manager.state == SAFETY_STARTUP);
    kernel_state = osKernelRunning;
    assert(robot_safety_get_state() == SAFETY_STARTUP);
    robot_safety_report_startup_ready(2000U);
    assert(robot_safety_get_state() == SAFETY_SAFE_IDLE);
    assert(history == 0U && !robot_safety_motion_allowed());
}

static void test_bootstrap_failure_never_creates_worker(void)
{
    fresh();
    fail_mutex_number = 3U; /* i2c, servo, then robotSafety */
    expect_error = true;
    if (setjmp(fatal_jump) == 0) {
        MX_FREERTOS_Init();
        assert(false);
    }
    assert(!robot_safety_initialized && robot_safety_internal_fault_pending);
    assert(thread_count == 0U);
}

static void test_uninitialized_access_reproduces_fault(void)
{
    fresh();
    kernel_state = osKernelRunning;
    assert(robot_safety_get_state() == SAFETY_FAULT);
    assert(robot_safety_internal_fault_pending);
    assert(!robot_safety_motion_allowed());
}

static void test_invalid_init_contexts_are_rejected(void)
{
    fresh();
    kernel_state = osKernelInactive;
    assert(!robot_safety_init(1U) && mutex_count == 0U);
    fresh();
    ipsr = 16U;
    assert(!robot_safety_init(1U) && mutex_count == 0U);
    fresh();
    kernel_state = osKernelRunning;
    assert(!robot_safety_init(1U) && mutex_count == 0U);
}

static void test_pending_fault_cannot_be_erased_by_init(void)
{
    fresh();
    assert(robot_safety_get_state() == SAFETY_FAULT);
    assert(robot_safety_init(2U));
    kernel_state = osKernelRunning;
    assert(robot_safety_get_state() == SAFETY_FAULT);
    assert(robot_safety_manager.fault_flags & ROBOT_FAULT_INTERNAL);
}

static void test_fault_and_duplicate_init_stay_latched(void)
{
    fresh();
    assert(robot_safety_init(1U));
    assert(!robot_safety_init(2U));
    assert(mutex_count == 1U);
    kernel_state = osKernelRunning;
    robot_safety_report_startup_ready(3U);
    robot_safety_latch_fault(ROBOT_FAULT_MOTOR, 4U);
    robot_safety_report_startup_ready(5U);
    assert(robot_safety_get_state() == SAFETY_FAULT);
    assert(!robot_safety_request_stop(1U, 6U));
    assert(robot_safety_request_boot(1U, 7U) == BOOT_DENIED_FAULT);
    assert(!robot_safety_init(8U)); /* Running-state reset is forbidden. */
    assert(robot_safety_get_state() == SAFETY_FAULT);
    assert(history & ROBOT_FAULT_MOTOR);
}

static void test_lock_failure_stops_motion(void)
{
    fresh();
    assert(robot_safety_init(1U));
    kernel_state = osKernelRunning;
    acquire_fails = true;
    assert(robot_safety_get_state() == SAFETY_FAULT);
    assert(!robot_safety_motion_allowed());
    assert(robot_safety_request_boot(1U, 2U) == BOOT_DENIED_FAULT);
    assert(!held);
}

static void test_stop_and_boot_handshake_still_required(void)
{
    fresh();
    assert(robot_safety_init(1U));
    kernel_state = osKernelRunning;
    robot_safety_report_startup_ready(2U);
    assert(robot_safety_request_boot(42U, 3U) == BOOT_DENIED_STOP_REQUIRED);
    assert(robot_safety_request_stop(42U, 4U));
    robot_safety_report_chassis_stopped(true, 5U);
    assert(!robot_safety_stop_completed(42U));
    robot_safety_report_arm_stopped(true, 6U);
    assert(robot_safety_stop_completed(42U));
    assert(robot_safety_request_boot(41U, 7U) == BOOT_DENIED_STOP_REQUIRED);
    assert(robot_safety_request_boot(42U, 8U) == BOOT_ALLOWED);
    assert(!robot_safety_motion_allowed());
}

int main(void)
{
    test_bootstrap_before_every_worker();
    test_bootstrap_failure_never_creates_worker();
    test_uninitialized_access_reproduces_fault();
    test_invalid_init_contexts_are_rejected();
    test_pending_fault_cannot_be_erased_by_init();
    test_fault_and_duplicate_init_stay_latched();
    test_lock_failure_stops_motion();
    test_stop_and_boot_handshake_still_required();
    puts("PASS: 8 safety/startup regression groups (host mocks, not hardware)");
    return 0;
}
