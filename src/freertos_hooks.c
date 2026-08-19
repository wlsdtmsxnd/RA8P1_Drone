#include "code/actuator_manager.h"

#include "FreeRTOS.h"
#include "task.h"

void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char * p_task_name);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);


static void freertos_system_fault_reset(void)
{
    /* 异常上下文不再使用可能已损坏的任务栈，直接复位。 */
    NVIC_SystemReset();

    while (1)
    {
        /* NVIC_SystemReset 正常不会返回。 */
    }
}


/* RA8P1 Cortex-M85 由 PSPLIM 执行硬件栈边界检查，异常不得原地挂死。 */
void HardFault_Handler(void)
{
    freertos_system_fault_reset();
}


void MemManage_Handler(void)
{
    freertos_system_fault_reset();
}


void BusFault_Handler(void)
{
    freertos_system_fault_reset();
}


void UsageFault_Handler(void)
{
    freertos_system_fault_reset();
}


void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char * p_task_name)
{
    FSP_PARAMETER_NOT_USED(task);
    FSP_PARAMETER_NOT_USED(p_task_name);

    /* 尽力写入停机脉宽，再复位到默认 SAFE 构建的上电状态。 */
    (void) actuator_manager_inhibit();
    NVIC_SystemReset();

    while (1)
    {
        /* NVIC_SystemReset 正常不会返回。 */
    }
}
