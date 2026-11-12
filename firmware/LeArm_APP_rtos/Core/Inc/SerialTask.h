#ifndef __SERIALTASK_H
#define __SERIALTASK_H

#ifdef __cplusplus
extern "C" {
#endif

/* SerialTask 独占协议解析、命令分发、ACK/NACK 和状态遥测。 */
void serial_task_run(void *argument);

#ifdef __cplusplus
}
#endif













#endif /* __SERIALTASK_H */
