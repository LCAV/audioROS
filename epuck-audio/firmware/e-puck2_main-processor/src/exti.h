#ifndef EXTI_H
#define EXTI_H

#ifdef __cplusplus
extern "C" {
#endif

//available flags for the exti_events
#define EXTI_EVENT_IMU_INT 1
#define EXTI_EVENT_IR_REMOTE_INT 2

extern event_source_t exti_events;

 /**
 * @brief   Starts the external interrupt processing service.
 */
void exti_start(void);
void exti_disable_ir_remote(void);
void exti_enable_ir_remote(void);

#ifdef __cplusplus
}
#endif



#endif /* EXTI_H */
