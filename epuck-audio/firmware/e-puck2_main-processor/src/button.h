#ifndef BUTTON_H
#define BUTTON_H

#ifdef __cplusplus
extern "C" {
#endif

uint8_t button_get_state(void);
void button_set_state(uint8_t state);
uint8_t button_is_pressed(void);

#ifdef __cplusplus
}
#endif

#endif
