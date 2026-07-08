#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdbool.h>
#include <stdint.h>

/* ── board-level LED driver (platform-abstracted header) ──
 *
 * Only STATE_LED  (PC6) is actively used.
 * PA4/LED1, PA5/LED2, PC4/LED3 are reserved on the board but not driven here.
 */

void BspLed_Init(void);
void BspLed_SetStateLed(bool on);

#endif /* BSP_LED_H */
