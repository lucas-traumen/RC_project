#ifndef DEBOUNCE_H
#define DEBOUNCE_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

typedef struct {
    uint8_t raw;
    uint8_t stable;
    uint8_t last_stable;
    uint32_t last_change;
    uint16_t debounce_ms;
} Debounce_t;

void Debounce_Init(Debounce_t *db, uint8_t initial, uint16_t db_ms);
void Debounce_Update(Debounce_t *db, uint8_t raw_now);
uint8_t Debounce_IsPressedEdge(Debounce_t *db);
uint8_t Debounce_IsReleasedEdge(Debounce_t *db);

#endif
