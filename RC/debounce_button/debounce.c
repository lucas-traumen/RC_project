#include "debounce.h"

void Debounce_Init(Debounce_t *db, uint8_t initial, uint16_t db_ms)
{
    db->raw = initial;
    db->stable = initial;
    db->last_stable = initial;
    db->last_change = HAL_GetTick();
    db->debounce_ms = db_ms;
}

void Debounce_Update(Debounce_t *db, uint8_t raw_now)
{
    uint32_t now = HAL_GetTick();

    if (raw_now != db->raw) {
        db->raw = raw_now;
        db->last_change = now;
    }

    if ((now - db->last_change) >= db->debounce_ms) {
        db->last_stable = db->stable;
        db->stable = raw_now;
    }
}

uint8_t Debounce_IsPressedEdge(Debounce_t *db)
{
    return (db->last_stable == 1 && db->stable == 0);
}

uint8_t Debounce_IsReleasedEdge(Debounce_t *db)
{
    return (db->last_stable == 0 && db->stable == 1);
}
