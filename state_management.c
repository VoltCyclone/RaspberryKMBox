/*
 * Hurricane PIOKMBox Firmware
*/

#include "state_management.h"
#include "defines.h"
#include <string.h>

static system_state_t g_system_state;

void system_state_init(system_state_t* state) {
    memset(state, 0, sizeof(system_state_t));
    // Set any non-zero initial values here
}

system_state_t* get_system_state(void) {
    return &g_system_state;
}

