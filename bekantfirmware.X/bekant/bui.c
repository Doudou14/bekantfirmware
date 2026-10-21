/**
 * BEKANT UI
 */

#include "bui.h"
#include "bctrl.h"
#include <stdlib.h> // abs()

typedef enum {
    BUI_STOP,
    BUI_UP,
    BUI_MEM_UP,
    BUI_DOWN,
    BUI_MEM_DOWN
} BUI_state_t;

/*
 * Absolute travel limits, in encoder units.
 *
 * These are the positions at which BUI sends the STOP signal to BCTRL,
 * whatever the movement (manual or memory). They already include the
 * BCTRL_DECEL_MARGIN so the deceleration finishes before the mechanical
 * end stop. Same semantics as the saved high_pos / low_pos values.
 *
 * TODO: adjust to the real desk: move it slowly to each end, read cur_pos
 * at the mechanical end stop, then
 *   BUI_LIMIT_MAX = end_stop_high - BCTRL_DECEL_MARGIN
 *   BUI_LIMIT_MIN = end_stop_low  + BCTRL_DECEL_MARGIN
 * The defaults below match the factory values of the EEPROM.
 */
#define BUI_LIMIT_MAX 0x1600
#define BUI_LIMIT_MIN 0x0636

BUI_state_t bui_state = BUI_STOP;
int16_t high_pos;
int16_t low_pos;
int16_t cur_pos; // accessed by bui_input() and bui_set_pos()

typedef struct {
    int16_t low_pos;
    int16_t high_pos;
} BUI_saved_t;

//__EEPROM_DATA(0x36, 0x06, 0x00, 0x16, 0, 0, 0, 0);
__eeprom BUI_saved_t saved = {
    .high_pos = BUI_LIMIT_MAX,
    .low_pos = BUI_LIMIT_MIN
};

void bui_save_pos(int16_t save_pos);

static int16_t bui_clamp(int16_t value, int16_t min_value, int16_t max_value) {
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

/**
 * Whether the desk is at (or beyond) the upper limit.
 * A negative position means encoder error: the position is unknown,
 * so don't block the movement (otherwise the user could never recover).
 */
static bool bui_at_upper_limit(void) {
    return cur_pos >= 0 && cur_pos >= BUI_LIMIT_MAX;
}

static bool bui_at_lower_limit(void) {
    return cur_pos >= 0 && cur_pos <= BUI_LIMIT_MIN;
}

static void bui_stop(void) {
    bui_state = BUI_STOP;
    bctrl_set_target(BCTRL_STOP);
}

/**
 * Start (or switch to) a manual UP move, unless already at the upper limit.
 */
static void bui_go_up(void) {
    if (bui_at_upper_limit()) {
        bui_stop();
    } else {
        bui_state = BUI_UP;
        bctrl_set_target(BCTRL_UP);
    }
}

/**
 * Start (or switch to) a manual DOWN move, unless already at the lower limit.
 */
static void bui_go_down(void) {
    if (bui_at_lower_limit()) {
        bui_stop();
    } else {
        bui_state = BUI_DOWN;
        bctrl_set_target(BCTRL_DOWN);
    }
}

/**
 * Initialize from stored position values.
 * Stored values are clamped to the absolute limits, in case the EEPROM
 * is blank (0xffff) or corrupted.
 */
void bui_init(void) {
    low_pos = bui_clamp(saved.low_pos, BUI_LIMIT_MIN, BUI_LIMIT_MAX);
    high_pos = bui_clamp(saved.high_pos, BUI_LIMIT_MIN, BUI_LIMIT_MAX);
}

/**
 * 
 * @param input Input gesture
 */
void bui_input(INPUT_t input) {
    switch (bui_state) {
        case BUI_STOP:
            if (input == INPUT_UP) {
                bui_go_up();
            } else if (input == INPUT_DOWN) {
                bui_go_down();
            } else if (input == INPUT_MEM_UP && cur_pos < high_pos) {
                bui_state = BUI_MEM_UP;
                bctrl_set_target(BCTRL_UP);
            } else if (input == INPUT_MEM_DOWN && cur_pos > low_pos) {
                bui_state = BUI_MEM_DOWN;
                bctrl_set_target(BCTRL_DOWN);
            } else if (input == INPUT_SAVE) {
                bui_save_pos(cur_pos);
            }
            break;

        case BUI_UP:
            if (input == INPUT_MEM_UP && cur_pos < high_pos) {
                bui_state = BUI_MEM_UP;
            } else if (input == INPUT_IDLE) {
                bui_stop();
            } else if (input == INPUT_DOWN) {
                bui_go_down();
            }
            break;

        case BUI_DOWN:
            if (input == INPUT_MEM_DOWN && cur_pos > low_pos) {
                bui_state = BUI_MEM_DOWN;
            } else if (input == INPUT_IDLE) {
                bui_stop();
            } else if (input == INPUT_UP) {
                bui_go_up();
            }
            break;

        case BUI_MEM_UP:
            if (input == INPUT_UP) {
                bui_state = BUI_UP;
            } else if (input == INPUT_DOWN) {
                bui_go_down(); // switch directions
            }
            break;

        case BUI_MEM_DOWN:
            if (input == INPUT_UP) {
                bui_go_up(); // switch directions
            } else if (input == INPUT_DOWN) {
                bui_state = BUI_DOWN;
            }
            break;
    }
}

/**
 * Update this module's copy of the current position.
 * Expect to be called by BEKANT Control module after the LIN bus transaction
 * is completed and position is reported.
 * 
 * Stops the desk when it reaches the memory position (MEM moves) or the
 * absolute limit (any move), so the motors never push against the end stops.
 * 
 * @param pos Current position
 */
void bui_set_pos(int16_t pos) {
    cur_pos = pos;

    if (cur_pos < 0) {
        // Encoder error state: position unknown, can't enforce limits
        return;
    }

    switch (bui_state) {
        case BUI_UP:
            if (bui_at_upper_limit()) {
                bui_stop();
            }
            break;
        case BUI_MEM_UP:
            if (cur_pos >= high_pos || bui_at_upper_limit()) {
                bui_stop();
            }
            break;
        case BUI_DOWN:
            if (bui_at_lower_limit()) {
                bui_stop();
            }
            break;
        case BUI_MEM_DOWN:
            if (cur_pos <= low_pos || bui_at_lower_limit()) {
                bui_stop();
            }
            break;
        default:
            break;
    }
}

/**
 * Store the given save_pos encoder value into persistent memory.
 * Overwrite stored low_pos or high_pos, whichever is closer to save_pos.
 * 
 * If the given save_pos looks like an encoder error state (negative),
 * then don't save.
 * The stored value is clamped to the absolute limits so an unreachable
 * position can't be memorized.
 * 
 * @param save_pos new encoder value to store into persistent memory
 */
void bui_save_pos(int16_t save_pos) {
    if (save_pos < 0) {
        // Encoder value indicates error state
        return;
    }

    int16_t diff_high = abs(high_pos - save_pos);
    int16_t diff_low = abs(low_pos - save_pos);

    // Next time during a MEM move, BUI will send the STOP signal to BCTRL
    // before it reaches the target position, to allow for deceleration.
    if (diff_low < diff_high) {
        low_pos = bui_clamp(save_pos + BCTRL_DECEL_MARGIN,
                            BUI_LIMIT_MIN, BUI_LIMIT_MAX);
        saved.low_pos = low_pos;
    } else {
        high_pos = bui_clamp(save_pos - BCTRL_DECEL_MARGIN,
                             BUI_LIMIT_MIN, BUI_LIMIT_MAX);
        saved.high_pos = high_pos;
    }

    // Click feedback
    bctrl_set_target(BCTRL_CLICK);
}
