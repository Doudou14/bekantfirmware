/**
 * BEKANT UI
 */

#include <xc.h>      // CLRWDT()
#include <stdbool.h>
#include "bui.h"
#include "bctrl.h"
#include <stdlib.h> // abs()

typedef enum {
    BUI_STOP,
    BUI_UP,
    BUI_MEM_UP,
    BUI_DOWN,
    BUI_MEM_DOWN,
    BUI_CAL_DOWN, // calibration: moving down until the encoder stalls
    BUI_CAL_UP    // calibration: moving up until the encoder stalls
} BUI_state_t;

/*
 * Travel limits, in encoder units.
 *
 * limit_min / limit_max are the positions at which BUI sends the STOP signal
 * to BCTRL, whatever the movement (manual or memory). They include the
 * BCTRL_DECEL_MARGIN so the deceleration finishes before the mechanical end
 * stop.
 *
 * They are measured automatically by the calibration run at the first boot
 * after flashing (see below). The defaults are only used until then, or if
 * the calibration is aborted.
 */
#define BUI_DEFAULT_LIMIT_MAX 0x1600
#define BUI_DEFAULT_LIMIT_MIN 0x0636

/*
 * Calibration
 *
 * There are no limit switches, so an end stop is detected when the encoder
 * value stops changing while the desk is commanded to move.
 *
 * bui_set_pos() is called once per bus transaction (20 x 5 ms = 100 ms),
 * so one "cycle" below is 100 ms.
 */
#define CAL_MAGIC             0xA5
#define CAL_HOLDOFF_CYCLES    20   // ignore stalls while the move starts (2 s)
#define CAL_STALL_CYCLES      4    // unchanged encoder for this long = stall
#define CAL_STALL_TOLERANCE   2    // encoder counts
#define CAL_TIMEOUT_CYCLES    600  // abort a phase after 60 s
#define CAL_MIN_RANGE         0x0200 // below this, the result is not credible

BUI_state_t bui_state = BUI_STOP;
int16_t high_pos;
int16_t low_pos;
int16_t limit_min = BUI_DEFAULT_LIMIT_MIN;
int16_t limit_max = BUI_DEFAULT_LIMIT_MAX;
int16_t cur_pos; // accessed by bui_input() and bui_set_pos()

static uint16_t cal_cycles;
static uint8_t cal_stall;
static int16_t cal_last_pos;
static int16_t cal_low_stop;
static volatile bool cal_save_pending = false;

typedef struct {
    int16_t low_pos;
    int16_t high_pos;
    int16_t limit_min;
    int16_t limit_max;
    uint8_t calibrated; // CAL_MAGIC once calibrated
} BUI_saved_t;

// Re-flashing resets the EEPROM to these values (calibrated = 0), which
// triggers the calibration run at the next boot.
__eeprom BUI_saved_t saved = {
    .high_pos = BUI_DEFAULT_LIMIT_MAX,
    .low_pos = BUI_DEFAULT_LIMIT_MIN,
    .limit_min = BUI_DEFAULT_LIMIT_MIN,
    .limit_max = BUI_DEFAULT_LIMIT_MAX,
    .calibrated = 0
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
    return cur_pos >= 0 && cur_pos >= limit_max;
}

static bool bui_at_lower_limit(void) {
    return cur_pos >= 0 && cur_pos <= limit_min;
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

/* ---------------------------- Calibration ---------------------------- */

static void bui_cal_start_phase(BUI_state_t state, BCTRL_state_t target) {
    bui_state = state;
    cal_cycles = 0;
    cal_stall = 0;
    cal_last_pos = cur_pos;
    bctrl_set_target(target);
}

/**
 * Detect that the encoder has stopped changing.
 * Called once per cycle during a calibration phase.
 */
static bool bui_cal_stalled(int16_t pos) {
    if (cal_cycles <= CAL_HOLDOFF_CYCLES) {
        // Start of the phase: the desk may still be decelerating from the
        // previous phase, or accelerating. Don't judge yet.
        cal_stall = 0;
        cal_last_pos = pos;
        return false;
    }

    if (abs(pos - cal_last_pos) <= CAL_STALL_TOLERANCE) {
        if (cal_stall < 255) {
            cal_stall++;
        }
    } else {
        cal_stall = 0;
        cal_last_pos = pos;
    }
    return cal_stall >= CAL_STALL_CYCLES;
}

/**
 * One calibration step, called from bui_set_pos() with the new position.
 * Phase 1: down until stall -> lower end stop.
 * Phase 2: up until stall -> upper end stop.
 * Nothing is stored unless both phases succeed.
 */
static void bui_cal_step(int16_t pos) {
    if (pos < 0 || cal_cycles >= CAL_TIMEOUT_CYCLES) {
        // Encoder error or desk not reaching an end stop: give up.
        // Previous limits are kept. If the desk was never calibrated,
        // it will be retried at next boot.
        bui_stop();
        return;
    }
    cal_cycles++;

    if (!bui_cal_stalled(pos)) {
        return;
    }

    if (bui_state == BUI_CAL_DOWN) {
        cal_low_stop = pos;
        bui_cal_start_phase(BUI_CAL_UP, BCTRL_UP);
        return;
    }

    // BUI_CAL_UP: upper end stop reached
    int16_t lo = cal_low_stop + BCTRL_DECEL_MARGIN;
    int16_t hi = pos - BCTRL_DECEL_MARGIN;
    bui_stop();

    if (hi - lo < CAL_MIN_RANGE) {
        return; // not credible, keep defaults
    }

    limit_min = lo;
    limit_max = hi;
    low_pos = lo;
    high_pos = hi;
    cal_save_pending = true; // EEPROM is written from bui_task(), not here
}

/**
 * Background task, to be called from the main loop.
 * Writes the calibration result into the EEPROM outside of the interrupt
 * handler, so that the slow EEPROM writes don't disturb the LIN bus timing.
 */
void bui_task(void) {
    if (!cal_save_pending) {
        return;
    }

    int16_t lo = limit_min;
    int16_t hi = limit_max;

    saved.limit_min = lo;
    CLRWDT();
    saved.limit_max = hi;
    CLRWDT();
    saved.low_pos = lo;
    CLRWDT();
    saved.high_pos = hi;
    CLRWDT();
    saved.calibrated = CAL_MAGIC; // last, so a partial write stays uncalibrated
    CLRWDT();

    cal_save_pending = false;
}

/* ------------------------------- Inputs ------------------------------ */

/**
 * Initialize from stored values.
 * Starts the calibration run if the desk has never been calibrated.
 */
void bui_init(void) {
    bool calibrated = (saved.calibrated == CAL_MAGIC);

    if (calibrated) {
        int16_t lo = saved.limit_min;
        int16_t hi = saved.limit_max;
        if (hi - lo >= CAL_MIN_RANGE) {
            limit_min = lo;
            limit_max = hi;
        } else {
            calibrated = false; // corrupted EEPROM
        }
    }

    // Stored positions are clamped to the limits, in case the EEPROM
    // is blank (0xffff) or corrupted.
    low_pos = bui_clamp(saved.low_pos, limit_min, limit_max);
    high_pos = bui_clamp(saved.high_pos, limit_min, limit_max);

    if (!calibrated) {
        bui_cal_start_phase(BUI_CAL_DOWN, BCTRL_DOWN);
    }
}

/**
 * 
 * @param input Input gesture
 */
void bui_input(INPUT_t input) {
    switch (bui_state) {
        case BUI_CAL_DOWN:
        case BUI_CAL_UP:
            // Any button press aborts the calibration (safety).
            if (input != INPUT_IDLE) {
                bui_stop();
            }
            break;

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
            } else if (input == INPUT_RECAL) {
                // Both buttons tapped 5 times: measure the end stops again.
                // The previous calibration stays valid if this one is aborted.
                bui_cal_start_phase(BUI_CAL_DOWN, BCTRL_DOWN);
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
 * limit (any move), so the motors never push against the end stops.
 * 
 * @param pos Current position
 */
void bui_set_pos(int16_t pos) {
    cur_pos = pos;

    if (bui_state == BUI_CAL_DOWN || bui_state == BUI_CAL_UP) {
        bui_cal_step(pos);
        return;
    }

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
 * The stored value is clamped to the limits so an unreachable
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
                            limit_min, limit_max);
        saved.low_pos = low_pos;
    } else {
        high_pos = bui_clamp(save_pos - BCTRL_DECEL_MARGIN,
                             limit_min, limit_max);
        saved.high_pos = high_pos;
    }

    // Click feedback
    bctrl_set_target(BCTRL_CLICK);
}
