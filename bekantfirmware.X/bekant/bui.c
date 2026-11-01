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
#define CAL_ERR_CYCLES        10   // abort after 1 s of negative encoder values

/*
 * Tilt protection
 *
 * The two legs have their own encoder. During a move both should advance
 * together, so the difference between them should stay about constant.
 * The absolute difference at rest is just an offset between the encoders, so
 * the difference is measured relative to its value at the start of the move.
 * If it drifts by more than TILT_MAX_DIFF for TILT_CONFIRM_CYCLES cycles
 * (100 ms each), one leg is lagging behind: stop the desk.
 *
 * TODO: TILT_MAX_DIFF is a provisional value. It has to be larger than the
 * normal drift over a full travel and as small as possible above that.
 */
#define TILT_MAX_DIFF         0x0040 // encoder counts
#define TILT_CONFIRM_CYCLES   2
#define CAL_MIN_RANGE         0x0200 // below this, the result is not credible

BUI_state_t bui_state = BUI_STOP;
int16_t high_pos;
int16_t low_pos;
int16_t limit_min = BUI_DEFAULT_LIMIT_MIN;
int16_t limit_max = BUI_DEFAULT_LIMIT_MAX;
int16_t cur_pos; // accessed by bui_input() and bui_set_pos()

static uint16_t cal_cycles;
static uint8_t cal_stall;
static uint8_t cal_err;
static int16_t cal_last_pos;
static int16_t cal_low_stop;
static volatile bool cal_save_pending = false;
static uint8_t cal_save_step; // field being written by bui_save_step()
static bool bui_loaded = false; // EEPROM read (done from the interrupt handler)
static uint8_t cal_save_tries;

typedef struct {
    int16_t low_pos;
    int16_t high_pos;
} BUI_saved_t;

// Calibration result. It is stored twice, each copy with a checksum, so that
// a copy corrupted or half-written by a power cut is detected and the other
// one is used.
typedef struct {
    int16_t limit_min;
    int16_t limit_max;
    uint8_t check;
} BUI_cal_t;

__eeprom BUI_saved_t saved = {
    .high_pos = BUI_DEFAULT_LIMIT_MAX,
    .low_pos = BUI_DEFAULT_LIMIT_MIN
};

// Re-flashing resets the EEPROM to these values. The checksum of the default
// limits is not 0, so both copies are invalid ("not calibrated"), which
// triggers the calibration run at the next boot.
__eeprom BUI_cal_t cal_a = {
    .limit_min = BUI_DEFAULT_LIMIT_MIN,
    .limit_max = BUI_DEFAULT_LIMIT_MAX,
    .check = 0
};
__eeprom BUI_cal_t cal_b = {
    .limit_min = BUI_DEFAULT_LIMIT_MIN,
    .limit_max = BUI_DEFAULT_LIMIT_MAX,
    .check = 0
};

void bui_save_pos(int16_t save_pos);

static uint8_t bui_cal_check(int16_t lo, int16_t hi) {
    return (uint8_t)(CAL_MAGIC ^ (uint8_t)lo ^ (uint8_t)(lo >> 8)
            ^ (uint8_t)hi ^ (uint8_t)(hi >> 8));
}

/**
 * Whether a stored copy is usable: credible limits and matching checksum.
 * A blank (0xff) or half-written copy is rejected.
 */
static bool bui_cal_valid(int16_t lo, int16_t hi, uint8_t check) {
    return lo >= 0 && hi - lo >= CAL_MIN_RANGE
            && check == bui_cal_check(lo, hi);
}

#define BUI_CAL_MATCH(copy, lo, hi, chk) \
    (copy.limit_min == (lo) && copy.limit_max == (hi) && copy.check == (chk))

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

/* ------------------------- Tilt protection --------------------------- */

static bool tilt_armed = false;  // reference difference captured for this move
static int16_t tilt_ref;         // encoder difference at the start of the move
static uint8_t tilt_count;       // consecutive cycles over the threshold
static bool tilt_fault = false;  // latched: manual moves refused

/**
 * Called once per cycle while the desk is moving (not in calibration).
 */
static void bui_tilt_update(void) {
    if (!bctrl_enc_valid) {
        return; // encoder error: no reliable difference this cycle
    }

    if (!tilt_armed) {
        tilt_ref = bctrl_enc_diff;
        tilt_armed = true;
        tilt_count = 0;
        return;
    }

    if (abs(bctrl_enc_diff - tilt_ref) > TILT_MAX_DIFF) {
        tilt_count++;
        if (tilt_count >= TILT_CONFIRM_CYCLES) {
            tilt_fault = true;
            bui_stop();
        }
    } else {
        tilt_count = 0;
    }
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
    cal_err = 0;
    cal_last_pos = cur_pos;
    // Calibration drives both legs against their end stops, which levels
    // them again: this clears a tilt fault.
    tilt_fault = false;
    tilt_armed = false;
    tilt_count = 0;
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
    if (cal_cycles >= CAL_TIMEOUT_CYCLES) {
        // Desk not reaching an end stop: give up.
        // Previous limits are kept. If the desk was never calibrated,
        // it will be retried at next boot.
        bui_stop();
        return;
    }
    cal_cycles++;

    if (pos < 0) {
        // The very first transaction after the bus scan reports a negative
        // value (the BCTRL_AFTER_SCAN command carries 0xfff6), and the
        // encoders can report errors. Ignore these values for a while,
        // but never use them; give up if the error persists.
        cal_err++;
        if (cal_err >= CAL_ERR_CYCLES) {
            bui_stop();
        }
        return;
    }
    cal_err = 0;

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
    // Keep the user's memorized positions: only bring them back inside the
    // new limits if they fall outside. (On a first calibration after flashing
    // they are the factory defaults, which end up on or near the limits.)
    low_pos = bui_clamp(low_pos, lo, hi);
    high_pos = bui_clamp(high_pos, lo, hi);
    cal_save_tries = 0;
    cal_save_step = 0;
    cal_save_pending = true; // written by bui_save_step(), one field per cycle
}

/**
 * Saves the calibration result into the EEPROM, one field per call.
 *
 * Called from bui_set_pos(), i.e. once per bus cycle (100 ms), when the bus is
 * idle until the next cycle. Writing one field at a time keeps each EEPROM
 * write short (a few ms), like the SAVE gesture. It does not depend on the
 * main loop.
 *
 * Copy A first, then copy B. At boot A is preferred, so a power cut while
 * writing A leaves the previous B, and a power cut while writing B leaves
 * the new A. The checksum is written last in each copy, so a partial copy
 * stays invalid.
 */
static void bui_save_step(void) {
    int16_t lo = limit_min;
    int16_t hi = limit_max;
    uint8_t chk = bui_cal_check(lo, hi);

    switch (cal_save_step) {
        case 0: cal_a.limit_min = lo; break;
        case 1: cal_a.limit_max = hi; break;
        case 2: cal_a.check = chk; break;
        case 3: cal_b.limit_min = lo; break;
        case 4: cal_b.limit_max = hi; break;
        case 5: cal_b.check = chk; break;
        case 6:
            // Memorized positions: only rewritten if the new limits moved them
            if (saved.low_pos != low_pos) {
                saved.low_pos = low_pos;
            }
            break;
        case 7:
            if (saved.high_pos != high_pos) {
                saved.high_pos = high_pos;
            }
            break;
        default:
            // Everything written: read it back before declaring success.
            if (BUI_CAL_MATCH(cal_a, lo, hi, chk)
                    && BUI_CAL_MATCH(cal_b, lo, hi, chk)
                    && saved.low_pos == low_pos
                    && saved.high_pos == high_pos) {
                cal_save_pending = false;
                // Click: confirms the limits were read back correctly
                // from the EEPROM.
                bctrl_set_target(BCTRL_CLICK);
            } else {
                // Write failed: start again, but not forever.
                cal_save_tries++;
                if (cal_save_tries >= 3) {
                    cal_save_pending = false;
                } else {
                    cal_save_step = 0;
                }
            }
            return;
    }
    CLRWDT();
    cal_save_step++;
}

/* ------------------------------- Inputs ------------------------------ */

/**
 * Load the stored values.
 * Starts the calibration run if no valid calibration is stored.
 *
 * Runs from the interrupt handler, on the first call to bui_set_pos():
 * XC8 doesn't allow __eeprom variables to be accessed from both the main
 * program and the interrupt handler, so every EEPROM access of this module
 * is done from the interrupt handler.
 */
static void bui_load(void) {
    bool calibrated;

    int16_t lo = cal_a.limit_min;
    int16_t hi = cal_a.limit_max;
    if (bui_cal_valid(lo, hi, cal_a.check)) {
        calibrated = true;
    } else {
        // Copy A invalid (or never written): try copy B
        lo = cal_b.limit_min;
        hi = cal_b.limit_max;
        calibrated = bui_cal_valid(lo, hi, cal_b.check);
    }

    if (calibrated) {
        limit_min = lo;
        limit_max = hi;
    }

    if (calibrated) {
        // Stored positions are clamped to the limits, in case the EEPROM
        // is blank (0xffff) or corrupted.
        low_pos = bui_clamp(saved.low_pos, limit_min, limit_max);
        high_pos = bui_clamp(saved.high_pos, limit_min, limit_max);
    } else {
        // The real limits are not known yet: don't squeeze the user's
        // positions into the default limits. The calibration will bring
        // them inside the measured limits when it ends. Only reject
        // obviously invalid values (blank EEPROM reads as -1).
        low_pos = bui_clamp(saved.low_pos, 0, 0x7fff);
        high_pos = bui_clamp(saved.high_pos, 0, 0x7fff);
    }

    if (!calibrated) {
        bui_cal_start_phase(BUI_CAL_DOWN, BCTRL_DOWN);
    }
}

/**
 * Initialize the module. The stored values are loaded a moment later, at the
 * first bus cycle (see bui_load()); until then, buttons are ignored.
 */
void bui_init(void) {
    bui_loaded = false;
}

/**
 * 
 * @param input Input gesture
 */
void bui_input(INPUT_t input) {
    if (!bui_loaded) {
        return; // stored values not loaded yet (first bus cycle)
    }

    switch (bui_state) {
        case BUI_CAL_DOWN:
        case BUI_CAL_UP:
            // Any button press aborts the calibration (safety).
            if (input != INPUT_IDLE) {
                bui_stop();
            }
            break;

        case BUI_STOP:
            if (tilt_fault && (input == INPUT_UP || input == INPUT_DOWN
                    || input == INPUT_MEM_UP || input == INPUT_MEM_DOWN)) {
                // The tilt protection stopped the desk: refuse to move
                // until a recalibration (5 taps) or a power cycle.
                break;
            }
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
            } else if (input == INPUT_RECAL && !cal_save_pending) {
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

    if (!bui_loaded) {
        bui_loaded = true;
        bui_load(); // may start the calibration
        return;
    }

    if (cal_save_pending) {
        bui_save_step();
    }

    if (bui_state == BUI_CAL_DOWN || bui_state == BUI_CAL_UP) {
        bui_cal_step(pos);
        return;
    }

    if (bui_state == BUI_STOP) {
        tilt_armed = false; // next move captures a new reference
        tilt_count = 0;
    } else {
        bui_tilt_update(); // may stop the desk
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

    if (cal_save_pending) {
        // The calibration result is being written (about 1 s): don't start a
        // second EEPROM write at the same time.
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
