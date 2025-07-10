/*
#################################################################
Klipper Touch Sensor Module @ Albatross
Description: This module provides low level klipper interface for spi-based touch sensor (MCP3462R-ADC)
Date: 2025-06-05
Version: 0.1
Owner: Mo
#################################################################
*/

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "board/gpio.h"
#include "board/irq.h"
#include "basecmd.h"
#include "board/misc.h"
#include "command.h"
#include "sched.h"
#include "spicmds.h"
#include "touch_sensor_mcp3462r.h"
#include "debug_utilities.h"
// -----------------------------------------------------------------------------

#define ADC_ACTIVE_STATE 0

// Global pointer to the ADC instance
static struct mcp3462r_adc *mcp_adc_ptr = NULL;
static struct RollingAverage rollingAvg_hndler;
static struct RollingAverage probe_avg_hndler;

// -----------------------------------------------------------------------------
// Utility Functions
// -----------------------------------------------------------------------------
void rolling_avg_init(struct RollingAverage *ra, int size, uint_fast8_t (*periodic_func)(struct timer *), 
                      uint32_t rest_ticks) {
    ra->size = size;
    ra->index = 0;
    ra->count = 0;
    ra->sum = 0;
    ra->lst_avg_value = 0.0f;
    ra->timer.func = periodic_func; 
    ra->rest_ticks = rest_ticks;
    for (int i = 0; i < BUFFER_SIZE; i++) {
        ra->buffer[i] = 0.0f;
    }
    ra->running_flag = 0;
}

float rolling_avg_push(struct RollingAverage *ra, float value) {
    if (ra == NULL || ra->size == 0 || ra->size > BUFFER_SIZE)
        return 0.0f;  // Safety check
    
    if (ra->count < ra->size) 
        ra->count++;
    else
        ra->sum -= ra->buffer[ra->index];  
    

    ra->buffer[ra->index] = value;
    ra->sum += value;

    ra->index = (ra->index + 1) % ra->size;
    ra->lst_avg_value = ra->sum / ra->count;
    return ra->lst_avg_value;
}

float rolling_avg_push_multiple(struct RollingAverage *ra, float *values, int count) {
    for (int i = 0; i < count; i++) {
        rolling_avg_push(ra, values[i]);
    }
    return ra->lst_avg_value;
}

float rolling_avg_get_last(struct RollingAverage *ra) {
    return ra->lst_avg_value;
}

void rolling_avg_pause(struct RollingAverage *ra) {
    irq_disable();
    sched_del_timer(&ra->timer);
    ra->running_flag = 0;
    ra->timer.func = NULL; 
    irq_enable();
}
void rolling_avg_resume(struct RollingAverage *ra) {
    ra->timer.func = periodic_read_event;
    irq_disable();
    ra->running_flag = 1;
    ra->timer.waketime = timer_read_time() + ra->rest_ticks;
    sched_add_timer(&ra->timer);
    irq_enable();
}

void rolling_avg_reset(struct RollingAverage *ra) {
    ra->index = 0;
    ra->count = 0;
    ra->sum = 0;
    ra->lst_avg_value = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++) 
        ra->buffer[i] = 0.0f;
    ra->running_flag = 0;
}
// Check if ADC data is ready
int8_t mcp3462r_is_data_ready(struct mcp3462r_adc *mcp3462r) {
    return gpio_in_read(mcp3462r->adc_ready_pin) == ADC_ACTIVE_STATE;
}

// -----------------------------------------------------------------------------
// Timer Event Handlers
// -----------------------------------------------------------------------------

// Terminator event: called at the end of a session to reset state
static uint_fast8_t mcp3462r_terminator_event(struct timer *t) {
    gpio_out_write(mcp_adc_ptr->trigger_out_pin, 0);
    sched_del_timer(&mcp_adc_ptr->timer);
    mcp_adc_ptr->active_session_flag = 0; // Ensure session flag is cleared
    DBG_INFO("Terminator event triggered session flag is= %u", mcp_adc_ptr->active_session_flag);
    return SF_DONE;
}

// Main event: handles ADC data polling and session logic
static uint_fast8_t mcp3462r_event(struct timer *t) {
    uint16_t doneP = 0, data = 0;
    uint16_t rolling_avg_value = 0, probe_avg_value = 0;
    DBG_VERB("Touch sensor ADC event triggered at cycle= %u", mcp_adc_ptr->timeout_cycles);

    if (mcp3462r_is_data_ready(mcp_adc_ptr)) {
        // Read ADC data
        mcp_adc_ptr->msg[0] = READ_CMD;
        mcp_adc_ptr->msg[1] = 0x00;
        mcp_adc_ptr->msg[2] = 0x00;
        spidev_transfer(mcp_adc_ptr->spi, 1, 3, mcp_adc_ptr->msg);

        data = (mcp_adc_ptr->msg[1] << 8) | mcp_adc_ptr->msg[2];
        // Push the raw data to the rolling average handler
        rolling_avg_push(&probe_avg_hndler, (float)data);
        
        // Process the raw values
        rolling_avg_value = (uint16_t)(rolling_avg_get_last(&rollingAvg_hndler));
        probe_avg_value = (uint16_t)(rolling_avg_get_last(&probe_avg_hndler));
        DBG_VERB("Probe: raw= %u, avg= %u BaseL= %u at cycle= %u sens= %u",
               data, probe_avg_value, rolling_avg_value, mcp_adc_ptr->timeout_cycles, mcp_adc_ptr->sensitivity);

        // sendf("Probing_read oid=%c raw=%u avg=%u", mcp_adc_ptr->oid,
        //     data, (uint16_t)(rolling_avg_get_last(&rollingAvg_hndler)));
        if (rolling_avg_value > probe_avg_value && 
            (rolling_avg_value - probe_avg_value) > mcp_adc_ptr->sensitivity) {
            // Touch detected
            DBG_INFO("Touch detected: raw data=%u, prove_avg=%u, rolling avg=%u, sensitivity=%u, cycle=%u",
                   data, probe_avg_value, rolling_avg_value, mcp_adc_ptr->sensitivity, mcp_adc_ptr->timeout_cycles);
            gpio_out_write(mcp_adc_ptr->trigger_out_pin, 1);
            mcp_adc_ptr->timeout_cycles = 1; // End session soon
            doneP = 1;
        } 
    }
    else {
        DEBUG_VERBOSE("ADC not ready at cycle=%u", mcp_adc_ptr->timeout_cycles);
    }

    mcp_adc_ptr->timer.waketime += mcp_adc_ptr->rest_ticks;

    if (--mcp_adc_ptr->timeout_cycles == 0) {
        // Timeout reached, stop the task
        sendf("Ts_session_result oid=%c status=%u lstValue=%u",
              mcp_adc_ptr->oid, doneP, probe_avg_value);
        DBG_INFO("Touch sensing session completed for OID=%c, status=%u, last value=%u",
               mcp_adc_ptr->oid, doneP, probe_avg_value);
        gpio_out_write(mcp_adc_ptr->PI_EN_pin, 0);

        if (doneP) {
            // Schedule terminator event to reset probe state
            mcp_adc_ptr->timer.waketime +=  500000;
            mcp_adc_ptr->timer.func = mcp3462r_terminator_event;
            return SF_RESCHEDULE;
        }
        mcp_adc_ptr->active_session_flag = 0;
        sched_del_timer(&mcp_adc_ptr->timer);
        return SF_DONE;
    }
    return SF_RESCHEDULE;
}

static uint_fast8_t periodic_read_event(struct timer *t) {
    uint16_t data = 0;

    static uint32_t last_output_time = 0;
    uint32_t current_time = timer_read_time();

    rollingAvg_hndler.timer.waketime = timer_read_time() + rollingAvg_hndler.rest_ticks;
    if (mcp_adc_ptr->active_session_flag || !mcp_adc_ptr->configured_flag) {
        DBG_WARN("Touch sensor ADC HW is not configured or session is active, pausing periodic read");
        rolling_avg_pause(&rollingAvg_hndler);
        return SF_DONE;
    }
    if (mcp3462r_is_data_ready(mcp_adc_ptr)) {
        // Read ADC data
        mcp_adc_ptr->msg[0] = READ_CMD;
        mcp_adc_ptr->msg[1] = 0x00;
        mcp_adc_ptr->msg[2] = 0x00;
        spidev_transfer(mcp_adc_ptr->spi, 1, 3, mcp_adc_ptr->msg);

        data = (mcp_adc_ptr->msg[1] << 8) | mcp_adc_ptr->msg[2];
        rolling_avg_push(&rollingAvg_hndler, (float)data);
        // sendf("Periodic_read oid=%c raw=%u avg=%u", mcp_adc_ptr->oid
        //       ,data, (uint16_t)(rolling_avg_get_last(&rollingAvg_hndler)));
        if (current_time - last_output_time > timer_from_us(1)) {
                last_output_time = current_time;
            DBG_VERB("Periodic: raw read= %u, rolling avg is= %u", data, (uint16_t)(rolling_avg_get_last(&rollingAvg_hndler)));
        }
    } 
    else {
        DBG_VERB("Periodic: ADC not ready");
    }
    return SF_RESCHEDULE;
}

// -----------------------------------------------------------------------------
// Command Handlers
// -----------------------------------------------------------------------------

// Configure the ADC hardware and state
void command_cfg_ts_adc(uint32_t *args) {
    mcp_adc_ptr = oid_alloc(args[0], command_cfg_ts_adc, sizeof(*mcp_adc_ptr));
    mcp_adc_ptr->oid = args[0];
    mcp_adc_ptr->spi = spidev_oid_lookup(args[1]);
    mcp_adc_ptr->adc_ready_pin = gpio_in_setup(args[2], !ADC_ACTIVE_STATE);
    mcp_adc_ptr->trigger_out_pin = gpio_out_setup(args[3], 0);
    mcp_adc_ptr->PI_EN_pin = gpio_out_setup(args[4], 0);
    mcp_adc_ptr->timer.func = mcp3462r_event;
    mcp_adc_ptr->active_session_flag = 0;
    mcp_adc_ptr->configured_flag = 1;
    mcp_adc_ptr->msg[0] = 0x00;
    mcp_adc_ptr->msg[1] = 0x00;
    gpio_out_write(mcp_adc_ptr->trigger_out_pin, 0);

    DBG_INFO("Touch sensor ADC configured with OID=%c, SPI OID=%c, ADC ready pin=%u, Trigger out pin=%u PI_EN pin=%u cycle_us=%u",
           mcp_adc_ptr->oid, args[1], args[2], args[3], args[4], args[5]);
    
    
    rolling_avg_init(&rollingAvg_hndler, BUFFER_SIZE, periodic_read_event, 
                     timer_from_us(args[5]));

    rolling_avg_resume(&rollingAvg_hndler);
}

DECL_COMMAND(command_cfg_ts_adc,
             "cfg_ts_adc oid=%c spi_oid=%c adc_int_pin=%u trigger_out_pin=%u PI_EN_pin=%u cycle_us=%u");

// Start a new touch sensing session
void command_start_touch_sensing_session(uint32_t *args) {
    if (mcp_adc_ptr == NULL || !mcp_adc_ptr->configured_flag) {
        shutdown("Touch sensor ADC HW is not configured or the session is already active");
    }
    if (mcp_adc_ptr->oid != args[0]) {
        shutdown("Touch sensor ADC OID does not match the configured OID");
    }

    mcp_adc_ptr->timeout_cycles = args[1];
    mcp_adc_ptr->rest_ticks = args[2];
    mcp_adc_ptr->sensitivity = args[3];
    gpio_out_write(mcp_adc_ptr->PI_EN_pin, 1);

    DBG_INFO("Starting touch sensing session with OID=%c, timeout_cycles=%u, rest_ticks=%u, sensitivity=%u",
           mcp_adc_ptr->oid, mcp_adc_ptr->timeout_cycles, mcp_adc_ptr->rest_ticks, mcp_adc_ptr->sensitivity);
    // TODO: Make the errors less aggressive
    if (!mcp_adc_ptr->timeout_cycles || !mcp_adc_ptr->rest_ticks) {
        shutdown("Timeout cycles and rest ticks must be greater than 0");
    }
    if (mcp_adc_ptr->sensitivity == 0) {
        shutdown("Sensitivity must be greater than 0");
    }
    if (mcp_adc_ptr->active_session_flag) {
        shutdown("Touch sensing session is already active");
    }

    mcp_adc_ptr->active_session_flag = 1;
    // Init a rolling average handler for the probe with the last value from the prioding read
    rolling_avg_init(&probe_avg_hndler, BUFFER_SIZE, NULL, 0);
    rolling_avg_push_multiple(&probe_avg_hndler, rollingAvg_hndler.buffer, rollingAvg_hndler.count);
    // Start the periodic event to listen for data ready
    sched_del_timer(&mcp_adc_ptr->timer);
    mcp_adc_ptr->timer.func = mcp3462r_event;

    irq_disable();
    mcp_adc_ptr->timer.waketime = timer_read_time() + mcp_adc_ptr->rest_ticks;
    sched_add_timer(&mcp_adc_ptr->timer);
    irq_enable();
}

DECL_COMMAND(command_start_touch_sensing_session,
             "start_ts_session oid=%c timeout_cycles=%u rest_ticks=%u sensitivity=%u");


void command_resume_rolling_avg(uint32_t *args) {
    if (mcp_adc_ptr == NULL || !mcp_adc_ptr->configured_flag) {
        shutdown("Touch sensor ADC HW is not configured");
    }
    if (rollingAvg_hndler.running_flag) {
        DBG_ERR("Rolling average is already running for OID=%c", mcp_adc_ptr->oid);
        return;
    }
    DBG_INFO("Resuming rolling average for OID=%c", mcp_adc_ptr->oid);
    rolling_avg_resume(&rollingAvg_hndler);
}
DECL_COMMAND(command_resume_rolling_avg,
             "resume_rolling_avg oid=%c");