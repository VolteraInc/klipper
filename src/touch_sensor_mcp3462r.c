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

#define ADC_ACTIVE_STATE 0

// Global pointer to the ADC instance
static struct mcp3462r_adc *mcp_adc_ptr = NULL;
static struct RollingAverage rollingAvg_hndler;

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
        ra->buffer[i] = 0.0f; // Initialize the buffer to zero
    }
}

float rolling_avg_push(struct RollingAverage *ra, float value) {
    if (ra->count < ra->size) {
        ra->count++;
    } else {
        ra->sum -= ra->buffer[ra->index];  
    }

    ra->buffer[ra->index] = value;
    ra->sum += value;

    ra->index = (ra->index + 1) % ra->size;
    ra->lst_avg_value = ra->sum / ra->count;
    return ra->lst_avg_value;
}

float rolling_avg_get_last(struct RollingAverage *ra) {
    return ra->lst_avg_value;
}

void rolling_avg_reset(struct RollingAverage *ra) {
    ra->size = 0;
    ra->index = 0;
    ra->count = 0;
    ra->sum = 0;
    ra->lst_avg_value = 0.0f;
    for (int i = 0; i < BUFFER_SIZE; i++) 
        ra->buffer[i] = 0.0f;
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
    output("Touch sensor ADC terminator event triggered at cycle= %u", mcp_adc_ptr->timeout_cycles);
    output("Touch sensor ADC session terminated at cycle= %u, session flag is= %u",
           mcp_adc_ptr->timeout_cycles, mcp_adc_ptr->active_session_flag);
    return SF_DONE;
}

// Main event: handles ADC data polling and session logic
static uint_fast8_t mcp3462r_event(struct timer *t) {
    uint16_t doneP = 0, data = 0;
    uint16_t rolling_avg_value = 0;
    // output("Touch sensor ADC event triggered at cycle= %u", mcp_adc_ptr->timeout_cycles);

    if (mcp3462r_is_data_ready(mcp_adc_ptr)) {
        // Read ADC data
        mcp_adc_ptr->msg[0] = READ_CMD;
        mcp_adc_ptr->msg[1] = 0x00;
        mcp_adc_ptr->msg[2] = 0x00;
        spidev_transfer(mcp_adc_ptr->spi, 1, 3, mcp_adc_ptr->msg);

        data = (mcp_adc_ptr->msg[1] << 8) | mcp_adc_ptr->msg[2];
        // Process the raw values
        // output("Got new raw ADC data: %u at cycle= %u sensitivity is %u",
        //         data, mcp_adc_ptr->timeout_cycles, mcp_adc_ptr->sensitivity);
        rolling_avg_value = (uint16_t)(rolling_avg_get_last(&rollingAvg_hndler));
        if (rolling_avg_value - data > mcp_adc_ptr->sensitivity) {
            // Touch detected
            output("Touch detected: raw data=%u, rolling avg=%u, sensitivity=%u",
                   data, rolling_avg_value, mcp_adc_ptr->sensitivity);
            gpio_out_write(mcp_adc_ptr->trigger_out_pin, 1);
            mcp_adc_ptr->timeout_cycles = 1; // End session soon
            doneP = 1;
        } else {
            // No touch detected, reset output pin
            gpio_out_write(mcp_adc_ptr->trigger_out_pin, 0);
        }
    }

    mcp_adc_ptr->timer.waketime += mcp_adc_ptr->rest_ticks;

    if (--mcp_adc_ptr->timeout_cycles == 0) {
        // Timeout reached, stop the task
        mcp_adc_ptr->active_session_flag = 0;
        sendf("Ts_session_result oid=%c status=%u lstValue=%u",
              mcp_adc_ptr->oid, doneP, data);
        output("Ts_session_result status=%u lstValue=%u", doneP, data);
        gpio_out_write(mcp_adc_ptr->PI_EN_pin, 0);

        if (doneP) {
            // Schedule terminator event to reset probe state
            mcp_adc_ptr->timer.waketime +=  5 * mcp_adc_ptr->rest_ticks;
            mcp_adc_ptr->timer.func = mcp3462r_terminator_event;
            return SF_RESCHEDULE;
        }
        sched_del_timer(&mcp_adc_ptr->timer);
        return SF_DONE;
    }
    return SF_RESCHEDULE;
}

static uint_fast8_t periodic_read_event(struct timer *t) {
    uint16_t data = 0;
    rollingAvg_hndler.timer.waketime = timer_read_time() + rollingAvg_hndler.rest_ticks;
    if (mcp_adc_ptr->active_session_flag || !mcp_adc_ptr->configured_flag) {
        output("Touch sensor ADC HW is not configured or session is active");
        return SF_RESCHEDULE;
    }
    if (mcp3462r_is_data_ready(mcp_adc_ptr)) {
        // Read ADC data
        mcp_adc_ptr->msg[0] = READ_CMD;
        mcp_adc_ptr->msg[1] = 0x00;
        mcp_adc_ptr->msg[2] = 0x00;
        spidev_transfer(mcp_adc_ptr->spi, 1, 3, mcp_adc_ptr->msg);

        data = (mcp_adc_ptr->msg[1] << 8) | mcp_adc_ptr->msg[2];
        rolling_avg_push(&rollingAvg_hndler, (float)data);
        output("Periodic read: Got new raw ADC data: %u, rolling avg is: %u", data, (uint16_t)(rolling_avg_get_last(&rollingAvg_hndler)));
    } else {
        output("Periodic read: No data ready from ADC");
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

    output("Touch sensor ADC configured with OID=%c, SPI OID=%c, ADC ready pin=%u, Trigger out pin=%u PI_EN pin=%u cycle_us=%u",
           mcp_adc_ptr->oid, args[1], args[2], args[3], args[4], args[5]);
    
    
    rolling_avg_init(&rollingAvg_hndler, BUFFER_SIZE, periodic_read_event, 
                     timer_from_us(args[5]));

    irq_disable();
    rollingAvg_hndler.timer.waketime = timer_read_time() + rollingAvg_hndler.rest_ticks;
    sched_add_timer(&rollingAvg_hndler.timer);
    irq_enable();

    output("Rolling average handler initialized with");

}

DECL_COMMAND(command_cfg_ts_adc,
             "cfg_ts_adc oid=%c spi_oid=%c adc_int_pin=%u trigger_out_pin=%u PI_EN_pin=%u cycle_us=%u");

// Start a new touch sensing session
void command_start_touch_sensing_session(uint32_t *args) {
    if (mcp_adc_ptr == NULL || !mcp_adc_ptr->configured_flag) {
        shutdown("Touch sensor ADC HW is not configured");
    }
    if (mcp_adc_ptr->oid != args[0]) {
        shutdown("Touch sensor ADC OID does not match the configured OID");
    }

    mcp_adc_ptr->timeout_cycles = args[1];
    mcp_adc_ptr->rest_ticks = args[2];
    mcp_adc_ptr->sensitivity = args[3];
    gpio_out_write(mcp_adc_ptr->PI_EN_pin, 1);

    output("Starting touch sensing session with OID=%c, timeout_cycles=%u, rest_ticks=%u, sensitivity=%u",
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