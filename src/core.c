/* Lifecycle: claiming the controller, the RS pin, and giving both back.
 * The acceptance filter lives here too, because it is latched at install time
 * and so is part of starting up rather than of running.
 */

#include <string.h>

#include "internal.h"

/* ---- shared with io.c and health.c, declared in internal.h ---- */

static SemaphoreHandle_t can_mutex;
static portMUX_TYPE can_mutex_init = portMUX_INITIALIZER_UNLOCKED;

static SemaphoreHandle_t can_mutex_get(void)
{
    static StaticSemaphore_t buffer;

    if (!can_mutex) {
        portENTER_CRITICAL(&can_mutex_init);

        if (!can_mutex)
            can_mutex = xSemaphoreCreateRecursiveMutexStatic(&buffer);

        portEXIT_CRITICAL(&can_mutex_init);
    }

    return can_mutex;
}

/* The RAII guard the C++ version used is gone, so every locked function has
 * one exit label and releases there. Never return from inside the lock.
 */
void can_lock(void)
{
    xSemaphoreTakeRecursive(can_mutex_get(), portMAX_DELAY);
}

void can_unlock(void)
{
    xSemaphoreGiveRecursive(can_mutex);
}

/* One bit per TWAI controller currently installed. See internal.h. */
uint32_t can_claimed_mask;

/* Argument validation. Every public call screens its inputs here before it
 * reaches the driver, so a bad pin, bitrate, identifier or payload is turned
 * away as CAN_EINVAL instead of being handed to the hardware.
 */

bool is_brate_valid(uint32_t bitrate)
{
    switch (bitrate) {
        case CAN_BITRATE_25K:
        case CAN_BITRATE_50K:
        case CAN_BITRATE_100K:
        case CAN_BITRATE_125K:
        case CAN_BITRATE_250K:
        case CAN_BITRATE_500K:
        case CAN_BITRATE_800K:
        case CAN_BITRATE_1M:
            return true;
        default:
            return false;
    }
}

bool is_pin_valid(int tx_pin, int rx_pin, int rs_pin)
{
    if (tx_pin < 0 || rx_pin < 0 || tx_pin == rx_pin || rs_pin < CAN_NO_PIN)
        return false;

    /* An RS pin is optional, but it cannot double as the TX or RX pin */
    return rs_pin == CAN_NO_PIN || (rs_pin != tx_pin && rs_pin != rx_pin);
}

bool is_id_valid(uint32_t id, bool extended)
{
    return extended ? id <= CAN_EXT_ID_MAX : id <= CAN_STD_ID_MAX;
}

bool is_msg_valid(const uint8_t *data, uint8_t len)
{
    return len <= CAN_MAX_DLC && (len == 0 || data != NULL);
}

bool brate_to_timing(uint32_t bitrate, twai_timing_config_t *timing)
{
    switch (bitrate) {
        case CAN_BITRATE_25K:
            *timing = TWAI_TIMING_CONFIG_25KBITS();
            return true;
        case CAN_BITRATE_50K:
            *timing = TWAI_TIMING_CONFIG_50KBITS();
            return true;
        case CAN_BITRATE_100K:
            *timing = TWAI_TIMING_CONFIG_100KBITS();
            return true;
        case CAN_BITRATE_125K:
            *timing = TWAI_TIMING_CONFIG_125KBITS();
            return true;
        case CAN_BITRATE_250K:
            *timing = TWAI_TIMING_CONFIG_250KBITS();
            return true;
        case CAN_BITRATE_500K:
            *timing = TWAI_TIMING_CONFIG_500KBITS();
            return true;
        case CAN_BITRATE_800K:
            *timing = TWAI_TIMING_CONFIG_800KBITS();
            return true;
        case CAN_BITRATE_1M:
            *timing = TWAI_TIMING_CONFIG_1MBITS();
            return true;
        default:
            return false;
    }
}

can_state can_to_state(twai_state_t state)
{
    switch (state) {
        case TWAI_STATE_RUNNING:
            return CAN_STATE_RUNNING;
        case TWAI_STATE_BUS_OFF:
            return CAN_STATE_BUS_OFF;
        case TWAI_STATE_RECOVERING:
            return CAN_STATE_RECOVERING;
        default:
            return CAN_STATE_STOPPED;
    }
}

/* pdMS_TO_TICKS() floors, so any timeout below one tick period would become a
 * non-blocking call. Only a t_ms of 0 is allowed to mean do not block.
 */
TickType_t ms_to_ticks(uint32_t t_ms)
{
    TickType_t ticks = pdMS_TO_TICKS(t_ms);

    if (t_ms > 0 && ticks == 0)
        ticks = MIN_BLOCKING_TICKS;

    return ticks;
}

void hvd230_init(hvd230_t *can)
{
    if (!can)
        return;

    memset(can, 0, sizeof(*can));

    can->ctrl_id = 0;
    can->rs_pin = CAN_NO_PIN;
    can->rx_queue = CAN_DEFAULT_RX_QUEUE;
}

/* The acceptance filter registers are not a plain (id, mask) pair.
 *
 * The identifier sits at the top of a 32-bit word: bit 31 down to bit 21 for
 * a standard frame, down to bit 3 for an extended one. The mask register is
 * inverted, where a 1 bit means "do not care" -- so an all-ones mask accepts
 * everything, which is what TWAI_FILTER_CONFIG_ACCEPT_ALL() sets.
 *
 * Getting this wrong does not fail loudly; it silently accepts nothing.
 */
static uint8_t filter_shift(bool extended)
{
    return extended ? FILTER_SHIFT_EXT : FILTER_SHIFT_STD;
}

can_sta hvd230_set_controller(hvd230_t *can, int id)
{
    can_sta sta = CAN_OK;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (can->is_installed)
        sta = CAN_EDRIVER;          /* the id is read at install time */
    else if (id < 0 || id >= SOC_TWAI_CONTROLLER_NUM || (id > 0 && !CAN_MULTI_CTRL))
        sta = CAN_EINVAL;
    else
        can->ctrl_id = id;

    can_unlock();
    return sta;
}

can_sta hvd230_begin_ex(hvd230_t *can, int tx_pin, int rx_pin, uint32_t bitrate,
                        int rs_pin, uint32_t tx_queue, uint32_t rx_queue)
{
    can_sta sta = CAN_OK;
    twai_timing_config_t timing;
    twai_general_config_t general;
    twai_filter_config_t filter = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (can->is_installed ||
        (can_claimed_mask & (1u << can->ctrl_id))) {
        sta = CAN_EDRIVER;      /* this handle, or this controller, is live */
        goto out;
    }

    if (!is_pin_valid(tx_pin, rx_pin, rs_pin) || !is_brate_valid(bitrate)) {
        sta = CAN_EINVAL;
        goto out;
    }

    if (tx_queue == 0 || rx_queue == 0) {
        sta = CAN_EINVAL;
        goto out;
    }

    if (!brate_to_timing(bitrate, &timing)) {
        sta = CAN_EINVAL;
        goto out;
    }

    {
        twai_general_config_t g = TWAI_GENERAL_CONFIG_DEFAULT(
            (gpio_num_t)tx_pin, (gpio_num_t)rx_pin, TWAI_MODE_NORMAL);
        general = g;
    }
    general.tx_queue_len = tx_queue;
    general.rx_queue_len = rx_queue;
    can->rx_queue = rx_queue;

    if (can->is_filter_set) {
        const uint8_t shift = filter_shift(can->is_filter_ext);

        filter.acceptance_code = can->filter_id << shift;
        filter.acceptance_mask = ~(can->filter_mask << shift);
        filter.single_filter = true;
    }

    if (can_hw_install(&general, &timing, &filter, can->ctrl_id,
                       &can->handle) != ESP_OK) {
        sta = CAN_EDRIVER;
        goto out;
    }

    can->is_installed = true;
    can_claimed_mask |= 1u << can->ctrl_id;

    /* The RS pin is claimed only after the driver is installed, so that every
     * failure below can hand it back through hvd230_end(). Driving it before
     * the controller starts keeps a half-configured node off the bus.
     */
    if (rs_pin >= 0) {
        can->rs_pin = rs_pin;

        /* Preload the standby level so enabling the output cannot glitch. */
        if (gpio_set_level((gpio_num_t)rs_pin, RS_LEVEL_STANDBY) != ESP_OK) {
            hvd230_end(can);
            sta = CAN_EDRIVER;
            goto out;
        }

        can->is_standby = true;

        if (gpio_set_direction((gpio_num_t)rs_pin, GPIO_MODE_OUTPUT) != ESP_OK) {
            hvd230_end(can);
            sta = CAN_EDRIVER;
            goto out;
        }
    }

    if (can_hw_start(can) != ESP_OK) {
        hvd230_end(can);
        sta = CAN_EDRIVER;
        goto out;
    }

    can->is_started = true;

    if (can->rs_pin >= 0) {
        if (gpio_set_level((gpio_num_t)can->rs_pin, RS_LEVEL_ACTIVE) != ESP_OK) {
            hvd230_end(can);
            sta = CAN_EDRIVER;
            goto out;
        }
    }

    can->is_standby = false;

out:
    can_unlock();
    return sta;
}

can_sta hvd230_begin(hvd230_t *can, int tx_pin, int rx_pin, uint32_t bitrate)
{
    return hvd230_begin_ex(can, tx_pin, rx_pin, bitrate, CAN_NO_PIN,
                           CAN_DEFAULT_TX_QUEUE, CAN_DEFAULT_RX_QUEUE);
}

can_sta hvd230_standby(hvd230_t *can)
{
    can_sta sta = CAN_OK;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (!can->is_started) {
        sta = CAN_ENOTSTARTED;
        goto out;
    }

    if (can->rs_pin < 0) {
        sta = CAN_ENOPIN;
        goto out;
    }

    if (!can->is_standby) {
        if (gpio_set_level((gpio_num_t)can->rs_pin, RS_LEVEL_STANDBY) != ESP_OK) {
            sta = CAN_EDRIVER;
            goto out;
        }

        can->is_standby = true;
    }

out:
    can_unlock();
    return sta;
}

can_sta hvd230_wake(hvd230_t *can)
{
    can_sta sta = CAN_OK;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (!can->is_started) {
        sta = CAN_ENOTSTARTED;
        goto out;
    }

    if (can->rs_pin < 0) {
        sta = CAN_ENOPIN;
        goto out;
    }

    if (can->is_standby) {
        if (gpio_set_level((gpio_num_t)can->rs_pin, RS_LEVEL_ACTIVE) != ESP_OK) {
            sta = CAN_EDRIVER;
            goto out;
        }

        can->is_standby = false;
    }

out:
    can_unlock();
    return sta;
}

can_sta hvd230_set_filter(hvd230_t *can, uint32_t id, uint32_t mask, bool extended)
{
    can_sta sta = CAN_OK;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    /* The driver reads the filter once, at install time */
    if (can->is_installed) {
        sta = CAN_EDRIVER;
        goto out;
    }

    if (!is_id_valid(id, extended) || !is_id_valid(mask, extended)) {
        sta = CAN_EINVAL;
        goto out;
    }

    /* Bits set in id but not in mask can never match, so the filter would
     * accept nothing. That is always a mistake, not an intent -- and its
     * failure mode is silence, which is expensive to debug on a bench.
     */
    if ((id & mask) != id) {
        sta = CAN_EINVAL;
        goto out;
    }

    can->filter_id = id;
    can->filter_mask = mask;
    can->is_filter_ext = extended;
    can->is_filter_set = true;

out:
    can_unlock();
    return sta;
}

can_sta hvd230_clear_filter(hvd230_t *can)
{
    can_sta sta = CAN_OK;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (can->is_installed)
        sta = CAN_EDRIVER;
    else
        can->is_filter_set = false;

    can_unlock();
    return sta;
}

can_sta hvd230_end(hvd230_t *can)
{
    can_sta sta = CAN_OK;
    twai_status_info_t info;
    bool known;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (!can->is_installed)
        goto out;

    if (can->rs_pin >= 0) {
        /* A transceiver we cannot park is worth reporting, but not worth
         * stranding the controller over: returning here would leak the
         * driver for the rest of the program.
         */
        if (gpio_set_level((gpio_num_t)can->rs_pin, RS_LEVEL_STANDBY) != ESP_OK)
            sta = CAN_EDRIVER;
        else
            can->is_standby = true;
    }

    /* Ask the controller what state it is in rather than trusting started:
     * a bus-off recovery may have stopped it behind our back. Only a running
     * controller can be stopped, and only a stopped one can be uninstalled.
     */
    memset(&info, 0, sizeof(info));
    known = can_hw_status(can, &info) == ESP_OK;

    if (known && info.state == TWAI_STATE_RUNNING) {
        if (can_hw_stop(can) != ESP_OK) {
            sta = CAN_EDRIVER;
            goto out;
        }
    }

    can->is_started = false;

    if (can_hw_uninstall(can) != ESP_OK) {
        sta = CAN_EDRIVER;
        goto out;
    }

    can->handle = NULL;
    can->rs_pin = CAN_NO_PIN;
    can->is_installed = false;
    can->is_standby = false;
    can->is_recovering = false;
    can->is_bus_off_seen = false;
    can_claimed_mask &= ~(1u << can->ctrl_id);

out:
    can_unlock();
    return sta;
}
