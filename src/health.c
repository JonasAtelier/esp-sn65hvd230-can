/* Error counters and the bus-off recovery walk. */

#include <string.h>

#include "internal.h"

can_sta hvd230_health(const hvd230_t *can, can_health_t *out)
{
    can_sta sta = CAN_OK;
    twai_status_info_t info;

    if (!can || !out)
        return CAN_EINVAL;

    can_lock();

    if (!can->is_installed) {
        sta = CAN_ENOTSTARTED;
        goto out_unlock;
    }

    memset(&info, 0, sizeof(info));

    if (can_hw_status(can, &info) != ESP_OK) {
        sta = CAN_EDRIVER;
        goto out_unlock;
    }

    out->state = can_to_state(info.state);
    out->tx_err_cnt = info.tx_error_counter;
    out->rx_err_cnt = info.rx_error_counter;
    out->tx_fail_cnt = info.tx_failed_count;
    out->rx_miss_cnt = info.rx_missed_count;
    out->arb_lost_cnt = info.arb_lost_count;
    out->bus_err_cnt = info.bus_error_count;
    out->tx_pending = info.msgs_to_tx;
    out->rx_pending = info.msgs_to_rx;

out_unlock:
    can_unlock();
    return sta;
}

void hvd230_set_auto_recover(hvd230_t *can, uint32_t restart_ms)
{
    if (!can)
        return;

    can_lock();
    can->restart_ms = restart_ms;
    can_unlock();
}

/* The bus-off recovery sequence, driven one step per call.
 *
 * A node that hits 256 transmit errors leaves the bus and never returns on
 * its own. Recovery is: ask the controller to recover, wait for it to see
 * 128 idle sequences, then start it again. Each of those is a separate state,
 * so this walks one edge per call and never blocks.
 */
can_sta hvd230_service(hvd230_t *can)
{
    can_sta sta = CAN_OK;
    twai_status_info_t info;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (!can->is_installed) {
        sta = CAN_ENOTSTARTED;
        goto out;
    }

    memset(&info, 0, sizeof(info));

    if (can_hw_status(can, &info) != ESP_OK) {
        sta = CAN_EDRIVER;
        goto out;
    }

    switch (info.state) {
        case TWAI_STATE_BUS_OFF:
            sta = CAN_EDRIVER;

            if (can->restart_ms == CAN_NO_AUTO_RECOVER)
                goto out;

            /* Hold the node off the bus for restart_ms first, so a hard
             * fault is not answered with a tight recovery loop.
             */
            if (!can->is_bus_off_seen) {
                can->bus_off_tick = xTaskGetTickCount();
                can->is_bus_off_seen = true;
                goto out;
            }

            if ((xTaskGetTickCount() - can->bus_off_tick) <
                pdMS_TO_TICKS(can->restart_ms))
                goto out;

            if (can_hw_recover(can) != ESP_OK)
                goto out;

            can->is_recovering = true;
            can->is_started = false;
            goto out;

        case TWAI_STATE_RECOVERING:
            sta = CAN_EDRIVER;
            goto out;

        case TWAI_STATE_STOPPED:
            /* Only restart a controller that got here by recovering; a plain
             * stopped controller was stopped on purpose.
             */
            if (!can->is_recovering) {
                sta = CAN_ENOTSTARTED;
                goto out;
            }

            if (can_hw_start(can) != ESP_OK) {
                sta = CAN_EDRIVER;
                goto out;
            }

            can->is_started = true;
            can->is_recovering = false;
            can->is_bus_off_seen = false;
            goto out;

        default:
            can->is_bus_off_seen = false;
            can->is_recovering = false;
            goto out;
    }

out:
    can_unlock();
    return sta;
}
