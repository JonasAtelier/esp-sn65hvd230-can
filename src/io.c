/* Frames in, frames out, and the callback table that routes them.
 *
 * send and receive read the state under the lock and then drop it: the TWAI
 * calls block for timeout_ms, and holding the lock across that would stall
 * every other call for just as long.
 */

#include <string.h>

#include "internal.h"

can_sta hvd230_tx(hvd230_t *can, uint32_t id, const uint8_t *data,
                  uint8_t len, bool extended, uint32_t timeout_ms)
{
    bool started;
    bool standby;
    twai_message_t msg;
    esp_err_t res;

    if (!can)
        return CAN_EINVAL;

    /* Read the state under the lock, then let it go: twai_transmit_v2() may
     * block for timeout_ms, and holding the lock across that would stall
     * every other call for just as long.
     */
    can_lock();
    started = can->is_started;
    standby = can->is_standby;
    can_unlock();

    if (!started)
        return CAN_ENOTSTARTED;

    if (standby)
        return CAN_ESTANDBY;

    if (!is_id_valid(id, extended) || !is_msg_valid(data, len))
        return CAN_EINVAL;

    memset(&msg, 0, sizeof(msg));
    msg.identifier = id;
    msg.data_length_code = len;
    msg.flags = extended ? TWAI_MSG_FLAG_EXTD : 0;

    if (len > 0)
        memcpy(msg.data, data, len);

    res = can_hw_tx(can, &msg, ms_to_ticks(timeout_ms));

    if (res == ESP_OK)
        return CAN_OK;

    /* A full queue and an expired wait are the same driver error; only the
     * caller's timeout tells them apart.
     */
    if (res == ESP_ERR_TIMEOUT)
        return (timeout_ms == 0) ? CAN_EQFULL : CAN_ETIMEOUT;

    if (res == ESP_ERR_INVALID_ARG)
        return CAN_EINVAL;

    return CAN_EDRIVER;
}

can_sta hvd230_tx_frame(hvd230_t *can, const can_frame_t *frame,
                        uint32_t timeout_ms)
{
    if (!frame)
        return CAN_EINVAL;

    return hvd230_tx(can, frame->id, frame->data, frame->len,
                     frame->extended, timeout_ms);
}

can_sta hvd230_rcv(hvd230_t *can, can_frame_t *frame, uint32_t timeout_ms)
{
    bool started;
    twai_message_t msg;
    esp_err_t res;

    if (!can || !frame)
        return CAN_EINVAL;

    /* Same reasoning as hvd230_tx(): twai_receive_v2() blocks, the lock must
     * not be held across it.
     */
    can_lock();
    started = can->is_started;
    can_unlock();

    if (!started)
        return CAN_ENOTSTARTED;

    memset(&msg, 0, sizeof(msg));

    res = can_hw_rx(can, &msg, ms_to_ticks(timeout_ms));

    if (res == ESP_ERR_TIMEOUT)
        return CAN_ETIMEOUT;

    if (res != ESP_OK)
        return CAN_EDRIVER;

    /* Remote frames carry no payload, so handing one up as a data frame would
     * mean handing up stale bytes. Classic CAN caps the payload at CAN_MAX_DLC.
     */
    if ((msg.flags & TWAI_MSG_FLAG_RTR) != 0 || msg.data_length_code > CAN_MAX_DLC)
        return CAN_EDRIVER;

    memset(frame, 0, sizeof(*frame));
    frame->id = msg.identifier;
    frame->len = (uint8_t)msg.data_length_code;
    frame->extended = (msg.flags & TWAI_MSG_FLAG_EXTD) != 0;

    if (frame->len > 0)
        memcpy(frame->data, msg.data, frame->len);

    return CAN_OK;
}

can_sta hvd230_flush_tx(hvd230_t *can)
{
    can_sta sta = CAN_OK;

    if (!can)
        return CAN_EINVAL;

    can_lock();

    if (!can->is_started)
        sta = CAN_ENOTSTARTED;
    else if (can_hw_flush(can) != ESP_OK)
        sta = CAN_EDRIVER;

    can_unlock();
    return sta;
}

can_sta hvd230_on_frame(hvd230_t *can, uint32_t id, uint32_t mask,
                        can_rx_cb_t cb, void *ctx)
{
    can_sta sta = CAN_OK;

    if (!can || !cb)
        return CAN_EINVAL;

    can_lock();

    if (can->sub_cnt >= CAN_MAX_CALLBACKS) {
        sta = CAN_ENOSLOT;
        goto out;
    }

    can->subs[can->sub_cnt].id = id;
    can->subs[can->sub_cnt].msk = mask;
    can->subs[can->sub_cnt].cb = cb;
    can->subs[can->sub_cnt].ctx = ctx;
    can->sub_cnt++;

out:
    can_unlock();
    return sta;
}

void hvd230_clear_callbacks(hvd230_t *can)
{
    if (!can)
        return;

    can_lock();
    can->sub_cnt = 0;
    can_unlock();
}

/* One queue's worth of frames per call, handed to every callback that
 * matches. The drain is bounded because hvd230_rcv() answers CAN_EDRIVER
 * on every call while the controller is bus-off, and an unbounded loop would
 * spin there instead of letting the caller run hvd230_service().
 */
can_sta hvd230_dispatch(hvd230_t *can, uint32_t timeout_ms)
{
    hvd230_sub_t subs[CAN_MAX_CALLBACKS];
    uint8_t count;
    uint8_t s;
    uint32_t budget;
    uint32_t i;
    bool started;
    can_sta sta = CAN_OK;
    can_frame_t frame;

    if (!can)
        return CAN_EINVAL;

    /* Copy the table out under the lock so the handlers below run without it
     * held. A handler may then take as long as it likes, and may call back
     * into this object, without stalling another task.
     */
    can_lock();
    started = can->is_started;
    budget = can->rx_queue;
    count = can->sub_cnt;

    for (s = 0; s < count; s++)
        subs[s] = can->subs[s];

    can_unlock();

    if (!started)
        return CAN_ENOTSTARTED;

    for (i = 0; i < budget; i++) {
        const can_sta res = hvd230_rcv(can, &frame, timeout_ms);

        if (res == CAN_ETIMEOUT)
            break;

        /* A dropped frame is reported once at the end; the frames queued
         * behind it still deserve their callbacks.
         */
        if (res != CAN_OK) {
            sta = res;
            continue;
        }

        for (s = 0; s < count; s++) {
            if ((frame.id & subs[s].msk) == subs[s].id)
                subs[s].cb(&frame, subs[s].ctx);
        }
    }

    return sta;
}
