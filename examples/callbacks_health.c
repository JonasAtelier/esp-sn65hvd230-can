/* Frame callbacks plus fault protection -- what a node on a real bus wants.
 *
 * callbacks.c is the routing alone. This adds the queue depth, bus-off
 * recovery and error counters that keep the node alive. Copy this one.
 *
 * Not a buildable ESP-IDF example project, just the call sequence.
 */

#include <inttypes.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hvd230can.h"

#define CAN_TX_PIN  5
#define CAN_RX_PIN  4
#define CAN_BITRATE CAN_BITRATE_1M

/* Frames. The default 5 is easily overrun; watch rx_miss_cnt. */
#define TX_QUEUE 8
#define RX_QUEUE 32

/* How long to stay off the bus after bus-off before rejoining. */
#define RESTART_MS 100

#define POLL_MS         10
#define HEALTH_EVERY_MS 1000

#define NODE_ID   0x200
#define NODE_MASK 0x7F0

static const char *TAG = "cb_health";

typedef struct {
    uint32_t node_frames;
    int16_t last_value;
} app_state_t;

static void on_node_frame(const can_frame_t *frame, void *ctx)
{
    app_state_t *state = ctx;

    state->node_frames++;

    if (frame->len >= 4)
        state->last_value = (int16_t)((frame->data[2] << 8) | frame->data[3]);
}

void app_main(void)
{
    static app_state_t state;
    uint32_t since_health_ms = 0;
    uint32_t frames_last_report = 0;
    hvd230_t can;

    hvd230_init(&can);

    /* Drop the rest in hardware, before it costs an interrupt. Before begin. */
    if (hvd230_set_filter(&can, NODE_ID, NODE_MASK, false) != CAN_OK) {
        ESP_LOGE(TAG, "bad acceptance filter");
        return;
    }

    if (hvd230_begin_ex(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE, CAN_NO_PIN,
                        TX_QUEUE, RX_QUEUE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    /* Without this the node never rejoins after bus-off. */
    hvd230_set_auto_recover(&can, RESTART_MS);

    if (hvd230_on_frame(&can, NODE_ID, NODE_MASK, on_node_frame, &state) !=
        CAN_OK) {
        ESP_LOGE(TAG, "could not register callback");
        hvd230_end(&can);
        return;
    }

    while (true) {
        can_health_t h;
        uint32_t frames_this_report;

        /* One recovery step. Dispatch alone would never recover. */
        hvd230_service(&can);

        /* Drains into the handler. Bounded, so bus-off cannot spin here. */
        hvd230_dispatch(&can, 0);

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        since_health_ms += POLL_MS;
        if (since_health_ms < HEALTH_EVERY_MS)
            continue;

        since_health_ms = 0;

        if (hvd230_health(&can, &h) != CAN_OK)
            continue;

        frames_this_report = state.node_frames - frames_last_report;
        frames_last_report = state.node_frames;

        ESP_LOGI(TAG,
                 "state=%d frames/s=%" PRIu32 " value=%d tec=%" PRIu32
                 " missed=%" PRIu32 " arb_lost=%" PRIu32,
                 (int)h.state, frames_this_report, (int)state.last_value,
                 h.tx_err_cnt, h.rx_miss_cnt, h.arb_lost_cnt);

        /* Frames arriving AND errors climbing: alive but degrading. */
        if (frames_this_report > 0 && h.tx_err_cnt > 96)
            ESP_LOGW(TAG, "bus alive but degrading, bus-off is close");

        /* Fewer handled than arrived: raise RX_QUEUE or shorten POLL_MS. */
        if (h.rx_miss_cnt > 0)
            ESP_LOGW(TAG, "rx queue overran, callbacks missed frames");

        if (h.state == CAN_STATE_BUS_OFF)
            ESP_LOGE(TAG, "bus-off, rejoining in %d ms", RESTART_MS);
        else if (frames_this_report == 0)
            ESP_LOGW(TAG, "bus healthy but silent, is anything transmitting?");
    }
}
