/* Route frames to handlers instead of switching on the identifier.
 *
 * Handlers run inside hvd230_dispatch(), on this thread -- no interrupt
 * context, so they may do anything, including sending. With fault
 * protection added, see callbacks_health.c.
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
#define POLL_MS     10

/* Bitwise: matches when (id & mask) == id. This pair takes 0x200..0x20F. */
#define NODE_ID   0x200
#define NODE_MASK 0x7F0

static const char *TAG = "callbacks";

/* Reaches the handlers through ctx, so they need no globals. */
typedef struct {
    uint32_t node_frames;
    uint32_t other_frames;
    int16_t last_value;
} app_state_t;

static void on_node_frame(const can_frame_t *frame, void *ctx)
{
    app_state_t *state = ctx;

    state->node_frames++;

    /* Bytes 2 and 3 are a big-endian int16 reading. */
    if (frame->len >= 4)
        state->last_value = (int16_t)((frame->data[2] << 8) | frame->data[3]);
}

/* Mask 0 matches everything, so this runs as well as the handler above. */
static void on_any_frame(const can_frame_t *frame, void *ctx)
{
    app_state_t *state = ctx;

    if ((frame->id & NODE_MASK) != NODE_ID)
        state->other_frames++;
}

void app_main(void)
{
    static app_state_t state;
    uint32_t since_report_ms = 0;
    hvd230_t can;

    hvd230_init(&can);

    if (hvd230_begin(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    if (hvd230_on_frame(&can, NODE_ID, NODE_MASK, on_node_frame, &state) !=
            CAN_OK ||
        hvd230_on_frame(&can, 0, 0, on_any_frame, &state) != CAN_OK) {
        ESP_LOGE(TAG, "could not register callbacks");
        hvd230_end(&can);
        return;
    }

    while (true) {
        /* Drains the queue into the handlers. Do not also call hvd230_rcv()
         * -- one queue, first call wins.
         */
        hvd230_dispatch(&can, 0);

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        since_report_ms += POLL_MS;
        if (since_report_ms < 1000)
            continue;

        since_report_ms = 0;

        ESP_LOGI(TAG, "matched=%" PRIu32 " other=%" PRIu32 " last value=%d",
                 state.node_frames, state.other_frames, (int)state.last_value);
    }
}
