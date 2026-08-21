/* Run the bus with fault protection: a queue sized for the traffic,
 * bus-off recovery, and error counters watched often enough to matter.
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

/* Frames, not bytes. Size against the worst-case burst; the default 5 is
 * easily overrun. Overruns show up as rx_miss_cnt.
 */
#define TX_QUEUE 8
#define RX_QUEUE 32

/* How long to stay off the bus before rejoining. */
#define RESTART_MS 100

#define POLL_MS         10
#define HEALTH_EVERY_MS 1000

static const char *TAG = "protected";

void app_main(void)
{
    uint32_t since_health_ms = 0;
    hvd230_t can;

    hvd230_init(&can);

    /* hvd230_begin_ex() is the full form; hvd230_begin() is the same call
     * with no RS pin and the default queue depths.
     */
    if (hvd230_begin_ex(&can, CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE, CAN_NO_PIN,
                        TX_QUEUE, RX_QUEUE) != CAN_OK) {
        ESP_LOGE(TAG, "CAN startup failed");
        return;
    }

    /* Without this the node never rejoins the bus after bus-off. It is off
     * by default because a bus that keeps failing is usually a wiring fault
     * worth seeing rather than retrying around.
     */
    hvd230_set_auto_recover(&can, RESTART_MS);

    while (true) {
        can_frame_t frame;
        can_health_t h;
        uint32_t i;

        /* One recovery step per call; nothing while the bus is healthy. */
        hvd230_service(&can);

        /* Bounded: while bus-off, rcv returns CAN_EDRIVER every call, so an
         * unbounded drain would spin and starve the service call.
         */
        for (i = 0; i < RX_QUEUE; i++) {
            const can_sta res = hvd230_rcv(&can, &frame, 0);

            if (res == CAN_ETIMEOUT)
                break; /* queue empty */

            if (res != CAN_OK)
                continue; /* a frame the driver dropped; keep draining */

            /* handle(&frame); */
        }

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        since_health_ms += POLL_MS;
        if (since_health_ms < HEALTH_EVERY_MS)
            continue;

        since_health_ms = 0;

        if (hvd230_health(&can, &h) != CAN_OK)
            continue;

        ESP_LOGI(TAG,
                 "state=%d tec=%" PRIu32 " rec=%" PRIu32 " missed=%" PRIu32
                 " arb_lost=%" PRIu32 " bus_err=%" PRIu32,
                 (int)h.state, h.tx_err_cnt, h.rx_err_cnt, h.rx_miss_cnt,
                 h.arb_lost_cnt, h.bus_err_cnt);

        /* Nowhere to put them: raise RX_QUEUE or poll more often. */
        if (h.rx_miss_cnt > 0)
            ESP_LOGW(TAG, "rx queue overran, frames were lost");

        /* Climbing TEC precedes bus-off at 256. Check termination and
         * that every node agrees on the bitrate.
         */
        if (h.tx_err_cnt > 96)
            ESP_LOGW(TAG, "transmit errors high, bus-off is close");

        /* Normal in moderation; constantly means a saturated bus. */
        if (h.arb_lost_cnt > 0)
            ESP_LOGW(TAG, "arbitration lost %" PRIu32 " times", h.arb_lost_cnt);

        if (h.state == CAN_STATE_BUS_OFF)
            ESP_LOGE(TAG, "bus-off, rejoining in %d ms", RESTART_MS);
    }
}
