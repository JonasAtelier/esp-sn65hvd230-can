/* Two independent CAN channels on one chip.
 *
 * Some parts have more than one TWAI controller -- two on the C6, three on
 * the P4. Each is a separate bus with its own pins, bitrate, filter, queues
 * and error state. One hvd230_t each, and pick the controller before begin.
 *
 * Controller 0 is the default, so single-channel code never mentions it.
 * Needs ESP-IDF 5.2 or newer; before that only controller 0 exists.
 *
 * Not a buildable ESP-IDF example project, just the call sequence.
 */

#include <inttypes.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hvd230can.h"
#include "soc/soc_caps.h"

#ifndef SOC_TWAI_CONTROLLER_NUM
#define SOC_TWAI_CONTROLLER_NUM 1
#endif

#if SOC_TWAI_CONTROLLER_NUM < 2
#error "This target has one TWAI controller. Use a single hvd230_t."
#endif

/* Channel A: fast, and filtered down to one range of identifiers */
#define A_CTRL     0
#define A_TX_PIN   5
#define A_RX_PIN   4
#define A_BITRATE  CAN_BITRATE_1M
#define A_ID       0x200
#define A_MASK     0x7F0

/* Channel B: slower, unfiltered. Nothing has to match channel A. */
#define B_CTRL     1
#define B_TX_PIN   7
#define B_RX_PIN   6
#define B_BITRATE  CAN_BITRATE_500K

#define POLL_MS         10
#define REPORT_EVERY_MS 1000

static const char *TAG = "dual";

static uint32_t a_frames, b_frames;

static void on_a(const can_frame_t *frame, void *ctx)
{
    (void)frame; (void)ctx;
    a_frames++;
}

static void on_b(const can_frame_t *frame, void *ctx)
{
    (void)frame; (void)ctx;
    b_frames++;
}

/* Both channels open the same way; only id, pins and bitrate differ. */
static bool open_channel(hvd230_t *can, int ctrl, int tx, int rx,
                         uint32_t bitrate, can_rx_cb_t cb)
{
    hvd230_init(can);

    if (hvd230_set_controller(can, ctrl) != CAN_OK) {
        ESP_LOGE(TAG, "no TWAI controller %d on this chip", ctrl);
        return false;
    }

    /* Per channel, before begin */
    if (ctrl == A_CTRL)
        hvd230_set_filter(can, A_ID, A_MASK, false);

    if (hvd230_begin_ex(can, tx, rx, bitrate, CAN_NO_PIN, 8, 32) != CAN_OK) {
        ESP_LOGE(TAG, "controller %d failed to start", ctrl);
        return false;
    }

    hvd230_set_auto_recover(can, 100);
    hvd230_on_frame(can, 0, 0, cb, NULL);   /* mask 0 = every frame */

    return true;
}

/* Each channel keeps its own error counters, so a fault on one says nothing
 * about the other. That separation is the reason to use two in the first
 * place: a wiring fault on B must not take A's motors down.
 */
static void report(const char *name, hvd230_t *can, uint32_t frames)
{
    can_health_t h;

    if (hvd230_health(can, &h) != CAN_OK)
        return;

    ESP_LOGI(TAG, "%s: %" PRIu32 " frames/s, state=%d tec=%" PRIu32
                  " rx_miss=%" PRIu32,
             name, frames, (int)h.state, h.tx_err_cnt, h.rx_miss_cnt);

    if (h.state == CAN_STATE_BUS_OFF)
        ESP_LOGE(TAG, "%s is bus-off -- check that bus, the other is fine",
                 name);
}

void app_main(void)
{
    uint32_t since_report_ms = 0;
    hvd230_t bus_a, bus_b;

    if (!open_channel(&bus_a, A_CTRL, A_TX_PIN, A_RX_PIN, A_BITRATE, on_a))
        return;

    if (!open_channel(&bus_b, B_CTRL, B_TX_PIN, B_RX_PIN, B_BITRATE, on_b)) {
        hvd230_end(&bus_a);
        return;
    }

    ESP_LOGI(TAG, "A on controller %d at %" PRIu32 ", B on %d at %" PRIu32,
             A_CTRL, (uint32_t)A_BITRATE, B_CTRL, (uint32_t)B_BITRATE);

    while (true) {
        /* Nothing runs in the background, so both want servicing. */
        hvd230_service(&bus_a);
        hvd230_service(&bus_b);

        hvd230_dispatch(&bus_a, 0);
        hvd230_dispatch(&bus_b, 0);

        vTaskDelay(pdMS_TO_TICKS(POLL_MS));

        since_report_ms += POLL_MS;
        if (since_report_ms < REPORT_EVERY_MS)
            continue;

        since_report_ms = 0;

        report("A", &bus_a, a_frames);
        report("B", &bus_b, b_frames);
        a_frames = b_frames = 0;
    }

    hvd230_end(&bus_a);
    hvd230_end(&bus_b);
}
