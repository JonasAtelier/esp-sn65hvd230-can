#pragma once

/* Shared between the core translation units. Not installed, not public. */

#include <stddef.h>

#include <driver/gpio.h>
#include <driver/twai.h>
#include <esp_idf_version.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <soc/soc_caps.h>

#include "hvd230can.h"

/* A target with no TWAI controller at all is a build mistake worth catching
 * early. This also fails if the soc_caps.h include above ever goes missing,
 * which is how the controller count silently fell back to 1 once already.
 *
 * There is deliberately no CAN FD check. This driver is classic-only by
 * construction -- 8-byte frames, the legacy driver API -- and FD-capable
 * silicon running that API simply runs classic. Nothing to refuse.
 */
#if !defined(SOC_TWAI_SUPPORTED) || !SOC_TWAI_SUPPORTED
#error "hvd230can needs a target with a TWAI controller."
#endif

/** How many TWAI controllers this chip has.
 *
 *  Current soc_caps.h defines this for every TWAI-capable target, 1UL on the
 *  single-controller ones. The fallback is for older headers that predate it,
 *  where assuming one is right by definition.
 */
#ifndef SOC_TWAI_CONTROLLER_NUM
#define SOC_TWAI_CONTROLLER_NUM 1
#endif

/** Fixed values only the driver uses. The ones a caller types -- bitrates,
 *  CAN_NO_PIN, CAN_NO_AUTO_RECOVER, CAN_MAX_DLC, CAN_MAX_CALLBACKS -- are in
 *  hvd230can.h instead, beside the calls that take them.
 *
 *  All of these are HARDWARE: they describe how the ESP32 TWAI registers and
 *  the SN65HVD230 are actually wired, not a preference. Changing one does not
 *  reconfigure anything, it only makes this code disagree with the silicon.
 */

/** Largest 11-bit standard identifier.
 *
 *  PROTOCOL. A standard frame carries 11 identifier bits, so 0x7FF is the
 *  ceiling. The driver screens against it and answers CAN_EINVAL; a caller
 *  never needs the number, which is why it is not in the public header.
 */
#define CAN_STD_ID_MAX 0x7FFUL

/** Largest 29-bit extended identifier.
 *
 *  PROTOCOL. An extended frame carries 29 identifier bits. The two spaces are
 *  separate: a standard 0x123 is not the same address as an extended 0x123.
 */
#define CAN_EXT_ID_MAX 0x1FFFFFFFUL

/** Transmit slots hvd230_begin() asks the TWAI driver for.
 *
 *  DEFAULT, and the ESP-IDF one. Safe to change, but hvd230_begin_ex() is the
 *  better answer: it keeps the choice next to the code that needs it. Too
 *  small shows up as CAN_EQFULL from a non-blocking send.
 */
#define CAN_DEFAULT_TX_QUEUE 5UL

/** Receive slots hvd230_begin() asks for, and the budget one
 *  hvd230_dispatch() call will drain.
 *
 *  DEFAULT, and the ESP-IDF one. This is the one people get wrong: five
 *  frames is easily overrun by a burst arriving between two polls, and the
 *  extras are simply dropped. A handful of nodes each sending at 1 kHz want
 *  32 or more. Watch rx_miss_cnt from hvd230_health() to know.
 */
#define CAN_DEFAULT_RX_QUEUE 5UL

/** Where a standard identifier sits in the TWAI acceptance filter register.
 *
 *  HARDWARE. The identifier is left-justified in a 32-bit word: bit 31 down
 *  to bit 21 for a standard frame. Get this wrong and the filter does not
 *  fail loudly -- it silently accepts nothing, which is an expensive
 *  afternoon on a bench.
 */
#define FILTER_SHIFT_STD 21

/** The same, for a 29-bit extended identifier: bit 31 down to bit 3.
 *
 *  HARDWARE. See FILTER_SHIFT_STD.
 */
#define FILTER_SHIFT_EXT 3

/** Level on the SN65HVD230's RS pin that parks it in low-current standby.
 *
 *  HARDWARE. RS high is standby, RS low is high-speed and on the bus. This is
 *  the part's own datasheet behaviour. A board that inverts the line with a
 *  transistor wants a different driver, not a different number here.
 */
#define RS_LEVEL_STANDBY 1

/** Level on RS that puts the transceiver on the bus. See RS_LEVEL_STANDBY. */
#define RS_LEVEL_ACTIVE 0

/** Shortest wait that still blocks.
 *
 *  HARDWARE-ish. pdMS_TO_TICKS() floors, so any timeout shorter than one tick
 *  period rounds to zero -- which the TWAI driver reads as "do not block at
 *  all". Only a caller asking for 0 is allowed to mean that, so a sub-tick
 *  wait is rounded up to this instead.
 */
#define MIN_BLOCKING_TICKS 1

/** One lock for the whole library, not one per object.
 *
 *  controller_claimed is shared between every instance, so a per-object lock
 *  would not protect it. There can only be one live controller anyway, so a
 *  single lock costs nothing in contention.
 *
 *  It is recursive because the public calls compose: begin unwinds through
 *  end, send_frame forwards to send, and dispatch calls receive. A plain
 *  mutex would deadlock on those.
 *
 *  The RAII guard the C++ version used is gone, so every locked function has
 *  one exit label and releases there. Never return from inside the lock.
 */
void can_lock(void);
void can_unlock(void);

/** One bit per TWAI controller that a live hvd230_t has installed.
 *
 *  A chip may have more than one -- two on the ESP32-C6, three on the P4 --
 *  and each is an independent channel. What must not happen is two handles
 *  claiming the same one, which this catches.
 *
 *  ponytail: one global lock guards every channel. It is never held across a
 *  blocking driver call, so two channels barely contend; make it per-device
 *  if a third ever shows that wrong.
 */
extern uint32_t can_claimed_mask;

/* Argument validation. Every public call screens its inputs before it reaches
 * the driver, so a bad pin, bitrate, identifier or payload is turned away as
 * CAN_EINVAL instead of being handed to the hardware.
 */
bool is_brate_valid(uint32_t bitrate);
bool is_pin_valid(int tx_pin, int rx_pin, int rs_pin);
bool is_id_valid(uint32_t id, bool extended);
bool is_msg_valid(const uint8_t *data, uint8_t len);

bool brate_to_timing(uint32_t bitrate, twai_timing_config_t *timing);
can_state can_to_state(twai_state_t state);

/** Which TWAI driver API this ESP-IDF has.
 *
 *  ESP-IDF 5.2 added the _v2 entry points, which take a per-controller
 *  handle and so can address a second controller. Before that there is one
 *  global controller and the calls take no handle at all.
 *
 *  Everything below hides that difference, so the rest of the driver is
 *  written once against the v2 shape and simply loses the second channel on
 *  an older IDF.
 */
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)
#define CAN_MULTI_CTRL 1
#else
#define CAN_MULTI_CTRL 0
#endif

#if CAN_MULTI_CTRL

#define CAN_H(can) ((twai_handle_t)(can)->handle)

static inline esp_err_t can_hw_install(twai_general_config_t *g,
                                       const twai_timing_config_t *t,
                                       const twai_filter_config_t *f,
                                       int ctrl_id, void **out)
{
    twai_handle_t h = NULL;
    esp_err_t err;

    g->controller_id = ctrl_id;
    err = twai_driver_install_v2(g, t, f, &h);
    *out = h;

    return err;
}

static inline esp_err_t can_hw_uninstall(const hvd230_t *c)
{ return twai_driver_uninstall_v2(CAN_H(c)); }
static inline esp_err_t can_hw_start(const hvd230_t *c)
{ return twai_start_v2(CAN_H(c)); }
static inline esp_err_t can_hw_stop(const hvd230_t *c)
{ return twai_stop_v2(CAN_H(c)); }
static inline esp_err_t can_hw_tx(const hvd230_t *c, const twai_message_t *m,
                                  TickType_t t)
{ return twai_transmit_v2(CAN_H(c), m, t); }
static inline esp_err_t can_hw_rx(const hvd230_t *c, twai_message_t *m,
                                  TickType_t t)
{ return twai_receive_v2(CAN_H(c), m, t); }
static inline esp_err_t can_hw_status(const hvd230_t *c,
                                      twai_status_info_t *i)
{ return twai_get_status_info_v2(CAN_H(c), i); }
static inline esp_err_t can_hw_flush(const hvd230_t *c)
{ return twai_clear_transmit_queue_v2(CAN_H(c)); }
static inline esp_err_t can_hw_recover(const hvd230_t *c)
{ return twai_initiate_recovery_v2(CAN_H(c)); }

#else /* ESP-IDF 5.0 and 5.1: one controller, no handle */

static inline esp_err_t can_hw_install(twai_general_config_t *g,
                                       const twai_timing_config_t *t,
                                       const twai_filter_config_t *f,
                                       int ctrl_id, void **out)
{
    (void)ctrl_id;              /* only controller 0 exists here */
    *out = NULL;
    return twai_driver_install(g, t, f);
}

static inline esp_err_t can_hw_uninstall(const hvd230_t *c)
{ (void)c; return twai_driver_uninstall(); }
static inline esp_err_t can_hw_start(const hvd230_t *c)
{ (void)c; return twai_start(); }
static inline esp_err_t can_hw_stop(const hvd230_t *c)
{ (void)c; return twai_stop(); }
static inline esp_err_t can_hw_tx(const hvd230_t *c, const twai_message_t *m,
                                  TickType_t t)
{ (void)c; return twai_transmit(m, t); }
static inline esp_err_t can_hw_rx(const hvd230_t *c, twai_message_t *m,
                                  TickType_t t)
{ (void)c; return twai_receive(m, t); }
static inline esp_err_t can_hw_status(const hvd230_t *c,
                                      twai_status_info_t *i)
{ (void)c; return twai_get_status_info(i); }
static inline esp_err_t can_hw_flush(const hvd230_t *c)
{ (void)c; return twai_clear_transmit_queue(); }
static inline esp_err_t can_hw_recover(const hvd230_t *c)
{ (void)c; return twai_initiate_recovery(); }

#endif

/* pdMS_TO_TICKS() floors, so any timeout below one tick period would become a
 * non-blocking call. Only a t_ms of 0 is allowed to mean do not block.
 */
TickType_t ms_to_ticks(uint32_t t_ms);
