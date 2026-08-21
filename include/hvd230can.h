#pragma once

/** Classic CAN for the ESP32 TWAI controller and a TI SN65HVD230 transceiver.
 *
 *  One chipset, deliberately: no ops tables, no transceiver descriptors, no
 *  indirection. If you need a different controller, write a different library.
 *
 *  Pin names, logic levels and mode behaviour below come from the
 *  SN65HVD230 datasheet, doc/sn65hvd230.pdf. That is the authority -- a
 *  breakout board may not bring every pin out, but it cannot change what a
 *  pin does.
 *
 *  How to use it
 *  -------------
 *      hvd230_t can;
 *
 *      hvd230_init(&can);                                   // always first
 *      hvd230_begin(&can, TX_PIN, RX_PIN, 500000);
 *
 *      uint8_t payload[3] = {1, 2, 3};
 *      hvd230_tx(&can, 0x123, payload, 3, false, 0);
 *
 *      can_frame_t frame;
 *      if (hvd230_rcv(&can, &frame, 0) == CAN_OK)
 *              use(&frame);
 *
 *      hvd230_end(&can);                                    // always last
 *
 *  The full order, with the optional parts in brackets:
 *
 *      init -> [set_filter] -> begin -> [set_auto_recover]
 *           -> [on_frame ...]
 *           -> loop { [service] ; receive or dispatch ; send }
 *           -> end
 *
 *  Five things worth knowing before you start
 *  ------------------------------------------
 *  1. There is no destructor. hvd230_init() before the first call and
 *     hvd230_end() on every exit path, or the controller stays claimed for
 *     the rest of the program. One handle owns it at a time.
 *  2. timeout_ms of 0 means do not block. Receive then answers CAN_ETIMEOUT
 *     when the queue is empty, which is normal and not an error; send
 *     answers CAN_EQFULL when the transmit queue is full.
 *  3. Nothing runs in the background. Frames reach the driver queue on their
 *     own and sit there. Your loop rate decides whether the queue overruns.
 *  4. Use hvd230_rcv() or hvd230_dispatch(), never both: they take from
 *     one queue, and whichever runs first gets the frame.
 *  5. Every call is safe from any task. The lock is not held across a
 *     blocking driver call or while your callbacks run, and it does not
 *     protect whatever you share through ctx.
 */

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------- constants
 *
 * Only the ones you actually type, or that size a struct below. The rest --
 * identifier ceilings, queue defaults, register layouts -- are the driver's
 * business and live in src/internal.h.
 *
 * Everything here is fixed by the CAN standard or the hardware. Changing one
 * reconfigures nothing; it only makes this driver disagree with the silicon,
 * and the failure is usually silence on the bus rather than an error you can
 * see. Treat them as read-only.
 */

/** Bytes a classic CAN frame can carry. Longer is CAN_EINVAL.
 *
 *  Fixed at 8 by classic CAN. CAN FD raised it to 64, but this driver is
 *  classic-only by construction, so 8 is the whole story here.
 */
#define CAN_MAX_DLC 8

/** The bitrates this driver can produce. Pass one to hvd230_begin().
 *
 *  These are the timings ESP-IDF ships for the TWAI controller; any other
 *  value is CAN_EINVAL. The driver will not quietly round to a near one,
 *  because a node at the wrong bitrate does not go silent -- it corrupts the
 *  bus for everyone else.
 *
 *  Every node must agree, and plenty of devices have no say in it: a
 *  fixed-rate peripheral sets the rate for the whole wire.
 */
#define CAN_BITRATE_25K   25000UL
#define CAN_BITRATE_50K   50000UL
#define CAN_BITRATE_100K  100000UL
#define CAN_BITRATE_125K  125000UL
#define CAN_BITRATE_250K  250000UL
#define CAN_BITRATE_500K  500000UL
#define CAN_BITRATE_800K  800000UL
#define CAN_BITRATE_1M    1000000UL

/** Pass as rs_pin to hvd230_begin_ex() when the RS line is not on a GPIO --
 *  tied to ground, or left to the transceiver's own pull.
 *
 *  -1 is simply "not a GPIO"; every real ESP32 pin is >= 0. Without an RS
 *  pin, hvd230_standby() and hvd230_wake() answer CAN_ENOPIN.
 */
#define CAN_NO_PIN (-1)

/** Pass to hvd230_set_auto_recover() to disable bus-off recovery, which is
 *  what a fresh handle already has.
 *
 *  Off on purpose: a bus that keeps failing is usually a wiring fault, and
 *  retrying forever hides it. Linux SocketCAN defaults its restart-ms off for
 *  the same reason.
 */
#define CAN_NO_AUTO_RECOVER 0UL

/** How many frame callbacks hvd230_on_frame() will take.
 *
 *  The table is fixed so that registering a handler never allocates. It sizes
 *  hvd230_t below, so raising it costs RAM in every handle you place. Past
 *  this many, hvd230_on_frame() answers CAN_ENOSLOT rather than dropping a
 *  handler on the floor.
 */
#define CAN_MAX_CALLBACKS 8

/* ------------------------------------------------------------------ types */

/** Result of a driver operation. CAN_OK is the only success. */
typedef enum {
    CAN_OK,
    CAN_ETIMEOUT,     /* a timed send or receive expired, or no frame waiting */
    CAN_EINVAL,       /* bad pin, bitrate, identifier, payload or length */
    CAN_ENOTSTARTED,  /* begin has not succeeded, or end was called */
    CAN_ESTANDBY,     /* send requested while the transceiver is in standby */
    CAN_ENOPIN,       /* standby/wake without a configured RS GPIO */
    CAN_EQFULL,       /* a non-blocking send found the transmit queue full */
    CAN_EDRIVER,      /* the TWAI driver failed, or dropped the frame */
    CAN_ENOSLOT,      /* no free callback slot left */
} can_sta;

/** Controller state, as reported by the hardware. Read it with
 *  hvd230_health(); recover from CAN_STATE_BUS_OFF with hvd230_service().
 */
typedef enum {
    CAN_STATE_STOPPED,     /* installed but not running */
    CAN_STATE_RUNNING,     /* on the bus */
    CAN_STATE_BUS_OFF,     /* too many transmit errors; off the bus entirely */
    CAN_STATE_RECOVERING,  /* waiting out the bus-off recovery sequence */
} can_state;

/** One classic CAN frame. len is 0 to CAN_MAX_DLC; longer is CAN_EINVAL.
 *  extended picks the 29-bit identifier space over the 11-bit one.
 */
typedef struct {
    uint32_t id;
    uint8_t data[CAN_MAX_DLC];
    uint8_t len;
    bool extended;
} can_frame_t;

/** Your frame handler, registered with hvd230_on_frame().
 *
 *  Runs inside hvd230_dispatch(), on the thread that called it -- never from
 *  an interrupt or a driver task, so it may do anything an ordinary function
 *  may, including calling hvd230_tx(). ctx is whatever you registered.
 *
 *  frame is yours only for the duration of the call. Copy what you need.
 */
typedef void (*can_rx_cb_t)(const can_frame_t *frame, void *ctx);

/** Controller health, filled in by hvd230_health().
 *
 *  tx_err_cnt climbing while frames still arrive is the warning that precedes
 *  bus-off: check termination, wiring, and that every node agrees on the
 *  bitrate. rx_miss_cnt above zero means your rx_queue is too small or your
 *  loop too slow.
 */
typedef struct {
    can_state state;
    uint32_t tx_err_cnt;    /* TEC; 256 means bus-off */
    uint32_t rx_err_cnt;    /* REC */
    uint32_t tx_fail_cnt;
    uint32_t rx_miss_cnt;   /* frames dropped because the queue was full */
    uint32_t arb_lost_cnt;  /* lost arbitration; high means a busy bus */
    uint32_t bus_err_cnt;
    uint32_t tx_pending;    /* frames still queued to transmit */
    uint32_t rx_pending;    /* frames waiting to be read */
} can_health_t;

/* One entry of the callback table. Private. */
typedef struct {
    uint32_t id;
    uint32_t msk;
    can_rx_cb_t cb;
    void *ctx;
} hvd230_sub_t;

/** The driver handle. Declared here only so you can place one -- as a local,
 *  a static, or wherever you like. Never read or write a field yourself.
 */
typedef struct hvd230 {
    void *handle;           /* twai_handle_t, opaque here on purpose */
    int ctrl_id;            /* which TWAI controller, 0 by default */
    int rs_pin;
    bool is_installed;
    bool is_started;
    bool is_standby;

    hvd230_sub_t subs[CAN_MAX_CALLBACKS];
    uint8_t sub_cnt;
    uint32_t rx_queue;      /* drives the hvd230_dispatch() drain budget */

    uint32_t filter_id;
    uint32_t filter_mask;
    bool is_filter_ext;
    bool is_filter_set;     /* false accepts every frame */

    uint32_t restart_ms;    /* 0 disables automatic recovery */
    uint32_t bus_off_tick;  /* tick at which bus-off was first seen */
    bool is_bus_off_seen;   /* bus_off_tick holds a real timestamp */
    bool is_recovering;     /* twai_initiate_recovery() has been issued */
} hvd230_t;

/* ------------------------------------------------------------ starting up */

/** Prepare a handle. Call this before anything else, including
 *  hvd230_set_filter().
 *
 *  @param  can   the handle to prepare
 *  @note   A zeroed struct is not a valid one, so this is not optional.
 */
void hvd230_init(hvd230_t *can);

/** Pick which TWAI controller this handle drives.
 *
 *  Most ESP32 parts have one, and the default of 0 is it. Some have more --
 *  the ESP32-C6 has two, the P4 three -- and each is an independent CAN
 *  channel with its own pins, bitrate, filter and queues. Place one hvd230_t
 *  per channel.
 *
 *  @param  id  controller index, from 0
 *  @retval CAN_OK      -> stored, and used by the next hvd230_begin()
 *          CAN_EINVAL  -> negative id, or more than this chip has
 *          CAN_EDRIVER -> already begun, too late to move it
 *  @note   Before begin, like the filter.
 *  @note   Needs ESP-IDF 5.2 or newer for any id but 0, which is where the
 *          multi-controller driver API arrived.
 */
can_sta hvd230_set_controller(hvd230_t *can, int id);

/** Claim the controller and go on the bus, with the defaults.
 *
 *  @param  tx_pin   GPIO to the transceiver's D pin
 *  @param  rx_pin   GPIO from the transceiver's R pin
 *  @param  bitrate  one of the CAN_BITRATE_* values above. Every node on the
 *                   bus must agree, or you get errors, not silence.
 *  @retval CAN_OK           -> on the bus, ready to send and receive
 *          CAN_EINVAL       -> bad pin or an unsupported bitrate
 *          CAN_EDRIVER      -> already begun, or the TWAI driver refused
 *  @note   No RS pin, and five queue slots either way -- the ESP-IDF default.
 *          Use hvd230_begin_ex() to choose them yourself.
 */
can_sta hvd230_begin(hvd230_t *can, int tx_pin, int rx_pin, uint32_t bitrate);

/** Claim the controller and go on the bus, spelling out everything.
 *
 *  @param  rs_pin    GPIO to the SN65HVD230's RS pin (8), or CAN_NO_PIN if
 *                    it is tied low or your board does not route it. Needed
 *                    by hvd230_standby() and hvd230_wake().
 *  @param  tx_queue  transmit slots inside the TWAI driver, must be > 0
 *  @param  rx_queue  receive slots, must be > 0. The default of five is
 *                    easily overrun by a burst arriving between two polls:
 *                    size it against the worst case, not the average rate,
 *                    and read rx_miss_cnt to see whether you guessed right.
 *  @retval as hvd230_begin()
 *  @note   Begin leaves the transceiver awake, RS low and on the bus.
 */
can_sta hvd230_begin_ex(hvd230_t *can, int tx_pin, int rx_pin, uint32_t bitrate,
                        int rs_pin, uint32_t tx_queue, uint32_t rx_queue);

/** Leave the bus and hand the controller back.
 *
 *  @note   Nothing calls this for you. Every hvd230_begin() needs one, on
 *          every exit path, or nothing can begin again for the rest of the
 *          program. Safe to call twice. A node in bus-off cannot end
 *          cleanly -- recover it first.
 */
can_sta hvd230_end(hvd230_t *can);

/* -------------------------------------------------------- send and receive */

/** Queue one frame for transmission.
 *
 *  @param  id         11-bit identifier, or 29-bit when extended is true
 *  @param  data       payload, may be NULL only when len is 0
 *  @param  len        0 to CAN_MAX_DLC
 *  @param  extended   false for a standard frame, true for an extended one
 *  @param  timeout_ms how long to wait for a free queue slot; 0 = do not wait
 *  @retval CAN_OK         -> queued. NOT delivered, and NOT acknowledged
 *          CAN_EQFULL     -> queue full and timeout_ms was 0
 *          CAN_ETIMEOUT   -> queue still full when the timeout expired
 *          CAN_EINVAL     -> bad identifier, length, or a NULL payload
 *          CAN_ESTANDBY   -> the transceiver is parked, call hvd230_wake()
 *          CAN_ENOTSTARTED
 */
can_sta hvd230_tx(hvd230_t *can, uint32_t id, const uint8_t *data,
                  uint8_t len, bool extended, uint32_t timeout_ms);

/** The same call, taking a can_frame_t you already have. */
can_sta hvd230_tx_frame(hvd230_t *can, const can_frame_t *frame,
                        uint32_t timeout_ms);

/** Read one frame from the receive queue.
 *
 *  @param  frame      filled in on CAN_OK, untouched otherwise
 *  @param  timeout_ms how long to wait for a frame; 0 = do not wait
 *  @retval CAN_OK        -> frame holds a received frame
 *          CAN_ETIMEOUT  -> nothing was waiting. Normal, not an error
 *          CAN_EDRIVER   -> the driver dropped one, e.g. a remote frame
 *          CAN_ENOTSTARTED
 *  @note   Bound your drain loop. While the controller is bus-off this
 *          answers CAN_EDRIVER every time, never CAN_ETIMEOUT, so a loop that
 *          only stops on CAN_ETIMEOUT spins forever:
 *
 *              for (int i = 0; i < RX_QUEUE; i++) {
 *                      can_sta r = hvd230_rcv(&can, &frame, 0);
 *                      if (r == CAN_ETIMEOUT) break;
 *                      if (r != CAN_OK) continue;
 *                      use(&frame);
 *              }
 */
can_sta hvd230_rcv(hvd230_t *can, can_frame_t *frame, uint32_t timeout_ms);

/** Throw away everything queued to transmit that has not reached the bus.
 *
 *  @note   For anything whose value expires -- a setpoint, a heartbeat -- a
 *          late frame is worse than no frame. Drop the backlog and queue a
 *          fresh one rather than letting stale commands go out. Only worth it
 *          when tx_pending stays high or tx_fail_cnt is rising; flushing a
 *          healthy queue throws away frames that were about to go.
 */
can_sta hvd230_flush_tx(hvd230_t *can);

/* -------------------------------------------------------------- filtering */

/** Drop non-matching frames in hardware, before they cost an interrupt.
 *
 *  Matching is bitwise, not a range: (frame.id & mask) == id. So id 0x200
 *  with mask 0x7F0 takes 0x200 through 0x20F.
 *
 *  @retval CAN_OK      -> stored, and applied by the next hvd230_begin()
 *          CAN_EINVAL  -> the identifier does not fit, or (id & mask) != id,
 *                         which would match nothing at all
 *          CAN_EDRIVER -> already begun, too late to change it
 *  @note   Must come BEFORE begin: the TWAI driver takes the filter when the
 *          peripheral is installed and cannot change it while running.
 *          Without one, every frame on the bus is accepted.
 *  @note   One filter only. For disjoint ranges, widen the mask here and sort
 *          the rest out with hvd230_on_frame().
 */
can_sta hvd230_set_filter(hvd230_t *can, uint32_t id, uint32_t mask, bool extended);

/** Forget the filter, so every frame is accepted again. Before begin only. */
can_sta hvd230_clear_filter(hvd230_t *can);

/* -------------------------------------------------------------- callbacks */

/** Route matching frames to cb instead of switching on the identifier.
 *
 *  @param  id, mask  same bitwise rule as the hardware filter:
 *                    (frame.id & mask) == id. A mask of 0 matches every
 *                    frame, which is how you register a catch-all.
 *  @param  ctx       handed back to cb untouched; NULL if it needs nothing
 *  @retval CAN_OK      -> registered
 *          CAN_EINVAL  -> cb was NULL
 *          CAN_ENOSLOT -> the table is full, see CAN_MAX_CALLBACKS
 *  @note   Every matching handler fires, so a catch-all and a specific one
 *          happily coexist. Handlers run in hvd230_dispatch(), not here.
 */
can_sta hvd230_on_frame(hvd230_t *can, uint32_t id, uint32_t mask,
                        can_rx_cb_t cb, void *ctx);

/** Empty the callback table. */
void hvd230_clear_callbacks(hvd230_t *can);

/** Drain the receive queue, handing each frame to every callback it matches.
 *
 *  @param  timeout_ms  passed to each underlying read; 0 = do not wait
 *  @retval CAN_OK      -> the queue was drained
 *          CAN_EDRIVER -> at least one frame was dropped; the rest still ran
 *          CAN_ENOTSTARTED
 *  @note   Callbacks run here, on this thread, before the call returns.
 *  @note   Bounded internally to one queue's worth, so a flooded or bus-off
 *          controller cannot spin here and starve hvd230_service().
 *  @note   Use this OR hvd230_rcv(), not both -- one queue, first call
 *          wins.
 */
can_sta hvd230_dispatch(hvd230_t *can, uint32_t timeout_ms);

/* ---------------------------------------------------- faults and recovery */

/** Read the controller state and error counters.
 *
 *  @param  out  filled in on CAN_OK
 *  @retval CAN_OK, CAN_EINVAL, CAN_ENOTSTARTED, CAN_EDRIVER
 */
can_sta hvd230_health(const hvd230_t *can, can_health_t *out);

/** How long to stay off the bus after bus-off before rejoining.
 *
 *  @param  restart_ms  CAN_NO_AUTO_RECOVER disables it, which is the default
 *  @note   This only stores the delay. hvd230_service() is what actually
 *          recovers -- you need both. Off by default because a bus that keeps
 *          failing is usually a wiring fault, and retrying forever hides it.
 */
void hvd230_set_auto_recover(hvd230_t *can, uint32_t restart_ms);

/** Advance bus-off recovery by one step. Call it in your loop.
 *
 *  After 256 transmit errors the controller leaves the bus and never returns
 *  on its own -- one disconnected CAN_H does it, and every send reports
 *  CAN_EDRIVER forever after. This walks the way back: wait out restart_ms,
 *  ask the controller to recover, then start it again once it has.
 *
 *  @retval CAN_OK      -> the bus is healthy, or the node has just rejoined
 *          CAN_EDRIVER -> bus-off or still recovering, keep calling
 *          CAN_ENOTSTARTED
 *  @note   One edge per call, never blocks, and costs one status read while
 *          the bus is fine. Does nothing until hvd230_set_auto_recover()
 *          enables it.
 */
can_sta hvd230_service(hvd230_t *can);

/* ------------------------------------------------------ transceiver power */

/** Park the transceiver in low-current standby (RS high).
 *
 *  @retval CAN_OK, CAN_ENOPIN (no RS GPIO given to begin), CAN_ENOTSTARTED
 *  @note   The controller stays on the bus; only the transceiver stops
 *          driving it. Sending while parked answers CAN_ESTANDBY.
 */
can_sta hvd230_standby(hvd230_t *can);

/** Bring the transceiver back out of standby (RS low). */
can_sta hvd230_wake(hvd230_t *can);

#ifdef __cplusplus
}
#endif
