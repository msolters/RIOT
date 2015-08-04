/*
 * Copyright (C) 2015 Daniel Krebs
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    net_lwmac Simplest possible MAC layer
 * @ingroup     net
 * @brief       Lightweight MAC protocol that allows for duty cycling to save
 *              energy.
 * @{
 *
 * @file
 * @brief       Interface definition for the LWMAC protocol
 *
 * @author      Daniel Krebs <github@daniel-krebs.net>
 */

// TODO: cleanup type definitions, maybe move to other files

#ifndef LWMAC_H_
#define LWMAC_H_

#include "kernel.h"
#include "vtimer.h"
#include "net/gnrc/pkt.h"
#include "net/gnrc/netdev.h"
#include "net/gnrc/lwmac/packet_queue.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Set the default message queue size for LWMAC layer
 */
#ifndef LWMAC_IPC_MSG_QUEUE_SIZE
#define LWMAC_IPC_MSG_QUEUE_SIZE        (8U)
#endif

/**
 * @brief   Count of parallel timeouts. Shouldn't needed to be changed.
 */
#ifndef LWMAC_TIMEOUT_COUNT
#define LWMAC_TIMEOUT_COUNT             (2U)
#endif

/**
 * @brief   Set the default queue size for packets coming from higher layers
 */
#ifndef LWMAC_TX_QUEUE_SIZE
#define LWMAC_TX_QUEUE_SIZE             (8U)
#endif

#ifndef LWMAC_WAKEUP_INTERVAL_MS
#define LWMAC_WAKEUP_INTERVAL_MS        (100U)          /* 100 ms */
#endif

#ifndef LWMAC_TIME_BETWEEN_WR_US
#define LWMAC_TIME_BETWEEN_WR_US        (7000U)         /* 5000 us */
#endif

#ifndef LWMAC_WAKEUP_DURATION_MS
#define LWMAC_WAKEUP_DURATION_MS        (LWMAC_TIME_BETWEEN_WR_US / 1000 * 2)
#endif

#ifndef LWMAC_DATA_DELAY_US
#define LWMAC_DATA_DELAY_US             (50000U)        /* 50 ms */
#endif

#define LWMAC_EVENT_RTT_TYPE            (0x4300)
#define LWMAC_EVENT_RTT_START           (0x4301)
#define LWMAC_EVENT_RTT_STOP            (0x4302)
#define LWMAC_EVENT_RTT_PAUSE           (0x4303)
#define LWMAC_EVENT_RTT_RESUME          (0x4304)
#define LWMAC_EVENT_RTT_WAKEUP_PENDING  (0x4305)
#define LWMAC_EVENT_RTT_SLEEP_PENDING   (0x4306)
#define LWMAC_EVENT_TIMEOUT_TYPE        (0x4400)

typedef enum {
    UNDEF = -1,
    STOPPED,
    START,
    STOP,
    RESET,
    LISTENING,
    RECEIVING,      /* RX is handled in own state machine */
    TRANSMITTING,   /* TX is handled in own state machine */
    SLEEPING,
    STATE_COUNT
} lwmac_state_t;

typedef enum {
    TX_STATE_STOPPED = 0,
    TX_STATE_INIT,          /**< Initiate transmission */
    TX_STATE_SEND_WR,       /**< Send a wakeup request */
    TX_STATE_WAIT_FOR_WA,   /**< Wait for dest node's wakeup ackknowledge */
    TX_STATE_SEND_DATA,     /**< Send the actual payload data */
    TX_STATE_WAIT_FEEDBACK, /**< Wait if packet was ACKed */
    TX_STATE_SUCCESSFUL,    /**< Transmission has finished successfully */
    TX_STATE_FAILED         /**< Payload data couldn't be delivered to dest */
} lwmac_tx_state_t;
#define LWMAC_TX_STATE_INIT TX_STATE_STOPPED

typedef enum {
    RX_STATE_STOPPED = 0,
    RX_STATE_INIT,          /**< Initiate reception */
    RX_STATE_WAIT_FOR_WR,   /**< Wait for a wakeup request */
    RX_STATE_SEND_WA,       /**< Send wakeup ackknowledge to requesting node */
    RX_STATE_WAIT_FOR_DATA, /**< Wait for actual payload data */
    RX_STATE_SUCCESSFUL,    /**< Recption has finished successfully */
    RX_STATE_FAILED         /**< Reception over, but nothing received */
} lwmac_rx_state_t;
#define LWMAC_RX_STATE_INIT RX_STATE_STOPPED

typedef enum {
    TIMEOUT_DISABLED = 0,
    TIMEOUT_WR,
    TIMEOUT_NO_RESPONSE,
    TIMEOUT_WA,
    TIMEOUT_DATA
} lwmac_timeout_type_t;

typedef struct {
    /* Timer used for timeouts */
    vtimer_t timer;
    /* When to expire */
    timex_t interval;
    /* If type != DISABLED, this indicates if timeout has expired */
    bool expired;
    /* Lastest timeout that occurred and hasn't yet been acknowledged */
    lwmac_timeout_type_t type;
} lwmac_timeout_t;

typedef enum {
    TX_FEEDBACK_UNDEF = -1,
    TX_FEEDBACK_SUCCESS,
    TX_FEEDBACK_NOACK,
    TX_FEEDBACK_BUSY
} lwmac_tx_feedback_t;

typedef struct {
    /* Internal state of reception state machine */
    lwmac_rx_state_t state;
    packet_queue_t queue;
    gnrc_pktsnip_t* packet;
} lwmac_rx_t;

typedef struct {
    /* Internal state of transmission state machine */
    lwmac_tx_state_t state;
    packet_queue_t queue;
    uint32_t wr_sent;
    /* Packet that is currently scheduled to be sent */
    gnrc_pktsnip_t* packet;
} lwmac_tx_t;


typedef struct {
    /* PID of lwMAC thread */
    kernel_pid_t pid;
    /* NETDEV device used by lwMAC */
    gnrc_netdev_t* netdev;
    /* Internal state of MAC layer */
    lwmac_state_t state;
    /* Don't start sending while receiving in progress */
    bool rx_in_progress;
    /* Own address */
    uint64_t addr;
    unsigned int addr_len;
    lwmac_rx_t rx;
    lwmac_tx_t tx;
    /* Feedback of last packet that was sent */
    lwmac_tx_feedback_t tx_feedback;
    /* Store timeouts used for protocol */
    lwmac_timeout_t timeouts[LWMAC_TIMEOUT_COUNT];
    /* Used to calculate wakeup times */
    uint32_t last_wakeup;
    /* Used internally for rescheduling state machine update, e.g. after state
     * transition caused in update */
    bool needs_rescheduling;
} lwmac_t;

#define LWMAC_INIT { \
/* pid */                   KERNEL_PID_UNDEF,  \
/* netdev */                NULL, \
/* state */                 UNDEF, \
/* rx_in_progress */        false, \
/* addr */                  0, \
/* addr_len */              0, \
/* rx::state */             { LWMAC_RX_STATE_INIT, \
/* rx::queue */               {}, \
/* rx::packet */              NULL }, \
/* tx::state */             { LWMAC_TX_STATE_INIT, \
/* tx::queue */               {}, \
/* tx::wr_sent */             0, \
/* tx::packet */              NULL }, \
/* tx_feedback */           TX_FEEDBACK_UNDEF, \
/* timeouts */              {}, \
/* last_wakeup */           0, \
/* needs_rescheduling */    false \
}

typedef enum {
    FRAMETYPE_WR = 1,
    FRAMETYPE_WA,
    FRAMETYPE_DATA
} lwmac_frame_type_t;

/**
 * @brief   lwMAC header
 */
typedef struct __attribute__((packed)) {
    lwmac_frame_type_t type;    /**< type of frame */
    bool data_pending;          /**< is there more to send? */
} lwmac_hdr_t;

/**
 * @brief   Initialize an instance of the LWMAC layer
 *
 * The initialization starts a new thread that connects to the given netdev
 * device and starts a link layer event loop.
 *
 * @param[in] stack         stack for the control thread
 * @param[in] stacksize     size of *stack*
 * @param[in] priority      priority for the thread housing the LWMAC instance
 * @param[in] name          name of the thread housing the LWMAC instance
 * @param[in] dev           netdev device, needs to be already initialized
 *
 * @return                  PID of LWMAC thread on success
 * @return                  -EINVAL if creation of thread fails
 * @return                  -ENODEV if *dev* is invalid
 */
kernel_pid_t gnrc_lwmac_init(char *stack, int stacksize, char priority,
                           const char *name, gnrc_netdev_t *dev);

/* @brief Extract the destination address out of an GNRC_NETTYPE_NETIF pktsnip
 *
 * @param[in]   pkt                 pktsnip from whom to extract
 * @param[out]  pointer_to_addr     pointer to address will be stored here
 *
 * @return                          length of destination address
 */
int _get_dest_address(gnrc_pktsnip_t* pkt, uint8_t* pointer_to_addr[]);

/* @brief Find the first pktsnip of @p type
 *
 * Will search linearly through the packet buffer @p pkt and yield
 * gnrc_pktsnip_t::data of the first pktsnip match the type @p type.
 *
 * @param[in]   pkt     pktsnip that will be searched
 * @param[in]   type    type to search for
 *
 * @return              pointer to data, NULL is not found
 */
void* _gnrc_pktbuf_find(gnrc_pktsnip_t* pkt, gnrc_nettype_t type);

/* @brief Check packet if address matches and is of expected type
 *
 * Check if @p pkt type is @p expected_type and then check if destination
 * address matches own address in @p lwmac.
 *
 * TODO: also accept broadcast
 *
 * @param[in]   pkt             packet that will be checked
 * @param[in]   expected_type   required type
 * @param[in]   lwmac           lwmac state that stores own address
 *
 * @return                      whether the packet should be accepted or not
 */
bool _accept_packet(gnrc_pktsnip_t* pkt, lwmac_frame_type_t expected_type, lwmac_t* lwmac);

/* @brief Shortcut to get the state of netdev
 *
 * @param[in]   lwmac           lwmac state that stores netdev pointer
 *
 * @return                      state of netdev
 */
netopt_state_t _get_netdev_state(lwmac_t* lwmac);

#ifdef __cplusplus
}
#endif

#endif /* LWMAC_H_ */
/** @} */
